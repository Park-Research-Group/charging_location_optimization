# Multiprocessing for iteration-0 shortest-path assignment (Wave 2,
# option B). The main process keeps the sequential bucket walk
# (guards, gen_rng draws, key extraction) and farms the distinct SP
# keys to a spawn Pool; workers attach the SP table from shared
# memory and run the same find_best_path, so results are identical
# to the serial run. See loading.assign_paths_iteration_zero.
#
# How it works
# - resolve_keys_parallel(world, net, tasks, workers) builds compact
#   topology arrays once (CSR outgoing lists, link from/to, the
#   connector-link flag, node zones) plus the small scalars
#   find_best_path needs (start/end/horizon/num_periods and the
#   assignment-interval index table), and puts net.td_time in a
#   multiprocessing.shared_memory block. When the table is uniform
#   across intervals (the usual iteration-0 case: every interval
#   holds the same free-flow value) only its first column is shared
#   and workers view it broadcast back to the full shape; otherwise
#   the full table is shared. The uniformity check is a chunked
#   full-table comparison, not a sample: sharing one column when
#   the columns differ would change results.
# - Each worker rebuilds a lightweight world/net shim from those
#   arrays once, in the initializer, then answers keys with the
#   real find_best_path from sp.py. The shim mirrors the World
#   attributes find_best_path reads (nodes with outgoing lists and
#   zone ids, links with from/to/type_code, interval_index, start,
#   end, horizon) and nothing else, so the search itself is the
#   same code on the same data.
# - Results stream back with imap_unordered and are reassembled
#   into a dict keyed by (origin_zone, origin, dest_zone, dest);
#   the main process assigns them to legs in first-seen key order.
#   Workers need no RNG: every gen_rng draw stays in the main
#   process's sequential walk.
#
# Fallbacks (a pool failure never fails a run that would have
# worked serially): resolve_keys_parallel returns None when the
# pool is not wanted (workers <= 1, one CPU, or too few keys for
# the automatic mode) or when anything about the pool fails
# (creation, worker init, a worker crash, an incomplete result
# set). The caller then resolves the same keys serially, with the
# RNG draws already done, so the outputs are unchanged. The
# tripwire-tripped (time-varying table) case never reaches the pool
# at all: those keys depend on their own departure time, so they
# stay on the old per-leg serial path.
#
# Windows / spawn notes (Eli runs Python 3.9 on Windows, where
# spawn is the default start method; this module asks for spawn
# explicitly so Linux behaves the same way)
# - The pool is created inside run_pysim's call stack, never at
#   import time, so a plain `python -m pysim.run` invocation works
#   unchanged (run.py has the usual `if __name__ == "__main__":`
#   guard). A driver script that calls run_pysim must likewise put
#   the call under `if __name__ == "__main__":`: with spawn, each
#   worker re-imports the calling script, and a top-level
#   run_pysim call would re-run in the worker instead of the
#   guarded main process.
# - Shared memory needs Python 3.8+ (3.9 on Eli's machine: fine).
#   The block is unlinked in a finally block after the pool joins,
#   so a worker crash cannot leak it past the run; if the main
#   process itself is killed hard (not a normal exception) the
#   block can be left behind until it is removed by hand
#   (Linux: /dev/shm, names start with "psm_"; Windows: the
#   resource tracker removes it when the last handle closes).
# - The pysim package must be importable in the workers, which it
#   is whenever `from pysim import run_pysim` works in the caller:
#   spawn children inherit sys.path from the parent.
import atexit
import os
from multiprocessing import get_context
from multiprocessing import shared_memory

import numpy as np

from .cpp import f32
from .sp import find_best_path

# Automatic mode (workers=None) uses the pool only when the key
# count clears this floor: pool creation plus one worker init per
# process costs a few seconds, which only pays off once there are
# enough distinct keys. An explicit workers >= 2 always tries the
# pool (and still falls back to serial on any failure).
_MIN_KEYS_AUTO = 64
# Automatic mode caps the pool here; explicit workers=N is used
# as given. Iteration-0 keys are CPU-bound and independent, so
# more workers help until the machine runs out of cores.
_MAX_WORKERS_AUTO = 16


class _WorkerNode:
    __slots__ = ("zone_id", "outgoing")

    def __init__(self, zone_id, outgoing):
        self.zone_id = zone_id
        self.outgoing = outgoing


class _WorkerLink:
    __slots__ = ("from_id", "to_id", "type_code")

    def __init__(self, from_id, to_id, type_code):
        self.from_id = from_id
        self.to_id = to_id
        # find_best_path only ever tests `"c" in link.type_code`,
        # so the stored string is "c" for a connector link and ""
        # otherwise; the test behaves exactly as on the real link.
        self.type_code = type_code


class _WorkerWorld:
    __slots__ = ("nodes", "links", "start", "end", "horizon",
                 "num_periods", "assign_index")

    def interval_index(self, time_in_min):
        # Verbatim copy of World.interval_index (network.py):
        # g_FindAssignmentIntervalIndexFromTime. The worker shim
        # must compute the same interval for the same label time,
        # or the TD lookup -- and the path -- could differ.
        if time_in_min < self.start:
            return 0
        if time_in_min < self.horizon:
            slot = max(0, int(f32(time_in_min) + 0.1) // 15)
            if slot < len(self.assign_index):
                value = self.assign_index[slot]
            else:
                value = max(0, self.num_periods - 1)
        else:
            value = max(0, self.num_periods - 1)
        return min(value, self.num_periods - 1)


class _WorkerNet:
    __slots__ = ("td_time",)


# Worker-process globals, set once by _sp_worker_init.
_WORLD = None
_NET = None
_SHM = []


def _close_worker_shm():
    # Workers close their handle only; the main process owns the
    # unlink (see resolve_keys_parallel). Closing here keeps the
    # handle from outliving the worker.
    for shm in _SHM:
        try:
            shm.close()
        except Exception:
            pass
    del _SHM[:]


def _sp_worker_init(shm_name, td_shape, single_col, topo, scalars):
    global _WORLD, _NET
    shm = shared_memory.SharedMemory(name=shm_name)
    _SHM.append(shm)
    atexit.register(_close_worker_shm)
    if single_col:
        col = np.ndarray((td_shape[0],), dtype=np.float32,
                         buffer=shm.buf)
        # Read-only broadcast view: find_best_path only reads
        # td_time[link_no, interval], and every column is the
        # shared column by the uniformity check in the main
        # process, so this indexes exactly like the full table.
        td = np.broadcast_to(col[:, None], td_shape)
    else:
        td = np.ndarray(td_shape, dtype=np.float32, buffer=shm.buf)
    node_zone, link_from, link_to, link_is_c, out_ptr, out_idx = topo
    nodes = []
    for i in range(len(node_zone)):
        nodes.append(_WorkerNode(
            int(node_zone[i]),
            [int(x) for x in out_idx[out_ptr[i]:out_ptr[i + 1]]]))
    links = []
    for j in range(len(link_from)):
        links.append(_WorkerLink(int(link_from[j]), int(link_to[j]),
                                 "c" if link_is_c[j] else ""))
    world = _WorkerWorld()
    world.nodes = nodes
    world.links = links
    (world.start, world.end, world.horizon, world.num_periods,
     world.assign_index) = scalars
    net = _WorkerNet()
    net.td_time = td
    _WORLD = world
    _NET = net


def _sp_worker(task):
    # task: (origin_zone, origin, departure, dest_zone, dest,
    # demand_type, vot). Returns the key plus the same
    # (node_size, path_tuple) the serial _sp_path would return;
    # total_cost is dropped, as the serial caller drops it too.
    oz, o, dep, dz, d, dt, vot = task
    node_size, path_links, _ = find_best_path(
        _WORLD, _NET, oz, o, dep, dz, d, dt, vot)
    return (oz, o, dz, d, node_size, tuple(path_links))


def _td_is_uniform(td):
    # True when every column of the TD table equals column 0, so
    # sharing one column is exact. Checked over the whole table
    # in row blocks (a full-table boolean temporary would be as
    # big as the table itself); exits early on the first mismatch.
    if td.shape[1] <= 1:
        return True
    # NOTE: np.array_equal does not broadcast, so a (rows, cols)
    # block against a (rows, 1) block would report False even when
    # equal; compare with == (which broadcasts) and .all() instead.
    col0 = td[:, :1]
    step = 4096
    for lo in range(0, td.shape[0], step):
        if not bool((td[lo:lo + step] ==
                     col0[lo:lo + step]).all()):
            return False
    return True


def _topology_arrays(world):
    # Compact picklable topology for the worker initializer: node
    # zones, link endpoints, the connector-link flag, and the
    # outgoing lists as one CSR pair (every link appears in
    # exactly one node's outgoing list, so the index is one entry
    # per link). The TD table itself is not copied here; it
    # travels in shared memory.
    nodes = world.nodes
    links = world.links
    node_zone = np.array([nd.zone_id for nd in nodes],
                         dtype=np.int32)
    link_from = np.array([lk.from_id for lk in links],
                         dtype=np.int32)
    link_to = np.array([lk.to_id for lk in links], dtype=np.int32)
    link_is_c = np.array(["c" in lk.type_code for lk in links],
                         dtype=np.bool_)
    out_ptr = np.zeros(len(nodes) + 1, dtype=np.intp)
    out_idx = np.empty(len(links), dtype=np.intp)
    pos = 0
    for i, nd in enumerate(nodes):
        out_ptr[i] = pos
        for link_no in nd.outgoing:
            out_idx[pos] = link_no
            pos += 1
    out_ptr[len(nodes)] = pos
    return (node_zone, link_from, link_to, link_is_c, out_ptr, out_idx)


def resolve_workers(workers, n_keys):
    # How many pool workers to use, or 0 for the serial path.
    # workers=None means automatic: min(cpu_count, 16) when there
    # are enough distinct keys to pay for pool setup, else serial.
    # workers <= 1 (or 0) is serial. An explicit workers >= 2 is
    # used as given. The PYSIM_SP_WORKERS environment variable
    # overrides the automatic choice when workers is None (an
    # escape hatch: set it to 1 to force serial without touching
    # code); an explicit workers argument always wins over it.
    if workers is None:
        env = os.environ.get("PYSIM_SP_WORKERS")
        if env is not None:
            try:
                workers = int(env)
            except ValueError:
                workers = None
    if workers is None:
        ncpu = os.cpu_count() or 1
        if ncpu < 2 or n_keys < _MIN_KEYS_AUTO:
            return 0
        return min(ncpu, _MAX_WORKERS_AUTO)
    if workers <= 1:
        return 0
    return int(workers)


def resolve_keys_parallel(world, net, tasks, workers, on_key=None):
    # Resolve distinct iteration-0 SP keys in a spawn Pool.
    # tasks is [(key, departure, demand_type, vot), ...] in
    # first-seen order, key = (origin_zone, origin, dest_zone,
    # dest). Returns {key: (node_size, path_tuple)}, or None when
    # the pool should not or could not be used: the caller then
    # resolves the same tasks serially, so a pool failure never
    # fails a run that would have worked serially. on_key, when
    # given, is called once per result received, in the main
    # process's imap_unordered loop (a counter increment only;
    # it cannot change pool behaviour, ordering or results, and
    # an exception in it is swallowed here so it cannot kill
    # the pool).
    if not tasks:
        return {}
    n_workers = resolve_workers(workers, len(tasks))
    if n_workers < 2:
        return None
    td = net.td_time
    if td is None:
        return None
    single_col = _td_is_uniform(td)
    if single_col:
        payload = np.ascontiguousarray(td[:, 0])
    else:
        payload = np.ascontiguousarray(td)
    topo = _topology_arrays(world)
    scalars = (world.start, world.end, world.horizon,
               world.num_periods, list(world.assign_index))
    shm = None
    pool = None
    results = {}
    ok = False
    try:
        shm = shared_memory.SharedMemory(create=True,
                                         size=payload.nbytes)
        arr = np.ndarray(payload.shape, dtype=np.float32,
                         buffer=shm.buf)
        arr[:] = payload
        ctx = get_context("spawn")
        pool = ctx.Pool(n_workers, initializer=_sp_worker_init,
                        initargs=(shm.name, tuple(td.shape),
                                  single_col, topo, scalars))
        jobs = [(key[0], key[1], dep, key[2], key[3], dt, vot)
                for key, dep, dt, vot in tasks]
        for oz, o, dz, d, node_size, path in pool.imap_unordered(
                _sp_worker, jobs, chunksize=1):
            results[(oz, o, dz, d)] = (node_size, path)
            if on_key is not None:
                try:
                    on_key()
                except Exception:
                    pass
        ok = True
    except Exception:
        ok = False
    finally:
        if pool is not None:
            if ok:
                pool.close()
            else:
                pool.terminate()
            pool.join()
        if shm is not None:
            try:
                shm.close()
            except Exception:
                pass
            try:
                shm.unlink()
            except Exception:
                pass
    if not ok or len(results) != len(tasks):
        return None
    return results
