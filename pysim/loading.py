# The network loading: g_NetworkLoading + g_VehicularSimulation
# (simulation.cpp) and the two real-time path adjustments
# (RealTimeRouting.cpp). One step = 0.1 min (6 s), five stages per step:
# generation, loading buffer, entrance->exit transfer, capacities with
# merge distribution, node transfer; then per-step statistics.
import math

import numpy as np

from .charging import KM_PER_MILE, _format_decision_row
from .cpp import ProgramStop, f32, floating_less, less_eq
from .network import VehicleLink
from .outputs import _format_agent_leg_row, _format_agent_vehicle_row
from .sp import SPNetwork, find_best_path
from .sp_mp import resolve_keys_parallel

SIM_INTERVAL = 0.1  # g_DTASimulationInterval (double)


class Simulator:
    def __init__(self, world, sink=None, progress=None, bar=None):
        self.world = world
        # Incremental-output sink (v8): an IncrementalWriter, or
        # None for exact v7 behaviour. progress is a RunProgress
        # (or None); it carries the assignment key counts and the
        # progress file. bar is a ProgressBar (or None); it is
        # display only, ticked from the loading loop below and
        # from the assignment key callback.
        self.sink = sink
        self.progress = progress
        self.bar = bar
        self.generated = 0
        self.completed = 0
        # Time of the most recent step, for the progress file
        # (and team 2's bar); None before loading starts.
        self.current_time = None
        # g_PrevailingTimeNetwork_MP[0], built in g_ReadInputFiles with
        # current (prevailing) information at the loading start time
        self.prevailing_net = SPNetwork(world, use_prevailing=True,
                                        current_time=world.start)
        # Free-flow network (iteration 0): fftt never changes during
        # loading, so the charging engine reuses it for en-route
        # charger searches instead of rebuilding it per leg.
        self.ff_net = None
        self.td_map = {}
        # Freeway-only links for stage 5, built in network_loading:
        # ring buffers are read only in the stage-3 freeway branch.
        self.freeway_links = None
        # Agent ids parked at en-route chargers (slice 3a), in park
        # order. step() resumes them before stage 0; _discharge_link
        # appends when a vehicle parks.
        self.parked_ids = []

    # ----- iteration 0 of the agent-based driver -----

    def _sp_path(self, world, net, cache, use_cache,
                 origin_zone, origin, departure, dest_zone, dest,
                 demand_type, vot):
        # Iteration-0 shortest path with memoization. The path
        # depends only on (origin zone/node, destination zone/node),
        # not on the departure time: the iteration-0 network is built
        # from free-flow times, so every assignment interval holds the
        # same link times (see network.travel_time_by_min), and
        # find_best_path reads the interval only to index that table.
        # Demand type and value of time are not in the key either:
        # find_best_path uses vot only for the vot <= 0.01 -> 1 clamp
        # and demand_type only for the 0 -> 1 clamp (the generalized
        # cost flag is off, so neither affects the returned path).
        # The caller guards all of this with the uniformity tripwire
        # in assign_paths_iteration_zero; if the table ever becomes
        # time-varying, use_cache is False and every leg gets its own
        # search, exactly as before. The cached value is the raw
        # search result; the caller still owns setting node_size /
        # link_ary / distance, and the RNG draw stays in the caller's
        # loop, so the random stream and every leg field are identical
        # with or without a cache hit.
        key = (origin_zone, origin, dest_zone, dest)
        if use_cache:
            hit = cache.get(key)
            if hit is not None:
                self.sp_cache_hits += 1
                return hit
        node_size, path_links, _ = find_best_path(
            world, net, origin_zone, origin, departure,
            dest_zone, dest, demand_type, vot)
        result = (node_size, tuple(path_links))
        if use_cache:
            cache[key] = result
            self.sp_cache_misses += 1
        return result

    def _report_sp_cache(self):
        total = self.sp_cache_hits + self.sp_cache_misses
        self.world.sp_cache_hits = self.sp_cache_hits
        self.world.sp_cache_misses = self.sp_cache_misses
        # The cache stats print after assignment, when the bar may
        # already have drawn on a TTY: end its line first so the
        # stats start on a fresh line (a no-op off a TTY, or when
        # the bar never drew).
        if self.bar is not None:
            self.bar.ensure_newline()
        if total > 0:
            print("iteration-0 shortest paths: %d computed, %d reused "
                  "from cache (%.0f%% hit rate)"
                  % (self.sp_cache_misses, self.sp_cache_hits,
                     100.0 * self.sp_cache_hits / total))

    def _note_key_resolved(self):
        # One assignment key resolved (pool, serial fallback, or
        # tripwire path): count it for the progress file and tick
        # the console bar, which throttles its own display. The
        # counts live on RunProgress; the bar only reads them.
        if self.progress is not None:
            self.progress.note_key_resolved()
        if self.bar is not None and self.progress is not None:
            self.bar.tick_assigning(
                self.progress.assign_keys_resolved,
                self.progress.assign_keys_total,
                self.progress.started_mono)

    def assign_paths_iteration_zero(self, workers=None):
        # g_AgentBasedDynamicTrafficAssignmentSimulation, iteration 0:
        # pathless agents (by zone bucket, then read order) get a free-flow
        # shortest path on the physical network; preassigned paths are kept.
        # workers controls the Wave-2 multiprocessing of the distinct
        # SP keys (see sp_mp): None means automatic (a spawn Pool when
        # there are enough distinct keys and more than one CPU, else
        # serial); 1 (or 0) is serial; N >= 2 uses a Pool of N workers.
        # The pool only ever resolves keys; the bucket walk, the RNG
        # draws and the assignment all stay in the main process, so a
        # pool run and a serial run produce identical outputs. Any
        # pool failure falls back to resolving the same keys serially.
        world = self.world
        net = SPNetwork(world, use_prevailing=False)
        self.ff_net = net
        engine = getattr(world, "charging_engine", None)
        if engine is not None:
            engine.ff_net = net
        # Memoization tripwire: the cache in _sp_path is valid only
        # while the SP table is the same in every interval. The
        # rebuild walk visits every interval with the same free-flow
        # value, so compare the first, middle and last columns for
        # every link -- O(links), and any time-varying table trips
        # it. A tripped wire disables the cache, not the run.
        self.sp_cache_hits = 0
        self.sp_cache_misses = 0
        cache = {}
        use_cache = True
        if net.td_time is not None and world.num_periods > 1:
            cols = (0, world.num_periods // 2,
                    world.num_periods - 1)
            for row in net.td_time:
                if row[cols[0]] != row[cols[1]] or \
                        row[cols[0]] != row[cols[2]]:
                    use_cache = False
                    break
        # Phase 1 -- the sequential bucket walk, exactly as before:
        # guards, gen_rng.ratio() draws in bucket order, key
        # extraction. No path is computed here, so the random stream
        # and the set of legs needing a path are identical whether
        # the keys are resolved serially or in a pool. Each call is
        # (key, departure, demand_type, vot, target) with target
        # (veh, leg) in tour mode or (veh, None) in legacy mode.
        # A leg (or vehicle) appears at most once in the walk, so
        # deferring the assignment to phase 3 changes nothing: no
        # guard in the walk reads a node_size set by an earlier
        # assignment.
        calls = []
        if world.tour_mode:
            # Per leg: buckets keyed by (leg origin zone, interval of
            # the leg's scheduled departure). Paths land on the leg,
            # never on the vehicle's working fields directly.
            buckets = {}
            for agent_id in world.vehicle_order:
                veh = world.vehicles[agent_id]
                for leg in veh.legs:
                    zone_seq = world.zones[leg.origin_zone].seq_no
                    interval = world.interval_index(leg.scheduled_departure)
                    buckets.setdefault((zone_seq, interval), []).append(
                        (veh, leg))
            for key in sorted(buckets):
                for veh, leg in buckets[key]:
                    if veh.info_type == 0 and leg.node_size >= 2:
                        continue
                    if leg.origin_node == leg.dest_node:
                        continue  # intrazonal: same node, instant leg
                    world.gen_rng.ratio()  # pVeh->GetRandomRatio() draw
                    if leg.node_size != 0:
                        continue
                    calls.append(((leg.origin_zone, leg.origin_node,
                                   leg.dest_zone, leg.dest_node),
                                  leg.scheduled_departure, veh.demand_type,
                                  veh.vot, (veh, leg)))
        else:
            buckets = {}
            for agent_id in world.vehicle_order:
                veh = world.vehicles[agent_id]
                zone_seq = world.zones[veh.origin_zone].seq_no
                interval = world.interval_index(veh.departure)
                buckets.setdefault((zone_seq, interval), []).append(veh)
            for key in sorted(buckets):
                for veh in buckets[key]:
                    if veh.info_type == 0 and veh.node_size >= 2:
                        continue
                    if veh.origin_zone == veh.dest_zone:
                        continue  # intra-zone traffic is not simulated
                    world.gen_rng.ratio()  # pVeh->GetRandomRatio() draw, unused
                    if veh.node_size != 0:
                        continue
                    calls.append(((veh.origin_zone, veh.origin_node,
                                   veh.dest_zone, veh.dest_node),
                                  veh.departure, veh.demand_type,
                                  veh.vot, (veh, None)))
        # Phase 2 -- resolve the calls.
        per_call = []
        order = []
        if use_cache:
            # Distinct keys in first-seen order; a key's path is
            # computed once, at its first leg's departure, exactly
            # as the old cache did (the cached value never depended
            # on departure, demand_type or vot beyond the clamps in
            # find_best_path, and the first-seen values are passed
            # through unchanged).
            first_seen = {}
            for key, dep, dt, vot, _t in calls:
                if key not in first_seen:
                    first_seen[key] = (dep, dt, vot)
                    order.append(key)
            self.sp_cache_misses = len(order)
            self.sp_cache_hits = len(calls) - len(order)
            tasks = [(key, first_seen[key][0], first_seen[key][1],
                      first_seen[key][2]) for key in order]
            if self.progress is not None:
                self.progress.assign_keys_total = len(tasks)
            on_key = self._note_key_resolved \
                if self.progress is not None or self.bar is not None \
                else None
            resolved = resolve_keys_parallel(world, net, tasks,
                                             workers, on_key=on_key)
            if resolved is None:
                # Serial fallback (pool not wanted, or any pool
                # failure): the same keys at the same departures.
                # Reset the resolved count first: a pool that
                # failed mid-stream may already have reported
                # some keys through on_key, and the fallback
                # re-resolves every key.
                if self.progress is not None:
                    self.progress.assign_keys_resolved = 0
                resolved = {}
                for key, dep, dt, vot in tasks:
                    node_size, path_links, _ = find_best_path(
                        world, net, key[0], key[1], dep,
                        key[2], key[3], dt, vot)
                    resolved[key] = (node_size, tuple(path_links))
                    if on_key is not None:
                        on_key()
            per_call = [resolved[key]
                        for key, _d, _t, _v, _tg in calls]
        else:
            # Tripwire tripped: the table is time-varying, so a
            # key's path depends on its own departure -- no dedup,
            # no pool, one search per call in walk order, exactly
            # as before. (No cache report: hits and misses stay 0,
            # as before.)
            if self.progress is not None:
                self.progress.assign_keys_total = len(calls)
            for key, dep, dt, vot, _t in calls:
                per_call.append(self._sp_path(
                    world, net, cache, use_cache,
                    key[0], key[1], dep, key[2], key[3], dt, vot))
                self._note_key_resolved()
        # Phase 3 -- assign the resolved paths in the main process,
        # in first-seen key order; distance is accumulated in path
        # order exactly as before.
        if use_cache:
            by_key = {}
            for call, res in zip(calls, per_call):
                by_key.setdefault(call[0], []).append((call, res))
            ordered = []
            for key in order:
                ordered.extend(by_key[key])
        else:
            ordered = list(zip(calls, per_call))
        for (_key, _dep, _dt, _vot, target), (node_size, path_links) \
                in ordered:
            veh, leg = target
            obj = leg if leg is not None else veh
            if node_size >= 2:
                obj.node_size = node_size
                obj.link_ary = [VehicleLink(link_no)
                                for link_no in path_links]
                obj.link_ary.append(VehicleLink(65530))
                distance = f32(0)
                for link_no in path_links:
                    distance = f32(distance +
                                   world.links[link_no].length)
                obj.distance = distance
        # Phase 4 -- tour-mode post checks, as before: a pathless
        # leg between distinct nodes must fail loud, and leg 0's
        # path is re-mirrored onto the vehicle's working fields.
        if world.tour_mode:
            for agent_id in world.vehicle_order:
                veh = world.vehicles[agent_id]
                for leg in veh.legs:
                    if leg.origin_node != leg.dest_node and \
                            leg.node_size < 2:
                        raise ProgramStop(
                            "agent %d leg %d has no path from node %d "
                            "to node %d"
                            % (agent_id, leg.leg_index,
                               world.nodes[leg.origin_node].number,
                               world.nodes[leg.dest_node].number))
            for agent_id in world.vehicle_order:
                veh = world.vehicles[agent_id]
                if veh.legs:
                    veh.begin_leg(veh.legs[0])
        self._report_sp_cache()

    # ----- real-time path adjustments -----

    def _has_pending_charger_stop(self, veh):
        # True while the current leg's en-route stop has not been
        # reached (or cancelled): the via path is a committed
        # charging plan, so the built-in real-time adjustments
        # must not re-plan it away. Once the stop is done the leg
        # is an ordinary trip again.
        if not veh.legs:
            return False
        stop = veh.legs[veh.current_leg].charger_stop
        return stop is not None and not stop["done"]

    def _build_via_path(self, veh, leg, site, links_to, links_from):
        # Splice the en-route via path at generation (slice 3a):
        # the free-flow links origin -> charger plus charger ->
        # destination, replacing the direct path. The direct path
        # is stored as the original path first, so output keeps
        # both sequences. Distance and free-flow time are the via
        # sums (the writer computes scheduled arrival from
        # trip_fftt, so this keeps it correct); the planning
        # energy on the leg stays the direct-path estimate. The
        # decision already verified the site is viable, so a via
        # path that fails validation here is an internal
        # inconsistency and raises instead of falling back.
        world = self.world
        links = world.links
        via = list(links_to) + list(links_from)
        if not links_to or not links_from or \
                links[via[0]].from_id != leg.origin_node or \
                links[via[-1]].to_id != leg.dest_node:
            raise ProgramStop(
                "en-route via path for agent %d leg %d via charger "
                "%s does not run from its origin to its destination"
                % (veh.agent_id, leg.leg_index, site.charger_id))
        for a, b in zip(via, via[1:]):
            if links[a].to_id != links[b].from_id:
                raise ProgramStop(
                    "en-route via path for agent %d leg %d via "
                    "charger %s is not a connected chain"
                    % (veh.agent_id, leg.leg_index, site.charger_id))
        if links[via[len(links_to) - 1]].to_id != site.node_id or \
                links[via[len(links_to)]].from_id != site.node_id:
            raise ProgramStop(
                "en-route via path for agent %d leg %d does not "
                "pass through charger %s"
                % (veh.agent_id, leg.leg_index, site.charger_id))
        self._store_original_path(veh)
        veh.link_ary = [VehicleLink(link_no) for link_no in via]
        veh.link_ary.append(VehicleLink(65530))
        leg.link_ary = veh.link_ary
        veh.node_size = len(via) + 1
        leg.node_size = veh.node_size
        distance = f32(0)
        trip_fftt = f32(0)
        for link_no in via:
            distance = f32(distance + links[link_no].length)
            trip_fftt = f32(trip_fftt + links[link_no].fftt)
        veh.distance = distance
        leg.distance = distance
        veh.trip_fftt = trip_fftt
        leg.trip_fftt = trip_fftt
        leg.charger_stop = {"node": site.node_id, "site": site,
                            "done": False,
                            "target_soc": leg.decision_target_soc}

    def _splice_mid_leg_via(self, veh, leg, site, links_to,
                            links_from, via_time_min):
        # Liquid-fuel mid-leg splice (v5): replace the rest of the
        # leg's path from the current node with the via path
        # current node -> charger -> destination, keeping the
        # links already travelled (up to and including the
        # just-completed link, with their arrival stamps). Unlike
        # _splice_reroute this also sets the en-route stop, the
        # trip free-flow time, and the leg fields: the leg fields
        # are read mid-leg by the energy cache and the stop check,
        # so they must stay in sync now, not only at finish_leg.
        # sim_link_seq still points at the just-completed link, so
        # the next link (index sim_link_seq + 1) is the first via
        # link; soc_deducted_seq, generated, the decision fields,
        # soc_start_kwh, and veh.soc_kwh are all left alone (no
        # charge happens at splice time; charge_enroute charges
        # at the stop).
        world = self.world
        links = world.links
        current_node = links[veh.link_ary[veh.sim_link_seq].link_no] \
            .to_id
        if not links_to or not links_from or \
                links[links_to[0]].from_id != current_node or \
                links[links_to[-1]].to_id != site.node_id or \
                links[links_from[0]].from_id != site.node_id or \
                links[links_from[-1]].to_id != veh.dest_node:
            raise ProgramStop(
                "liquid-fuel via path for agent %d leg %d via "
                "charger %s does not run from its current node to "
                "its destination" % (veh.agent_id, leg.leg_index,
                                     site.charger_id))
        combined = list(links_to) + list(links_from)
        for a, b in zip(combined, combined[1:]):
            if links[a].to_id != links[b].from_id:
                raise ProgramStop(
                    "liquid-fuel via path for agent %d leg %d via "
                    "charger %s is not a connected chain"
                    % (veh.agent_id, leg.leg_index,
                       site.charger_id))
        self._store_original_path(veh)
        kept = [entry.link_no
                for entry in veh.link_ary[:veh.sim_link_seq + 1]]
        stamps = [entry.abs_arrival
                  for entry in veh.link_ary[:veh.sim_link_seq + 1]]
        full = kept + combined
        veh.node_size = len(full) + 1
        veh.link_ary = []
        distance = f32(0)
        trip_fftt = f32(0)
        for i, link_no in enumerate(full):
            entry = VehicleLink(link_no)
            entry.abs_arrival = stamps[i] if i < len(stamps) \
                else f32(0)
            veh.link_ary.append(entry)
            distance = f32(distance + links[link_no].length)
            trip_fftt = f32(trip_fftt + links[link_no].fftt)
        veh.link_ary.append(VehicleLink(65530))
        veh.distance = distance
        veh.trip_fftt = trip_fftt
        leg.link_ary = veh.link_ary
        leg.node_size = veh.node_size
        leg.distance = veh.distance
        leg.trip_fftt = veh.trip_fftt
        leg.charger_stop = {"node": site.node_id, "site": site,
                            "done": False,
                            "target_soc": leg.enroute_target_soc}
        leg.enroute_via = (links_to, links_from, via_time_min)
        leg.trigger = "liquid"
        leg.trigger_charger_id = site.charger_id
        leg.liquid_fired = True

    def _store_original_path(self, veh):
        # Keep the FIRST original: a later adjustment (e.g. an
        # info-3 enroute_adjustment after a tow resume) must not
        # overwrite the pre-tow original with the spliced path.
        if veh.original_link_ary is not None:
            return
        veh.original_link_ary = [
            VehicleLink(entry.link_no) for entry in veh.link_ary]
        for new, old in zip(veh.original_link_ary, veh.link_ary):
            new.abs_arrival = old.abs_arrival
        veh.original_node_size = veh.node_size

    def generation_adjustment(self, veh, current_time):
        # g_AgentBasedPathAdjustmentWithRealTimeInfo (RealTimeRouting.cpp:399)
        # Skipped while an en-route charger stop is pending: the
        # via path is a committed plan, and re-planning from the
        # first link's to-node would silently drop the detour.
        if self._has_pending_charger_stop(veh):
            return
        world = self.world
        if veh.node_size >= 1 and veh.link_ary:
            current_link = veh.link_ary[veh.sim_link_seq].link_no
            starting_node = world.links[current_link].to_id
            if starting_node == veh.dest_node:
                return
            current_path = [entry.link_no for entry in veh.link_ary[:-1]]
            stamps = [entry.abs_arrival for entry in veh.link_ary[:-1]]
            link_count = veh.sim_link_seq
            updated = list(current_path[:veh.sim_link_seq + 1])
        else:
            starting_node = veh.origin_node
            current_path = []
            stamps = []
            link_count = 0
            updated = []
        node_size, path_links, _ = find_best_path(
            world, self.prevailing_net, veh.origin_zone, starting_node,
            veh.departure, veh.dest_zone, veh.dest_node, veh.demand_type,
            veh.vot)
        switch = 0
        for i in range(node_size - 1):
            link_count += 1
            old = current_path[link_count] if link_count < len(current_path) \
                else -1
            if old != path_links[i]:
                switch = 1
            updated.append(path_links[i])
        if node_size <= 1 or switch == 0:
            return
        if veh.node_size >= 2:
            self._store_original_path(veh)
            veh.node_size = len(updated) + 1
            veh.link_ary = []
            distance = f32(0)
            for i, link_no in enumerate(updated):
                entry = VehicleLink(link_no)
                entry.abs_arrival = stamps[i] if i < len(stamps) else f32(0)
                veh.link_ary.append(entry)
                distance = f32(distance + world.links[link_no].length)
            veh.link_ary.append(VehicleLink(65530))
            veh.distance = distance

    def enroute_adjustment(self, veh, current_time):
        # g_OpenMPAgentBasedPathAdjustmentWithRealTimeInfo
        # (RealTimeRouting.cpp:649), VMS detour branch inert (no signs)
        # Skipped while an en-route charger stop is pending, for the
        # same reason as generation_adjustment: re-planning to the
        # destination from here would drop the charger detour.
        if self._has_pending_charger_stop(veh):
            return
        world = self.world
        current_link = veh.link_ary[veh.sim_link_seq].link_no
        starting_node = world.links[current_link].to_id
        current_path = [entry.link_no for entry in veh.link_ary[:-1]]
        stamps = [entry.abs_arrival for entry in veh.link_ary[:-1]]
        node_size, sub_links, _ = find_best_path(
            world, self.prevailing_net, veh.origin_zone, starting_node,
            veh.departure, veh.dest_zone, veh.dest_node, veh.demand_type,
            veh.vot)
        if veh.sim_link_seq == veh.node_size - 2:
            return
        now_count = veh.sim_link_seq + 1
        first_diff_old = -1
        first_diff_new = -1
        switch = 0
        for i in range(node_size - 1):
            old = current_path[now_count] if now_count < len(current_path) \
                else -1
            if old != sub_links[i]:
                first_diff_old = now_count
                first_diff_new = i
                switch = 1
                break
            now_count += 1
        if node_size - 1 <= 0:
            return
        diff_size = (node_size - 1) - first_diff_new
        new_node_size = first_diff_old + diff_size + 1
        if new_node_size >= 3000:
            return
        if switch == 0:
            return
        self._store_original_path(veh)
        combined = current_path[:first_diff_old] + \
            sub_links[first_diff_new:first_diff_new + diff_size]
        veh.node_size = new_node_size
        veh.link_ary = []
        distance = f32(0)
        for i, link_no in enumerate(combined):
            entry = VehicleLink(link_no)
            stamp = stamps[i] if i < len(stamps) else f32(0)
            if stamp == f32(0):
                stamp = f32(99999.0)
            entry.abs_arrival = stamp
            veh.link_ary.append(entry)
            distance = f32(distance + world.links[link_no].length)
        veh.link_ary.append(VehicleLink(65530))
        veh.distance = distance

    def _splice_reroute(self, veh, next_node, new_links):
        # Replace the rest of a vehicle's path with a caller-supplied
        # link list, keeping everything up to and including the link
        # the vehicle just completed (sim_link_seq still points at
        # it: the splice runs before the move to the next link).
        # Contract for new_links: a connected chain of link numbers
        # whose first link starts at next_node (the to-node of the
        # just-completed link) and whose last link ends at the
        # vehicle's destination node.
        # Anything else raises ProgramStop rather than silently
        # corrupting the path. The pre-splice path is stored as the
        # original path, exactly as the built-in adjustments do, so
        # output_agent.csv keeps both the travelled and original
        # node sequences.
        world = self.world
        links = world.links
        if not new_links:
            return
        if links[new_links[0]].from_id != next_node:
            raise ProgramStop(
                "reroute path for agent %d does not start at its "
                "current node" % veh.agent_id)
        if links[new_links[-1]].to_id != veh.dest_node:
            raise ProgramStop(
                "reroute path for agent %d does not end at its "
                "destination" % veh.agent_id)
        for a, b in zip(new_links, new_links[1:]):
            if links[a].to_id != links[b].from_id:
                raise ProgramStop(
                    "reroute path for agent %d is not a connected "
                    "chain" % veh.agent_id)
        self._store_original_path(veh)
        kept = [entry.link_no for entry in veh.link_ary[:veh.sim_link_seq + 1]]
        stamps = [entry.abs_arrival
                  for entry in veh.link_ary[:veh.sim_link_seq + 1]]
        combined = kept + list(new_links)
        veh.node_size = len(combined) + 1
        veh.link_ary = []
        distance = f32(0)
        for i, link_no in enumerate(combined):
            entry = VehicleLink(link_no)
            entry.abs_arrival = stamps[i] if i < len(stamps) else f32(0)
            veh.link_ary.append(entry)
            distance = f32(distance + world.links[link_no].length)
        veh.link_ary.append(VehicleLink(65530))
        veh.distance = distance

    # ----- network loading -----

    def network_loading(self):
        world = self.world
        for link in world.links:
            link.setup_loading(world.horizon, world.min_inflow_ratio)
        self.freeway_links = [link for link in world.links
                              if "f" in link.type_code]
        world.info_updating_flag = False
        for agent_id in world.vehicle_order:
            veh = world.vehicles[agent_id]
            if veh.demand_type != 4:
                veh.arrival = f32(0)
                veh.trip_time = f32(0)
                veh.trip_fftt = f32(0)
            veh.loaded = False
            veh.complete = False
            veh.delay = f32(0)
            if veh.info_type >= 2:
                # Fail loud: info-updating agents would switch on the
                # info-updating block in step() (per-link exit-queue
                # walk plus update_current_travel_time every minute),
                # a catastrophic silent slowdown on a full-week run.
                # The converter writes info_type 1 for every agent,
                # so this never fires on a real run; anything else is
                # a bad input, not a case to simulate slowly.
                raise ProgramStop(
                    "agent %d has info_type %d: info-updating "
                    "agents (info_type >= 2) are not supported in "
                    "this build" % (agent_id, veh.info_type))
        self.generated = 0
        self.completed = 0
        # rebuild the time-dependent departure lists from the original map,
        # walking the same floating-point time loop as the C++ code
        self.td_map = {}
        step = 0
        time = float(world.start)
        limit = min(world.horizon, world.horizon)
        while time < limit:
            time = world.start + step * SIM_INTERVAL
            time_int = int(time * 10)
            if time_int in world.original_td_map:
                self.td_map[time_int] = list(world.original_td_map[time_int])
            step += 1
        # Early-exit bookkeeping. n_generatable counts, once, every
        # leg (tour mode) or vehicle (legacy) that can ever be
        # generated and completed: stage 0 loads only node_size >= 2
        # legs / vehicles, and node_size is final at this point
        # (iteration-0 assignment already ran; the real-time
        # adjustments rewrite only loaded or in-transit vehicles and
        # keep node_size >= 2, so nothing can join the generatable
        # set later). In tour mode every leg counts, including
        # intrazonal instant legs: an instant leg starts and finishes
        # atomically in stage 0, so it increments both generated and
        # completed when it fires. Real legs increment generated at
        # generation and completed at completion, as before. The
        # count is therefore exact for the generated set and a
        # superset of the completed set: a counted leg that never
        # completes (permanent gridlock) simply keeps the break from
        # firing, and generated == completed == n_generatable implies
        # every leg started and finished.
        self.n_generatable = 0
        for agent_id in world.vehicle_order:
            veh = world.vehicles[agent_id]
            if veh.legs:
                self.n_generatable += len(veh.legs)
            elif veh.node_size >= 2:
                self.n_generatable += 1
        self._early_exit_vetoed = False
        if self.bar is not None:
            # The loading phase change is announced here, not in
            # run.py: the leg total is known only now, so the
            # first bar line can already show legs 0/total.
            self.bar.set_phase("loading", total=self.n_generatable)
        # Main loop: stops early once the network is drained (see the
        # break below). horizon = loading end + 300.
        step = 0
        time = float(world.start)
        while time < limit:
            time = world.start + step * SIM_INTERVAL
            self.step(time, step)
            step_no = step
            step += 1
            if self.bar is not None:
                # Per-step loading tick (design section 9.4): the
                # bar throttles itself, so this is one cheap check
                # per step and an emission at most twice a minute.
                self.bar.tick_loading(
                    time, self.completed, self.n_generatable,
                    self.progress.started_mono
                    if self.progress is not None else None)
            # Early exit. Every clause is an O(1) integer/float compare.
            # time > world.end: every registered departure lies in
            # [start, end] (read time rejects anything outside), and each
            # departure bucket is visited exactly once as the clock
            # advances, so past end no vehicle can still be generated.
            # step_no % 10 == 0: the break step is a minute boundary, so
            # stage 5 has just written this minute's MOE from final flows
            # and the post-pass propagates those finals over the tail.
            # generated == completed == n_generatable: every generatable
            # leg / vehicle entered the network and reached its destination.
            if not self._early_exit_vetoed and time > world.end and \
                    step_no % 10 == 0 and \
                    self.generated == self.n_generatable and \
                    self.completed == self.n_generatable:
                # Belt-and-braces veto scan: ground truth for "nothing in
                # transit" is the queue system itself (a generated vehicle
                # sits in exactly one of loading_buffer, entrance_queue,
                # exit_queue until its completion pop). It runs at most
                # once: a veto disables all later checks and the run
                # continues to the horizon, outputs identical to unpatched.
                if self._queues_empty():
                    break
                self._early_exit_vetoed = True
        self._post_loading()

    def _queues_empty(self):
        for link in self.world.links:
            if link.loading_buffer or link.entrance_queue or \
                    link.exit_queue:
                return False
        return True

    def _post_loading(self):
        world = self.world
        # post-simulation tally (completed vehicles only): trip FFTT.
        # Tour vehicles are skipped: each leg's trip_fftt was set at
        # leg completion, and the last leg's links must not overwrite
        # the earlier legs' values.
        for agent_id in world.vehicle_order:
            veh = world.vehicles[agent_id]
            if veh.legs:
                continue
            if not veh.complete:
                continue
            veh.trip_fftt = f32(0)
            for j in range(veh.node_size - 1):
                link = world.links[veh.link_ary[j].link_no]
                veh.trip_fftt = f32(veh.trip_fftt + link.fftt)

    def _schedule_next_leg(self, veh, leg, current_time,
                           simulation_time_no):
        # A leg just completed: start the next one, or finish the tour.
        # Dwell legs leave arrival_actual + dwell_min after the leg
        # just finished (delays cascade); fixed legs leave at their
        # scheduled departure, or at arrival if that is later (a late
        # arrival can push a fixed departure, never pull it earlier).
        # The next leg is generated from its own departure bucket;
        # a bucket at or before this step's bucket never fires again,
        # so it is pushed to the next bucket.
        if veh.current_leg + 1 < len(veh.legs):
            veh.current_leg += 1
            nxt = veh.legs[veh.current_leg]
            if nxt.departure_mode == "dwell":
                d_next = float(leg.arrival_actual) + float(nxt.dwell_min)
            else:
                d_next = max(float(nxt.scheduled_departure),
                             float(leg.arrival_actual))
            veh.begin_leg(nxt)
            veh.departure = f32(d_next)
            veh.loaded = False
            veh.complete = False
            key = int(f32(d_next * f32(10)))
            if key <= simulation_time_no:
                key = simulation_time_no + 1
            self.td_map.setdefault(key, []).append(veh.agent_id)
            return d_next
        else:
            veh.complete = True
            return None

    def _deduct_link_energy(self, veh, leg, link):
        # Per-link SoC deduction (slice 3a): the energy of the link
        # just completed comes off the vehicle's charge now, so a
        # mid-leg path change (reroute, info adjustment) deducts
        # the links actually travelled, not the planned path.
        # Non-EVs and legacy vehicles (soc_kwh None) skip. The
        # first link that would drive SoC below zero strands the
        # leg at that link's to-node and clamps SoC to 0; later
        # links deduct nothing further (soc stays 0). Stranded is
        # the permanent record (once stranded, always flagged);
        # depleted is the live suppression flag, keyed off here
        # instead of stranded: the tow charge (slice 4) clears
        # depleted and restores SoC, so deduction resumes after
        # the tow while the leg stays flagged stranded. A stop
        # the vehicle strands before reaching is cancelled here:
        # it is marked done without charging and no event is
        # recorded -- including a stranding on the link INTO the
        # stop node: the vehicle never reached the charger, so
        # there is nothing to charge from. Dust-level negatives
        # (within 1e-9 kWh, float rounding on an exactly-depleting
        # leg) clamp to 0 without stranding, so an exact-0 arrival
        # at the stop still charges normally.
        if veh.soc_kwh is None:
            return
        params = self.world.ev_params(veh)
        if params is None:
            return
        if leg is not None and leg.depleted:
            return
        _battery, consumption = params
        use = float(link.length) * KM_PER_MILE * consumption
        remaining = veh.soc_kwh - use
        if remaining < -1e-9:
            veh.soc_kwh = 0.0
            if leg is not None:
                leg.stranded = True
                leg.depleted = True
                leg.stranding_count += 1
                leg.stranded_node = \
                    self.world.nodes[link.to_id].number
                stop = leg.charger_stop
                if stop is not None and not stop["done"]:
                    stop["done"] = True
        elif remaining < 0:
            veh.soc_kwh = 0.0
        else:
            veh.soc_kwh = remaining

    def _try_tow(self, veh, leg, link, arrival, travel_time):
        # Tow for a vehicle that just stranded on this link (slice
        # 4): the tow truck takes it to the nearest charger, it
        # charges there, and it resumes driving to the leg's
        # original destination. Returns True when the vehicle was
        # towed (it leaves the exit queue here and is parked until
        # the charge end); False when there is no tow, in which
        # case the caller lets it continue in traffic as before
        # (stranded, SoC 0, leg completes). No tow when there is
        # no engine (no charger file / charging=False), when the
        # leg has already been towed 3 times (cap against an
        # infinite tow loop on a pathological leg), or when the
        # engine finds no viable tow. The link departure is
        # counted exactly as a normal move. The tow blockage
        # window is recorded only (see README): other vehicles
        # are not delayed by the tow truck in v1.
        engine = getattr(self.world, "charging_engine", None)
        if engine is None or leg.tow_count >= 3:
            return False
        stranded_time = float(arrival)
        if travel_time < link.fftt:
            stranded_time += float(link.fftt - travel_time)
        plan = engine.tow_vehicle(self.world, veh, leg, link.to_id,
                                  stranded_time, link)
        if plan is None:
            return False
        world = self.world
        links = world.links
        travelled = [entry.link_no
                     for entry in veh.link_ary[:veh.sim_link_seq + 1]]
        stamps = [entry.abs_arrival
                  for entry in veh.link_ary[:veh.sim_link_seq + 1]]
        if veh.original_link_ary is None:
            self._store_original_path(veh)
        combined = travelled + list(plan["path_from_links"])
        veh.node_size = len(combined) + 1
        veh.link_ary = []
        distance = f32(0)
        trip_fftt = f32(0)
        for i, link_no in enumerate(combined):
            entry = VehicleLink(link_no)
            entry.abs_arrival = stamps[i] if i < len(stamps) \
                else f32(0)
            veh.link_ary.append(entry)
            distance = f32(distance + links[link_no].length)
            trip_fftt = f32(trip_fftt + links[link_no].fftt)
        veh.link_ary.append(VehicleLink(65530))
        veh.distance = distance
        veh.trip_fftt = trip_fftt
        # The resume reuses the parked-vehicle machinery (step()):
        # the vehicle re-enters the first charger->dest link's
        # loading buffer at the charge end. The new link_ary is
        # disconnected by design -- the jump from the stranded
        # node to the charger node is the tow -- and the writer
        # shows it in path_node_sequence from leg.tow_hops. When
        # the charger is at the destination there is no link left
        # to drive (path_from_links is empty), so there is no next
        # link: the resume completes the leg at the charge end.
        leg.tow_hops.append(
            (len(travelled), plan["site"].node_number,
             plan["arrival_charger"]))
        veh.parked = {"resume_time": plan["charge_end"],
                      "next_link_index": len(travelled)
                      if plan["path_from_links"] else None}
        self.parked_ids.append(veh.agent_id)
        veh.delay = f32(veh.delay + f32(travel_time - link.fftt))
        link.exit_queue.popleft()
        veh.soc_deducted_seq = -1
        link.cflow_departure += 1
        link.departure_count += 1
        link.departure_based_tt = f32(
            link.departure_based_tt + travel_time)
        leg.tow = True
        leg.tow_count += 1
        # A tow counts as the leg's one diversion: no liquid
        # re-trigger on the charger -> destination drive.
        leg.liquid_fired = True
        leg.depleted = False
        veh.soc_kwh = plan["target_kwh"]
        return True

    def _plan_ahead_trigger(self, veh, leg, d_next):
        # Plan-ahead trigger (v5), called from _complete_leg after
        # the dwell charge and before the next-leg SoC sync. When
        # the post-charge SoC cannot cover the remaining legs, the
        # nearest reachable charger is planned: a pending via on
        # the first non-intrazonal later leg (applied at trigger,
        # effective on next departure -- the next leg routes via
        # the charger when it generates), or a forced dwell charge
        # when the charger is AT the arrival node. A charger at
        # the next leg's destination needs no detour and records
        # nothing. The last leg of a tour never triggers (nothing
        # remains to plan for).
        engine = getattr(self.world, "charging_engine", None)
        if engine is None or veh.complete:
            return
        if leg.archetype != "plan_ahead":
            return
        if leg.stranded or leg.depleted or leg.charger_stop is not None:
            return
        if leg.node_size < 2:
            return
        params = self.world.ev_params(veh)
        if params is None:
            return
        battery, consumption = params
        later = veh.legs[veh.current_leg:]
        remaining = 0.0
        for later_leg in later:
            remaining += engine._leg_energy(self.world, later_leg,
                                            consumption)
        if remaining <= 0:
            return
        if not veh.soc_kwh < remaining:
            return
        target_leg = None
        for later_leg in later:
            if later_leg.origin_node != later_leg.dest_node:
                target_leg = later_leg
                break
        if target_leg is None:
            return
        from .decision import choose_charger_for_trigger
        departure = d_next if d_next is not None \
            else float(leg.arrival_actual)
        found = choose_charger_for_trigger(
            engine.nearest_map, "plan_ahead", leg.dest_node,
            target_leg.dest_node, veh.soc_kwh, consumption, battery,
            target_leg.enroute_target_soc, world=self.world,
            ff_net=engine._ff_net(), veh=veh, departure=departure)
        if found is None:
            return
        site, links_to, links_from, via_time = found
        if site.node_id == target_leg.dest_node:
            # Charger IS the next leg's destination: no detour
            # needed, no record; the next leg's generation-time
            # v4 rules handle charging at arrival.
            return
        if site.node_id == leg.dest_node:
            # Charger AT the arrival node: forced dwell charge,
            # unless a dwell charge already ran there (its
            # outcome already reflects the post-charge SoC, and
            # the dwell window is exhausted either way).
            if leg.decision == "destination" and \
                    leg.charge_outcome in ("charged-partial",
                                           "charged-destination",
                                           "at-target"):
                return
            engine.charge_at_site_during_dwell(
                self.world, veh, leg, site,
                float(leg.arrival_actual), d_next,
                target_leg.enroute_target_soc)
            leg.trigger = "plan_ahead"
            leg.trigger_charger_id = site.charger_id
            return
        target_leg.pending_via = (site, links_to, links_from,
                                  via_time)
        leg.trigger = "plan_ahead"
        leg.trigger_charger_id = site.charger_id

    def _emit_completed_vehicle(self, veh, current_time):
        # Legacy completion: the agent row is final now (arrival,
        # trip time and path are set; _post_loading's trip_fftt
        # recompute is not a column in the row), so render it and
        # hand it to the sink. Called from the two legacy
        # completion sites; tour vehicles never reach here.
        sink = self.sink
        if sink is None:
            return
        sink.emit_agent_row(_format_agent_vehicle_row(self.world, veh))
        sink.note_vehicle_completed()
        sink.maybe_flush(current_time, self.completed)

    def _complete_leg(self, veh, current_time, step_no):
        # Destination reached: snapshot the leg, count the leg
        # completion, and schedule the next leg (or finish the
        # tour). Intermediate legs do not count as vehicle
        # completions: veh.complete is cleared again by
        # _schedule_next_leg, and only the last leg leaves it
        # True. Energy was already deducted per link as the leg
        # travelled (slice 3a), so soc_end is the vehicle's
        # current charge and the stranded flag/node were set at
        # the crossing link; nothing is deducted again here.
        leg = veh.legs[veh.current_leg]
        veh.finish_leg(leg)
        params = self.world.ev_params(veh)
        if params is not None:
            leg.soc_end_kwh = veh.soc_kwh \
                if veh.soc_kwh is not None else leg.soc_start_kwh
            veh.soc_kwh = leg.soc_end_kwh
        else:
            leg.soc_start_kwh = 0.0
            leg.soc_end_kwh = 0.0
            leg.stranded = False
        self.completed += 1
        simulation_time_no = int(current_time + 0.0001) * 10 + step_no % 10
        d_next = self._schedule_next_leg(veh, leg, current_time,
                                         simulation_time_no)
        engine = getattr(self.world, "charging_engine", None)
        if engine is not None:
            # Dwell charging runs after the next departure is known
            # (so the window is known) and after the leg's energy
            # was deducted per link. Stranded legs are recorded but
            # never dwell-charged: an untowed one never reached its
            # destination, and a towed one already charged at the
            # tow charger (the stranded flag stays set, so the tow
            # charge is not topped up here). Charging updates
            # veh.soc_kwh, so the next leg's starting charge (mirrored
            # by _schedule_next_leg before the charge) is re-synced.
            if leg.stranded:
                engine.record_stranded(self.world, veh, leg)
            elif leg.decision == "destination":
                engine.apply_destination_charge(
                    self.world, veh, leg, float(leg.arrival_actual),
                    d_next)
            # Plan-ahead trigger (v5): on arrival, check the
            # post-dwell-charge SoC against the energy of the
            # remaining legs. Runs after the dwell charge above
            # (a dwell charge may fix the shortfall) and before
            # the next leg's SoC sync below (a forced charge at
            # this node must be visible to it).
            self._plan_ahead_trigger(veh, leg, d_next)
            if not veh.complete and veh.legs and \
                    veh.current_leg < len(veh.legs):
                veh.legs[veh.current_leg].soc_start_kwh = veh.soc_kwh
        # Incremental output (v8): the leg's rows are final now --
        # finish_leg snapshotted it, the engine block ran, and the
        # next leg's SoC sync wrote the next leg, not this one --
        # so render both rows and hand them to the sink. Formatting
        # here (not at flush time) is what makes the rows immune
        # to any later mutation of the leg.
        sink = self.sink
        if sink is not None:
            sink.emit_agent_row(
                _format_agent_leg_row(self.world, veh, leg))
            if getattr(self.world, "charging_engine", None) is not None:
                sink.emit_decision_row(
                    _format_decision_row(self.world, veh, leg, True))
            if veh.complete:
                sink.note_vehicle_completed()
            sink.maybe_flush(current_time, self.completed)

    # ----- one simulation step -----

    def step(self, current_time, step_no):
        world = self.world
        links = world.links
        time_stamp = int(current_time + 0.0001)

        # Resume vehicles parked at en-route chargers (slice 3a),
        # before stage 0: a vehicle whose charge has finished
        # re-enters the next link's loading buffer with a stamp of
        # its resume time plus that link's free-flow time, exactly
        # as stage 0 stamps a generated vehicle. sim_link_seq
        # points at the next link (never reset to 0), generated is
        # not incremented, and the decision is not re-run: the leg
        # never stopped being in progress. The loading-buffer ->
        # entrance-queue -> exit-queue flow then carries it, and
        # when the next link discharges it, link_ary[sim_link_seq]
        # is that link.
        if self.parked_ids:
            still_parked = []
            for agent_id in self.parked_ids:
                veh = world.vehicles.get(agent_id)
                if veh is None or veh.parked is None:
                    continue
                if veh.parked["resume_time"] > current_time:
                    still_parked.append(agent_id)
                    continue
                resume_time = veh.parked["resume_time"]
                next_index = veh.parked["next_link_index"]
                if next_index is None:
                    # Towed to the destination charger (slice 4):
                    # no links left to drive, so the leg completes
                    # at the charge end, as if the vehicle had
                    # driven its last link to the destination.
                    # trip_fftt is the driven links only, as the
                    # destination branch recomputes it.
                    veh.arrival = f32(resume_time)
                    veh.trip_time = f32(resume_time - veh.departure)
                    veh.travel_time = f32(
                        resume_time - veh.leaving_buffer_time)
                    veh.sim_link_seq = len(veh.link_ary) - 1
                    veh.complete = True
                    veh.trip_fftt = f32(0)
                    for j in range(veh.node_size - 1):
                        veh.trip_fftt = f32(
                            veh.trip_fftt +
                            links[veh.link_ary[j].link_no].fftt)
                    veh.parked = None
                    if veh.legs:
                        self._complete_leg(veh, resume_time, step_no)
                    else:
                        self.completed += 1
                        self._emit_completed_vehicle(veh, current_time)
                    continue
                next_link = links[veh.link_ary[next_index].link_no]
                next_node_number = -1
                if next_index + 1 < veh.node_size - 1:
                    after = links[
                        veh.link_ary[next_index + 1].link_no]
                    next_node_number = world.nodes[after.to_id].number
                next_link.loading_buffer.append(
                    [agent_id, next_node_number,
                     f32(resume_time + next_link.fftt)])
                veh.sim_link_seq = next_index
                veh.parked = None
            self.parked_ids = still_parked

        # information updating block (before generation)
        if world.info_updating_flag and \
                step_no % (10 * world.info_updating_interval) == 0:
            for link in links:
                total_delay = 0.0
                for item in link.exit_queue:
                    delay = current_time - float(item[2])
                    if delay > 0:
                        total_delay += delay
                prevailing = f32(float(link.fftt) +
                                 total_delay / max(1, len(link.exit_queue)))
                based = f32(link.departure_based_tt /
                            f32(max(1, link.departure_count)))
                if based < link.fftt:
                    based = link.fftt
                link.prevailing_tt = f32(
                    f32(f32(1 - 0.5) * prevailing) + f32(f32(0.5) * based))
                link.departure_count = 0
                link.departure_based_tt = f32(0)
            self.prevailing_net.update_current_travel_time(current_time)

        # stage 0: generation from the departure bucket of this step
        simulation_time_no = time_stamp * 10 + step_no % 10
        computation = []
        for agent_id in self.td_map.get(simulation_time_no, []):
            veh = world.vehicles.get(agent_id)
            if veh is None or veh.loaded:
                continue
            if veh.legs:
                leg = veh.legs[veh.current_leg]
                if leg.origin_node == leg.dest_node:
                    # Intrazonal instant leg: completes now, with no
                    # loading buffer and no SoC change. It starts and
                    # finishes atomically, so it counts as both
                    # generated and completed. Its departure and
                    # arrival are the step time it was generated.
                    leg.departure_actual = current_time
                    leg.arrival_actual = current_time
                    leg.leaving_buffer_time = f32(current_time)
                    leg.trip_fftt = 0.0
                    leg.delay = 0.0
                    leg.distance = f32(0)
                    leg.complete = True
                    veh.arrival = f32(current_time)
                    veh.trip_time = f32(0)
                    if world.ev_params(veh) is not None:
                        leg.soc_start_kwh = veh.soc_kwh
                        leg.soc_end_kwh = veh.soc_kwh
                    else:
                        leg.soc_start_kwh = 0.0
                        leg.soc_end_kwh = 0.0
                    self.generated += 1
                    self.completed += 1
                    self._schedule_next_leg(veh, leg, current_time,
                                            simulation_time_no)
                    # Incremental output (v8): the instant leg
                    # completes atomically here and _complete_leg
                    # is never called for it, so its rows are
                    # emitted here instead. It gets no decision
                    # and no dwell charge in v7 either; its
                    # decision fields keep their defaults, which
                    # is what the formatter renders.
                    sink = self.sink
                    if sink is not None:
                        sink.emit_agent_row(
                            _format_agent_leg_row(world, veh, leg))
                        if getattr(world, "charging_engine",
                                   None) is not None:
                            sink.emit_decision_row(
                                _format_decision_row(world, veh, leg,
                                                     True))
                        if veh.complete:
                            sink.note_vehicle_completed()
                        sink.maybe_flush(current_time, self.completed)
                    continue
                if leg.node_size < 2:
                    # No feasible path between distinct nodes:
                    # iteration-0 assignment raises for these, so
                    # this is unreachable. Skip it without
                    # generating, completing, or counting it.
                    continue
            if veh.legs:
                # The leg begins travel now: make its pre-trip
                # charging decision (path known, charge known) before
                # it enters the loading buffer. Instant legs return
                # above, so only travelling legs decide. An en-route
                # decision splices the via path here, before the
                # vehicle enters the loading buffer: the direct path
                # is stored as the original path, and the leg's
                # planning energy stays the direct-path estimate.
                # Before the splice, the stay-and-finish check may
                # keep the vehicle at its origin charger instead:
                # it re-schedules the leg for the stay end and the
                # leg is not generated on this step (veh.loaded
                # stays False), so the computation append below must
                # not run for it either.
                engine = getattr(world, "charging_engine", None)
                if engine is not None:
                    leg = veh.legs[veh.current_leg]
                    decision, site, _target = engine.decide_for_leg(
                        world, veh, leg)
                    if decision == "enroute":
                        stay_end = engine.try_stay_and_finish(
                            world, veh, leg, float(veh.departure))
                        if stay_end is not None:
                            veh.departure = f32(stay_end)
                            leg.soc_start_kwh = veh.soc_kwh
                            # Ceil, not floor: the leg is generated
                            # at the first step at or after the stay
                            # end, so it never departs before the
                            # charge it was credited has finished
                            # (floor let it leave up to 0.1 min early
                            # with the full target charge). The
                            # epsilon keeps float noise in stay_end
                            # from pushing an exact step boundary one
                            # bucket late.
                            key = int(math.ceil(stay_end * 10 - 1e-6))
                            if key <= simulation_time_no:
                                key = simulation_time_no + 1
                            self.td_map.setdefault(key, []).append(
                                veh.agent_id)
                            continue
                    # After the stay resolution (stay or not), the
                    # leg's starting charge is the vehicle's charge
                    # before it enters the network. Non-EVs carry
                    # no charge (veh.soc_kwh is None), so they keep
                    # 0.0 -- the value begin_leg set -- rather than
                    # None, which the writer cannot render for a leg
                    # that never completes: a completed non-EV leg
                    # is reset to 0.0 in _complete_leg, but a
                    # never-completed one kept the None and crashed
                    # v7's end-of-run writer.
                    leg.soc_start_kwh = veh.soc_kwh \
                        if veh.soc_kwh is not None else 0.0
                    if decision == "enroute":
                        if site is None or leg.enroute_via is None:
                            raise ProgramStop(
                                "agent %d leg %d decided en-route "
                                "with no charger or via path"
                                % (veh.agent_id, leg.leg_index))
                        links_to, links_from, _via_time = \
                            leg.enroute_via
                        self._build_via_path(veh, leg, site, links_to,
                                             links_from)
            if veh.info_type in (2, 3):
                computation.append(veh)
            if veh.node_size >= 2:
                first = links[veh.link_ary[0].link_no]
                stamp = f32(veh.departure + first.fftt)
                next_node_number = -1
                if veh.node_size >= 3:
                    next_node_number = world.nodes[
                        links[veh.link_ary[1].link_no].to_id].number
                first.loading_buffer.append(
                    [agent_id, next_node_number, stamp])
                veh.loaded = True
                veh.sim_link_seq = 0
                self.generated += 1
                if veh.legs:
                    # The leg begins travel now: record its actual
                    # departure (the generation step time). Its
                    # starting charge was set in begin_leg and does
                    # not change while waiting at a node.
                    leg = veh.legs[veh.current_leg]
                    leg.departure_actual = current_time
        for veh in computation:
            self.generation_adjustment(veh, current_time)

        # stage 1: loading buffer -> entrance queue (storage gate)
        for link in links:
            while link.loading_buffer and link.outflow_lanes > 0.01:
                item = link.loading_buffer[0]
                stamp = item[2]
                if less_eq(stamp, current_time):
                    stamp = f32(current_time)
                on_link = link.cflow_arrival - link.cflow_departure
                available = f32(link.space_capacity - f32(on_link))
                threshold = f32(link.space_capacity *
                                f32(f32(1) - f32(0.8)))
                if threshold < 2:
                    threshold = f32(2)
                if available >= threshold:
                    link.loading_buffer.popleft()
                    item[2] = stamp
                    link.entrance_queue.append(item)
                    link.cflow_arrival += 1
                    veh = world.vehicles[item[0]]
                    link.counts_demand[veh.demand_type] += 1
                    link.counts_vtype[veh.vehicle_type] += 1
                    # Only the leg's first link stamps the buffer
                    # exit: a parked resume (en-route park, tow)
                    # re-enters here with sim_link_seq > 0, and
                    # overwriting would corrupt the leg's
                    # path_time_sequence first stamp and
                    # veh.travel_time with the resume time.
                    if veh.sim_link_seq == 0:
                        veh.leaving_buffer_time = f32(current_time)
                    link.buffer_waiting_time = f32(
                        float(link.buffer_waiting_time) +
                        (current_time - float(veh.departure)))
                else:
                    break

        # stage 2: entrance queue -> exit queue at the free-flow stamp
        for link in links:
            while link.entrance_queue:
                item = link.entrance_queue[0]
                if less_eq(item[2], current_time):
                    link.entrance_queue.popleft()
                    link.exit_queue.append(item)
                else:
                    break

        # stage 3: link in/out capacities (Newell only). Plain-float
        # arithmetic on the per-link constants precomputed in
        # link.setup_loading (link.s3_consts); f32() is re-applied
        # only at the two non-f32 inputs, f_in before the cum-in add
        # and the ring+jam add for n_allow. Exactness: a double add
        # of two float32 values is exact here (the addends span at
        # most ~48 significand bits, under the 53-bit double), so the
        # one f32() rounding equals the one float32 add it replaces;
        # f32(min(x, y)) with x float32 is likewise exact by the
        # monotonicity of rounding. Accumulation order is unchanged.
        relax = current_time >= world.end + 60
        for link in links:
            cap, cap_relax, inflow_cap, jam, inflow_rate = \
                link.s3_consts
            capacity = cap_relax if relax else cap
            link.cum_out_counter = f32(float(link.cum_out_counter) + \
                                     capacity)
            link.link_out_capacity = int(link.cum_out_counter) - \
                link.cum_out_prev
            link.cum_out_prev += link.link_out_capacity
            on_link = link.cflow_arrival - link.cflow_departure
            f_in = 99999.0
            # Space-capacity inflow limiting applies to freeway
            # links only. The old signal-model gate is gone: with no
            # signal model, this build keeps the default behaviour
            # (old signal_representation_model = 0). Networks that
            # previously set signal_model = 2 will see different
            # arterial inflow behaviour.
            if "f" in link.type_code:
                f_in = min(float(link.space_capacity) - on_link,
                         inflow_cap)
                if f_in < 0:
                    f_in = 0.0
                if step_no >= link.backward_wave_steps:
                    slot = max(0, step_no - link.backward_wave_steps) % 300
                    n_allow = int(f32(float(link.ring_departure[slot]) + \
                                  jam))
                    prev_slot = max(0, step_no - 1) % 300
                    flow_allowed = n_allow - int(link.ring_arrival[
                        prev_slot])
                    if flow_allowed < 0:
                        flow_allowed = 0
                    if f_in > flow_allowed:
                        f_in = float(flow_allowed)
                if relax:
                    inflow_rate = cap
                if f_in < inflow_rate:
                    f_in = inflow_rate
            link.cum_in_counter = f32(float(link.cum_in_counter) + \
                                    f32(f_in))
            link.link_in_capacity = int(link.cum_in_counter) - \
                link.cum_in_prev
            link.cum_in_prev += link.link_in_capacity
            if "r" in link.type_code:
                if link.on_ramp and world.use_pq_on_ramps == 1:
                    link.link_in_capacity = 99999
                if not link.on_ramp and world.use_pq_off_ramps == 1:
                    link.link_in_capacity = 99999
        # merge distribution of the downstream in-capacity
        for link in links:
            if link.merge_flag < 1:
                continue
            total_demand = sum(
                len(links[entry[0]].exit_queue)
                for entry in link.merge_incoming)
            if total_demand <= link.link_in_capacity:
                continue
            if len(link.merge_incoming) == 2:
                for il, entry in enumerate(link.merge_incoming):
                    other = link.merge_incoming[1 - il][0]
                    entry[2] = f32(link.link_in_capacity -
                                   len(links[other].exit_queue))
            for entry in link.merge_incoming:
                incoming = links[entry[0]]
                cap1 = f32(f32(link.link_in_capacity) * entry[1])
                cap2 = entry[2]
                cap3 = f32(incoming.link_out_capacity)
                merged = min(cap3, max(cap1, cap2))
                prev = incoming.cum_merge_counter
                incoming.cum_merge_counter = f32(
                    incoming.cum_merge_counter + merged)
                incoming.link_out_capacity = \
                    int(incoming.cum_merge_counter) - int(prev)

        # stage 4: node transfer
        self._node_transfer(current_time, step_no)

        # stage 5: ring buffers for Newell's backward wave. Only
        # freeway links are read (stage-3 freeway branch), so only
        # they need their ring written; the Swedish network writes
        # every link as freeway type, so this saves ~0 there.
        slot = step_no % 300
        freeway_links = self.freeway_links
        if freeway_links is None:
            freeway_links = self.freeway_links = \
                [link for link in links if "f" in link.type_code]
        for link in freeway_links:
            link.ring_arrival[slot] = f32(link.cflow_arrival)
            link.ring_departure[slot] = f32(link.cflow_departure)

        # Incremental-output cadence, sim-time leg: a quiet
        # stretch still ages the progress file, so check even on
        # steps where nothing completed. O(1): one float compare,
        # no wall-clock read here.
        self.current_time = current_time
        if self.sink is not None:
            self.sink.maybe_flush(current_time, self.completed)

    def _node_transfer(self, current_time, step_no):
        world = self.world
        links = world.links
        for node in world.nodes:
            incoming = node.incoming
            if not incoming:
                continue
            size = len(incoming)
            sequence = step_no % size
            count = 0
            while count < size:
                link = links[incoming[sequence]]
                if link.exit_queue:
                    self._discharge_link(link, node, current_time,
                                         step_no)
                count += 1
                sequence = (sequence + 1) % size

    def _discharge_link(self, link, node, current_time, step_no):
        world = self.world
        links = world.links
        vehicle_out_count = link.link_out_capacity
        if len(link.exit_queue) <= link.link_out_capacity:
            vehicle_out_count = len(link.exit_queue)
        # PCE pre-walk over the whole exit queue
        demand_counter = f32(0)
        agent_counter = 0
        for item in link.exit_queue:
            veh = world.vehicles[item[0]]
            demand_counter = f32(demand_counter + veh.pce)
            if demand_counter > vehicle_out_count:
                difference = f32(demand_counter - f32(vehicle_out_count))
                chance = float(f32(veh.pce - difference)) / \
                    max(0.0001, float(veh.pce))
                if link.rng.ratio() < chance:
                    agent_counter += 1
                break
            agent_counter += 1
        vehicle_out_count = agent_counter
        through_blocking = 0
        while link.exit_queue and vehicle_out_count > 0:
            if through_blocking > 0:
                break
            item = link.exit_queue[0]
            veh = world.vehicles[item[0]]
            seq = veh.sim_link_seq
            if seq >= 1:
                t_link_arrival = int(veh.link_ary[seq - 1].abs_arrival)
            else:
                t_link_arrival = int(veh.departure)
            number_of_links = veh.node_size - 1
            if less_eq(current_time - SIM_INTERVAL, item[2]):
                arrival = item[2]
            else:
                arrival = f32(current_time)
            veh.link_ary[seq].abs_arrival = arrival
            if seq >= 1:
                travel_time = f32(arrival - veh.link_ary[seq - 1].abs_arrival)
            else:
                travel_time = f32(arrival - veh.departure)
            # Per-link SoC deduction (slice 3a): the link just
            # completed, before any stop check or move, so the
            # charge at a charger stop sees the post-link SoC and
            # the destination branch below deducts this link too.
            # Deducted once per link: a vehicle blocked on its
            # next link stays in the exit queue and is retried
            # next step, so soc_deducted_seq records the link
            # already charged for and the retry deducts nothing;
            # every committed pop below clears it again. The
            # deduction itself stays here, ahead of the stop
            # check, so a stranding on the link into the stop
            # still cancels the stop before charge_enroute runs.
            leg = veh.legs[veh.current_leg] if veh.legs else None
            was_depleted = leg.depleted if leg is not None else False
            if veh.soc_deducted_seq != seq:
                self._deduct_link_energy(veh, leg, link)
                veh.soc_deducted_seq = seq
            # Tow (slice 4): a vehicle that just stranded mid-leg
            # stops here instead of continuing in traffic on empty.
            # Only a fresh stranding on this link tows (a blocked
            # retry re-enters with depleted already set, so the tow
            # is attempted once), and only mid-leg: a stranding on
            # the last link has already reached the destination and
            # completes below. _try_tow returns False when towing
            # is unavailable, and the vehicle continues as before.
            if leg is not None and leg.depleted and not was_depleted \
                    and seq < number_of_links - 1 and \
                    self._try_tow(veh, leg, link, arrival, travel_time):
                vehicle_out_count -= 1
                continue
            if seq < number_of_links - 1:
                # Liquid-fuel trigger (v5): after the per-link
                # deduction and the tow attempt, before the user
                # reroute callback. Fires at most once per leg,
                # when the post-deduction SoC drops below the leg's
                # threshold. A splice sets rerouted (the callback
                # is skipped for this transition); a charger at
                # the current node parks in place instead, and the
                # existing en-route stop check below charges it
                # in this same discharge call.
                rerouted = False
                if leg is not None and leg.archetype == "liquid" \
                        and not leg.liquid_fired \
                        and not leg.stranded and not leg.depleted \
                        and leg.charger_stop is None \
                        and leg.node_size >= 2:
                    engine = getattr(world, "charging_engine", None)
                    params = world.ev_params(veh) \
                        if engine is not None else None
                    if params is not None:
                        battery, consumption = params
                        if veh.soc_kwh < leg.soc_threshold_frac \
                                * battery:
                            from .decision import \
                                choose_charger_for_trigger
                            found = choose_charger_for_trigger(
                                engine.nearest_map, "liquid",
                                link.to_id, veh.dest_node,
                                veh.soc_kwh, consumption, battery,
                                leg.enroute_target_soc, world=world,
                                ff_net=engine._ff_net(), veh=veh,
                                departure=float(arrival))
                            leg.liquid_fired = True
                            if found is not None:
                                site, links_to, links_from, \
                                    via_time = found
                                if site.node_id == veh.dest_node:
                                    # Charger IS the destination:
                                    # driving straight there anyway,
                                    # no splice, no record.
                                    pass
                                elif site.node_id == link.to_id:
                                    # Charger AT the current node:
                                    # park in place (no splice); the
                                    # en-route stop check below runs
                                    # charge_enroute in this call.
                                    leg.charger_stop = {
                                        "node": site.node_id,
                                        "site": site, "done": False,
                                        "target_soc":
                                            leg.enroute_target_soc}
                                    leg.trigger = "liquid"
                                    leg.trigger_charger_id = \
                                        site.charger_id
                                else:
                                    self._splice_mid_leg_via(
                                        veh, leg, site, links_to,
                                        links_from, via_time)
                                    rerouted = True
                                    number_of_links = \
                                        veh.node_size - 1
                # Mid-run reroute hook (charging fork): the vehicle has
                # just completed this link and is at its to-node. A
                # user callback may replace the rest of its path from
                # that node onward, e.g. to detour via a charger. It
                # runs before the vehicle commits to its next link, so
                # a returned chain starts at the arrival node. Without
                # a callback, or when it returns None, the preassigned
                # path stands.
                arrival_node = link.to_id
                if arrival_node != veh.dest_node and not rerouted and \
                        world.reroute_callback is not None:
                    new_links = world.reroute_callback(
                        veh, arrival_node, world)
                    if new_links:
                        self._splice_reroute(veh, arrival_node, new_links)
                        number_of_links = veh.node_size - 1
                        rerouted = True
                next_link = links[veh.link_ary[seq + 1].link_no]
                minimum = link.fftt
                if travel_time < minimum:
                    arrival = f32(arrival + f32(minimum - travel_time))
                    travel_time = minimum
                # En-route charger stop (slice 3a): the vehicle
                # just completed the link into its stop node. The
                # engine charges it (or finds it already at/above
                # its target, in which case it drives through and
                # the stop is simply marked done). A charge parks
                # the vehicle: it leaves the exit queue, enters no
                # link, and resumes from the parked list at the
                # charge end time. The completed link's departure
                # is counted as in a normal move; the dwell itself
                # is not added to veh.delay here, but it lands in
                # the post-stop link's travel time when that link
                # discharges (its arrival minus this link's
                # abs_arrival spans the park), and veh.delay is
                # write-only in this build (copied to leg.delay,
                # never read), so that is only a reporting note.
                if leg is not None and leg.charger_stop is not None \
                        and not leg.charger_stop["done"] and \
                        link.to_id == leg.charger_stop["node"]:
                    engine = getattr(world, "charging_engine", None)
                    if engine is not None:
                        end = engine.charge_enroute(
                            world, veh, leg, float(arrival))
                        if end is not None:
                            veh.delay = f32(
                                veh.delay +
                                f32(travel_time - link.fftt))
                            veh.parked = {
                                "resume_time": end,
                                "next_link_index": seq + 1}
                            self.parked_ids.append(veh.agent_id)
                            link.exit_queue.popleft()
                            veh.soc_deducted_seq = -1
                            link.cflow_departure += 1
                            link.departure_count += 1
                            link.departure_based_tt = f32(
                                link.departure_based_tt + travel_time)
                            vehicle_out_count -= 1
                            continue
                if next_link.link_in_capacity > 0:
                    veh.delay = f32(veh.delay +
                                    f32(travel_time - link.fftt))
                    veh.sim_link_seq = seq + 1
                    new_item = [item[0], -1,
                                f32(arrival + next_link.fftt)]
                    if veh.sim_link_seq < number_of_links - 2:
                        next_next = links[
                            veh.link_ary[veh.sim_link_seq + 2].link_no]
                        new_item[1] = world.nodes[next_next.to_id].number
                    next_link.entrance_queue.append(new_item)
                    link.exit_queue.popleft()
                    veh.soc_deducted_seq = -1
                    next_link.cflow_arrival += 1
                    next_link.counts_demand[veh.demand_type] += 1
                    next_link.counts_vtype[veh.vehicle_type] += 1
                    link.cflow_departure += 1
                    link.departure_count += 1
                    link.departure_based_tt = f32(
                        link.departure_based_tt + travel_time)
                    next_link.link_in_capacity -= 1
                    # Built-in en-route adjustment (info 3), gated on
                    # not reaching the destination node with this move
                    # and not already rerouted by the callback above.
                    current_link_no = veh.link_ary[veh.sim_link_seq].link_no
                    next_node = links[current_link_no].to_id
                    if next_node != veh.dest_node and not rerouted and \
                            veh.info_type == 3:
                        self.enroute_adjustment(veh, current_time)
                else:
                    through_blocking += 1
            else:
                # destination reached on this link
                arrival = f32(current_time)
                veh.link_ary[seq].abs_arrival = arrival
                if seq >= 1:
                    travel_time = f32(
                        arrival - veh.link_ary[seq - 1].abs_arrival)
                else:
                    travel_time = f32(arrival - veh.departure)
                veh.delay = f32(veh.delay + f32(travel_time - link.fftt))
                veh.sim_link_seq = seq + 1
                veh.arrival = arrival
                veh.trip_time = f32(arrival - veh.departure)
                veh.travel_time = f32(arrival - veh.leaving_buffer_time)
                veh.complete = True
                veh.trip_fftt = f32(0)
                for j in range(veh.node_size - 1):
                    veh.trip_fftt = f32(
                        veh.trip_fftt + links[veh.link_ary[j].link_no].fftt)
                link.cflow_departure += 1
                link.departure_count += 1
                link.departure_based_tt = f32(
                    link.departure_based_tt + travel_time)
                link.exit_queue.popleft()
                veh.soc_deducted_seq = -1
                if veh.legs:
                    self._complete_leg(veh, current_time, step_no)
                else:
                    self.completed += 1
                    self._emit_completed_vehicle(veh, current_time)
            vehicle_out_count -= 1
