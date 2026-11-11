"""Public entry point for the charging-fork pysim folder.

Import it with the folder's parent on sys.path::

    from pysim import run_pysim

    result = run_pysim("my_inputs", "my_outputs")

    # With a mid-run reroute hook (see README for the contract):
    result = run_pysim("my_inputs", "my_outputs",
                       reroute_callback=my_callback)
"""
import os

from .run import run_loading


def run_pysim(input_dir, output_dir, verbose=False, reroute_callback=None,
              charging=None, workers=None):
    """Run one PySim loading pass.

    input_dir is a directory holding DTALite-format input files. It is
    read, never written, and it does not need to be called
    DTALite_Files or anything else: any name works. output_dir is
    created if needed and receives output_agent.csv, the only traffic
    output.

    reroute_callback is None by default (no hook: vehicles follow their
    paths except for the built-in info-type adjustments). When given,
    it must be a callable (vehicle, next_node, world) -> list of link
    numbers, or None to keep the vehicle's current path. It is called
    each time a vehicle completes a link, before it enters the next
    one: next_node is the internal id of the node just reached, and a
    returned list must be a connected chain from that node to the
    vehicle's destination. Bad chains raise ProgramStop.

    charging controls the charging accounting post-pass (see README):
    None (default) runs it when input_charger.csv is present, with
    default parameters; False disables it; a dict enables it with
    those parameters overriding the defaults. When it runs, three
    more files are written: output_charging.csv,
    output_charger_summary.csv, and output_stranded.csv. On a tour
    input it runs the inline engine instead (pre-trip decisions and
    dwell charging) and also writes output_decisions.csv, listed
    under outputs["decisions"].

    workers controls the iteration-0 shortest-path multiprocessing
    (see pysim.sp_mp): None (default) is automatic -- a spawn Pool
    when there are enough distinct origin/destination keys and more
    than one CPU, serial otherwise; 1 (or 0) is always serial; N >= 2
    uses a Pool of N workers. Pool and serial runs produce identical
    outputs, and any pool failure falls back to the serial path, so
    a run that would have worked serially still works. The
    PYSIM_SP_WORKERS environment variable overrides the automatic
    choice when workers is None (set it to 1 to force serial).
    A driver script that calls run_pysim must put the call under
    `if __name__ == "__main__":` for the pool to work on Windows
    (spawn re-imports the calling script); `python -m pysim.run`
    already satisfies this.

    Returns a dict with the input/output dirs, the output paths, the
    agent counts, and the charging summary (or None). Raises ProgramStop
    (from pysim.cpp) on out-of-scope scenarios: flow models other than
    Newell (2), scenarios requesting more than one iteration, a reroute
    chain that fails validation, or charging requested without its
    input file.
    """
    input_dir = os.path.abspath(input_dir)
    output_dir = os.path.abspath(output_dir)
    if not os.path.isdir(input_dir):
        raise ValueError("input_dir does not exist: %s" % input_dir)
    if verbose:
        print("pysim: %s -> %s" % (input_dir, output_dir))
    world = run_loading(input_dir, output_dir,
                        reroute_callback=reroute_callback,
                        charging=charging, workers=workers)
    agents = len(world.vehicles)
    completed = sum(1 for veh in world.vehicles.values() if veh.complete)
    outputs = {
        "agent": os.path.join(output_dir, "output_agent.csv"),
    }
    if world.charging_summary is not None:
        outputs["charging"] = os.path.join(output_dir,
                                           "output_charging.csv")
        outputs["charger_summary"] = os.path.join(
            output_dir, "output_charger_summary.csv")
        outputs["stranded"] = os.path.join(output_dir,
                                           "output_stranded.csv")
        if getattr(world, "charging_engine", None) is not None:
            outputs["decisions"] = os.path.join(
                output_dir, "output_decisions.csv")
    result = {
        "input_dir": input_dir,
        "output_dir": output_dir,
        "outputs": outputs,
        "agents": agents,
        "completed": completed,
        "charging": world.charging_summary,
    }
    if verbose:
        print("pysim: done, %d agents, %d completed"
              % (agents, completed))
    return result
