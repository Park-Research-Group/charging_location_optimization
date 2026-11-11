# Run one DTALite loading pass over an input directory, mirroring the exe
# flow: settings -> scenario -> input files -> iteration-0 path assignment
# (agent-based driver) -> network loading -> the output file. The input
# directory is used as given; it is never written to, and it does not
# need any particular name. This build writes only output_agent.csv.
import os
import sys

from .cpp import ProgramStop
from .loading import Simulator
from .network import World
from .outputs import write_agent_file


def run_loading(input_dir, output_dir, reroute_callback=None,
                charging=None, workers=None):
    world = World(input_dir)
    world.reroute_callback = reroute_callback
    if world.flow_model != 2:
        raise ProgramStop(
            "this build implements the Newell flow model only; "
            "traffic_flow_model=%d is out of scope" % world.flow_model)
    if world.num_iterations != 0:
        raise ProgramStop(
            "pysim implements a single loading pass; the scenario requests "
            "%d assignment iterations" % (world.num_iterations + 1))
    world.read_inputs()
    # Charging accounting config, resolved before simulation starts:
    # charging=None means auto (runs when input_charger.csv is
    # present, with default parameters); charging=False disables it
    # always; a dict enables it with those parameters overriding the
    # defaults (and requires the file). Tour mode gets the inline
    # ChargingEngine (slice 2); legacy mode keeps the v1 post-pass.
    charger_path = os.path.join(input_dir, "input_charger.csv")
    charge_cfg = None
    if charging is False:
        charge_cfg = None
    elif isinstance(charging, dict):
        if not os.path.exists(charger_path):
            raise ProgramStop(
                "charging requested but input_charger.csv is missing")
        from .charging import DEFAULT_CONFIG
        charge_cfg = dict(DEFAULT_CONFIG)
        charge_cfg.update(charging)
    elif os.path.exists(charger_path):
        from .charging import DEFAULT_CONFIG
        charge_cfg = dict(DEFAULT_CONFIG)
    world.charging_engine = None
    if charge_cfg is not None and world.tour_mode:
        from .charging import ChargingEngine
        world.charging_engine = ChargingEngine(world, charge_cfg)
    sim = Simulator(world)
    sim.assign_paths_iteration_zero(workers=workers)
    if world.charging_engine is not None:
        world.charging_engine.build_nearest_map()
    sim.network_loading()
    os.makedirs(output_dir, exist_ok=True)
    write_agent_file(world, os.path.join(output_dir, "output_agent.csv"))
    if charge_cfg is not None:
        if world.charging_engine is not None:
            world.charging_summary = world.charging_engine.finalize(
                world, output_dir)
        else:
            from .charging import run_charging
            world.charging_summary = run_charging(world, output_dir,
                                                  charge_cfg)
    else:
        world.charging_summary = None
    return world


def main(argv):
    # Optional --workers N (or --workers=N) after the two positional
    # arguments; plain `python -m pysim.run <in> <out>` is unchanged
    # and uses the automatic choice (pool when beneficial, else
    # serial). See sp_mp for the Windows `__main__` note: this
    # module already runs under its own guard, so the pool works.
    workers = None
    args = []
    i = 1
    while i < len(argv):
        arg = argv[i]
        if arg == "--workers" and i + 1 < len(argv):
            try:
                workers = int(argv[i + 1])
            except ValueError:
                print("python -m pysim.run: --workers needs an integer")
                return 2
            i += 2
        elif arg.startswith("--workers="):
            try:
                workers = int(arg.split("=", 1)[1])
            except ValueError:
                print("python -m pysim.run: --workers needs an integer")
                return 2
            i += 1
        else:
            args.append(arg)
            i += 1
    if len(args) < 2:
        print("usage: python -m pysim.run <input_dir> <output_dir> "
              "[--workers N]")
        return 2
    run_loading(args[0], args[1], workers=workers)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
