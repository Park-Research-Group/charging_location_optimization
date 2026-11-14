# Run one DTALite loading pass over an input directory, mirroring the exe
# flow: settings -> scenario -> input files -> iteration-0 path assignment
# (agent-based driver) -> network loading -> the output file. The input
# directory is used as given; it is never written to, and it does not
# need any particular name. This build writes only output_agent.csv.
import os
import sys
import time

from .charging import _any_trigger
from .cpp import ProgramStop
from .incremental import IncrementalWriter
from .loading import Simulator
from .network import World
from .outputs import write_agent_file
from .progress import ProgressBar, RunProgress


def run_loading(input_dir, output_dir, reroute_callback=None,
                charging=None, workers=None, incremental=True,
                progress=True, flush_legs=None, flush_sim_minutes=None):
    # incremental=True (default) writes outputs as legs complete
    # (v8); False restores exact v7 behaviour (all end-of-run
    # writers). progress is the console-bar switch: the progress
    # FILE (run_progress.json) is written either way -- it is the
    # durable signal and the bar is only the console convenience
    # (design section 6). The bar is display only; turning it off
    # changes no output file.
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
    # Console progress bar (design section 6): created once, before
    # assignment, with the sim range the loading percent needs.
    # run.py announces phase changes and finishes it; the Simulator
    # owns the per-step and per-key ticks. progress=False disables
    # the bar only -- the progress file is unaffected.
    bar = ProgressBar(enabled=progress, sim_start=world.start,
                      sim_horizon=world.horizon)
    sim = None
    writer = None
    prog = None
    final_rows = {}
    try:
        if incremental:
            prog = RunProgress(output_dir)
            writer = IncrementalWriter(
                output_dir, world.tour_mode,
                world.charging_engine is not None,
                engine=world.charging_engine,
                flush_legs=flush_legs,
                flush_sim_minutes=flush_sim_minutes,
                progress=prog).open()
            sim = Simulator(world, sink=writer, progress=prog,
                            bar=bar)
            prog.state_fn = lambda: _progress_state(
                world, sim, writer, prog, output_dir, final_rows)
            prog.write_now()
            bar.set_phase("assigning")
            prog.set_phase("assigning")
            prog.write_now()
            sim.assign_paths_iteration_zero(workers=workers)
            if world.charging_engine is not None:
                bar.set_phase("map")
                prog.set_phase("map")
                prog.write_now()
                world.charging_engine.build_nearest_map()
            prog.set_phase("loading")
            prog.write_now()
            sim.network_loading()
            bar.set_phase("finalizing")
            prog.set_phase("finalizing")
            prog.write_now()
            writer.final_sweep(world)
            has_trigger = world.charging_engine is not None and \
                _any_trigger(world)
            writer.close(has_trigger)
            if charge_cfg is not None:
                if world.charging_engine is not None:
                    world.charging_summary = \
                        world.charging_engine.finalize_summary(
                            world, output_dir)
                else:
                    from .charging import run_charging
                    world.charging_summary = run_charging(
                        world, output_dir, charge_cfg)
            else:
                world.charging_summary = None
        else:
            # Exact v7 path: all writers end-of-run. The progress
            # file still gets its phase writes; its rows_written
            # is filled in from the final counts below.
            prog = RunProgress(output_dir)
            sim = Simulator(world, progress=prog, bar=bar)
            prog.state_fn = lambda: _progress_state(
                world, sim, writer, prog, output_dir, final_rows)
            prog.write_now()
            bar.set_phase("assigning")
            prog.set_phase("assigning")
            prog.write_now()
            sim.assign_paths_iteration_zero(workers=workers)
            if world.charging_engine is not None:
                bar.set_phase("map")
                prog.set_phase("map")
                prog.write_now()
                world.charging_engine.build_nearest_map()
            prog.set_phase("loading")
            prog.write_now()
            sim.network_loading()
            bar.set_phase("finalizing")
            prog.set_phase("finalizing")
            prog.write_now()
            os.makedirs(output_dir, exist_ok=True)
            write_agent_file(world, os.path.join(output_dir,
                                                 "output_agent.csv"))
            if charge_cfg is not None:
                if world.charging_engine is not None:
                    world.charging_summary = \
                        world.charging_engine.finalize(world, output_dir)
                else:
                    from .charging import run_charging
                    world.charging_summary = run_charging(
                        world, output_dir, charge_cfg)
            else:
                world.charging_summary = None
            final_rows = _final_rows_v7(world, sim)
        # End the bar before any further printing, so the summary
        # lines below start on a fresh line (design section 6).
        bar.finish()
        # v7: report degraded nearest-charger lookups (same style as
        # the iteration-0 cache stats). The consistency rule in
        # nearest.py should keep this at zero; a nonzero count is
        # visible, not silent, and means the map still has a hole.
        engine = world.charging_engine
        if engine is not None and engine.nearest_map is not None:
            broken = engine.nearest_map.broken_lookups
            if broken > 0:
                print("nearest-charger map: %d broken lookups "
                      "degraded to no-charger" % broken)
        prog.mark_complete()
    except Exception as exc:
        # Best-effort failed progress write, then re-raise. The
        # writer is aborted FIRST: its flush would otherwise
        # overwrite the failed status with a running one. A hard
        # kill cannot run this handler; the progress file then
        # simply stays at its last flush with status "running".
        if writer is not None:
            writer.abort()
        bar.finish()
        if prog is not None:
            prog.mark_failed(exc)
        raise
    return world


def _progress_state(world, sim, writer, prog, output_dir, final_rows):
    # One run_progress.json document (design section 5). Unknown
    # counts are 0, unknown times are null. rows_written counts
    # rows actually flushed to disk, not rows buffered.
    if sim is not None and getattr(sim, "current_time", None) is not None:
        sim_time = sim.current_time
    elif writer is not None and writer.sim_time is not None:
        sim_time = writer.sim_time
    else:
        sim_time = world.start
    if writer is not None:
        agents_completed = writer.agents_completed
        rows_written = dict(writer.rows_written)
        last_flush_sim = writer.last_flush_sim_time
        last_flush_wall = writer.last_flush_wall_time
    else:
        agents_completed = sum(
            1 for veh in world.vehicles.values() if veh.complete)
        rows_written = dict(final_rows)
        last_flush_sim = None
        last_flush_wall = None
    return {
        "format": "pysim-run-progress/1",
        "status": "running",
        "phase": prog.phase,
        "sim_time_min": sim_time,
        "sim_day": int(sim_time // 1440),
        "legs_completed": sim.completed if sim is not None else 0,
        "legs_total": getattr(sim, "n_generatable", 0)
        if sim is not None else 0,
        "legs_generated": sim.generated if sim is not None else 0,
        "agents_completed": agents_completed,
        "agents_total": len(world.vehicles),
        "assignment_keys_resolved": prog.assign_keys_resolved,
        "assignment_keys_total": prog.assign_keys_total,
        "wall_elapsed_s": round(time.monotonic() - prog.started_mono, 1),
        "started_wall_time": prog.started_wall_time,
        "last_flush_sim_time_min": last_flush_sim,
        "last_flush_wall_time": last_flush_wall,
        "rows_written": rows_written,
        "output_dir": os.path.abspath(output_dir),
    }


def _final_rows_v7(world, sim):
    # Row counts for the progress file's final write on the
    # --no-incremental path, where no writer tracked flushes.
    # Keys are omitted for files the run does not produce.
    rows = {}
    if world.tour_mode:
        rows["output_agent.csv"] = getattr(sim, "n_generatable", 0)
    else:
        rows["output_agent.csv"] = len(world.vehicles)
    engine = getattr(world, "charging_engine", None)
    if engine is not None:
        rows["output_decisions.csv"] = getattr(
            sim, "n_generatable", 0)
        rows["output_charging.csv"] = len(engine.events)
        rows["output_stranded.csv"] = len(engine.stranded)
    elif world.charging_summary is not None:
        rows["output_charging.csv"] = \
            world.charging_summary["charges"]
        rows["output_stranded.csv"] = \
            world.charging_summary["stranded"]
    return rows


def main(argv):
    # Optional --workers N (or --workers=N) after the two positional
    # arguments; plain `python -m pysim.run <in> <out>` is unchanged
    # and uses the automatic choice (pool when beneficial, else
    # serial). See sp_mp for the Windows `__main__` note: this
    # module already runs under its own guard, so the pool works.
    # v8 flags: --no-progress (console bar off; the progress file
    # is still written), --no-incremental (exact v7 end-of-run
    # behaviour), --flush-legs N / --flush-minutes M (flush
    # cadence tuning; defaults 10000 legs / 60 sim-minutes).
    workers = None
    incremental = True
    progress = True
    flush_legs = None
    flush_sim_minutes = None
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
        elif arg == "--no-progress":
            progress = False
            i += 1
        elif arg == "--no-incremental":
            incremental = False
            i += 1
        elif arg == "--flush-legs" and i + 1 < len(argv):
            try:
                flush_legs = int(argv[i + 1])
            except ValueError:
                print("python -m pysim.run: --flush-legs needs an integer")
                return 2
            i += 2
        elif arg.startswith("--flush-legs="):
            try:
                flush_legs = int(arg.split("=", 1)[1])
            except ValueError:
                print("python -m pysim.run: --flush-legs needs an integer")
                return 2
            i += 1
        elif arg == "--flush-minutes" and i + 1 < len(argv):
            try:
                flush_sim_minutes = float(argv[i + 1])
            except ValueError:
                print("python -m pysim.run: --flush-minutes needs a number")
                return 2
            i += 2
        elif arg.startswith("--flush-minutes="):
            try:
                flush_sim_minutes = float(arg.split("=", 1)[1])
            except ValueError:
                print("python -m pysim.run: --flush-minutes needs a number")
                return 2
            i += 1
        else:
            args.append(arg)
            i += 1
    if len(args) < 2:
        print("usage: python -m pysim.run <input_dir> <output_dir> "
              "[--workers N] [--no-progress] [--no-incremental] "
              "[--flush-legs N] [--flush-minutes M]")
        return 2
    run_loading(args[0], args[1], workers=workers,
                incremental=incremental, progress=progress,
                flush_legs=flush_legs,
                flush_sim_minutes=flush_sim_minutes)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
