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


def run_loading(input_dir, output_dir, reroute_callback=None):
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
    sim = Simulator(world)
    sim.assign_paths_iteration_zero()
    sim.network_loading()
    os.makedirs(output_dir, exist_ok=True)
    write_agent_file(world, os.path.join(output_dir, "output_agent.csv"))
    return world


def main(argv):
    if len(argv) < 3:
        print("usage: python -m pysim.run <input_dir> <output_dir>")
        return 2
    run_loading(argv[1], argv[2])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
