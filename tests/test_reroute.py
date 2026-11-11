# Mid-run reroute test for the charging-fork pysim.
#
# Tiny network: 1 -> 2 -> 3 -> 5 (main route) and 2 -> 4 -> 5
# (diversion). Agent 1 has the main route preassigned and a tour id;
# a callback diverts it via node 4 the first time it enters a link
# ending at node 2. Agent 2 is pathless, so iteration-0 shortest path
# assigns it a route. Checks: agent 1 completes on the diverted path
# with its original path preserved, its tour id kept, agent 2 gets a
# path, and a bad reroute chain fails loud with ProgramStop.
import csv
import io
import os
import shutil
import sys
import types

TRIM_DIR = os.path.join(os.path.dirname(__file__), "..", "pysim")
_pkg = types.ModuleType("pysim_trimmed")
_pkg.__path__ = [TRIM_DIR]
sys.modules["pysim_trimmed"] = _pkg
from pysim_trimmed.api import run_pysim  # noqa: E402
from pysim_trimmed.cpp import ProgramStop  # noqa: E402

BASE = "/tmp/reroute_case"


def write_inputs():
    if os.path.exists(BASE):
        shutil.rmtree(BASE)
    os.makedirs(BASE)
    files = {
        "DTASettings.txt":
            "[assignment]\naggregation_time_interval_in_min = 15\n"
            "agent_based_assignment = 0\n[simulation]\n"
            "demand_capcaity_scaling_factor = 1.0\n"
            "use_point_queue_for_on_ramps = 1\n"
            "use_point_queue_for_off_ramps = 1\n[traveler_information]\n"
            "information_updating_interval_in_min = 1\n",
        "input_scenario_settings.csv":
            "scenario_no,scenario_name,number_of_iterations,"
            "traffic_flow_model,signal_representation_model,"
            "traffic_analysis_method,random_seed\n"
            "1,reroute,1,2,0,0,100\n",
        "input_node.csv":
            "node_id,x,y\n1,0,0\n2,1,0\n3,2,0\n4,1,1\n5,3,0\n",
        "input_link_type.csv":
            "link_type,link_type_name,type_code\n1,Freeway,f\n",
        "input_link.csv":
            "from_node_id,to_node_id,link_type,direction,length,"
            "number_of_lanes,speed_limit,lane_cap,jam_density,wave_speed\n"
            "1,2,1,1,1.0,1,60,1800,180,12\n"
            "2,3,1,1,1.0,1,60,1800,180,12\n"
            "3,5,1,1,1.0,1,60,1800,180,12\n"
            "2,4,1,1,1.0,1,60,1800,180,12\n"
            "4,5,1,1,1.0,1,60,1800,180,12\n",
        "input_zone.csv":
            "zone_id\n1\n2\n3\n4\n5\n",
        "input_demand_type.csv":
            "demand_type,avg_VOT,percentage_of_pretrip_info,"
            "percentage_of_enroute_info\n1,10,0,0\n",
        "input_demand_file_list.csv":
            "file_sequence_no,file_name,format_type,start_time_in_min,"
            "end_time_in_min\n1,input_agent.csv,agent_csv,0,60\n",
        "input_agent.csv":
            "agent_id,tour_id,from_zone_id,to_zone_id,"
            "from_origin_node_id,to_destination_node_id,"
            "departure_time_in_min,demand_type,vehicle_type,"
            "information_type,vehicle_age,value_of_time,"
            "path_node_sequence\n"
            "1,7,1,5,1,5,0.0,1,1,1,5,10,1;2;3;5;\n"
            "2,-1,1,5,1,5,0.0,1,1,1,5,10,\n",
    }
    for name, text in files.items():
        with open(os.path.join(BASE, name), "w", newline="") as fh:
            fh.write(text)


def read_agent(path):
    with open(path, newline="") as fh:
        rows = [r for r in csv.reader(io.StringIO(fh.read())) if r]
    header = [h.strip() for h in rows[0]]
    out = {}
    for row in rows[1:]:
        rec = dict(zip(header, row))
        out[int(rec["agent_id"])] = rec
    return out


def main():
    write_inputs()
    diverted = set()

    def callback(veh, next_node, world):
        if veh.agent_id != 1 or next_node in diverted:
            return None
        if next_node != world.node_index[2]:
            return None
        diverted.add(next_node)
        return [world.link_map[(2, 4)], world.link_map[(4, 5)]]

    result = run_pysim(BASE, "/tmp/reroute_out", reroute_callback=callback)
    assert result["completed"] == 2, result
    agents = read_agent("/tmp/reroute_out/output_agent.csv")
    a1 = agents[1]
    assert a1["complete_flag"] == "c", a1
    assert a1["path_node_sequence"] == "1;2;4;5;", a1
    assert a1["org_path_node_sequence"] == "5;2;3;5;", a1
    assert a1["tour_id"] == "7", a1
    a2 = agents[2]
    assert a2["complete_flag"] == "c", a2
    assert a2["path_node_sequence"] in ("1;2;3;5;", "1;2;4;5;"), a2

    # No callback: agent 1 keeps its preassigned path.
    run_pysim(BASE, "/tmp/reroute_out_nocb")
    agents = read_agent("/tmp/reroute_out_nocb/output_agent.csv")
    assert agents[1]["path_node_sequence"] == "1;2;3;5;", agents[1]
    assert agents[1]["org_path_node_sequence"] == "", agents[1]

    # Bad chain: first link does not start at the current node.
    def bad_callback(veh, next_node, world):
        if veh.agent_id == 1 and next_node == world.node_index[2]:
            return [world.link_map[(4, 5)]]
        return None
    try:
        run_pysim(BASE, "/tmp/reroute_out_bad",
                  reroute_callback=bad_callback)
    except ProgramStop:
        pass
    else:
        raise AssertionError("bad reroute chain did not raise ProgramStop")
    print("REROUTE PASS: diverted path 1;2;4;5, original preserved, "
          "tour kept, bad chain fails loud")
    return 0


if __name__ == "__main__":
    sys.exit(main())
