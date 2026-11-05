# Charging accounting test for the charging-fork pysim.
#
# Tiny network: 1 -> 2 -> 3 (fast charger at 3), 2 -> 4 (no charger),
# 1 -> 5 (slow charger at 5). Links are 10 miles at 60 mph, so each
# leg takes 10 minutes free-flow and uses 10 * 1.60934 * 0.2 = 3.21868
# kWh. Agents: 1 and 2 are 40 kWh EVs (no home charger, start at 32)
# to node 3, departing 0 and 1; 3 is a 4 kWh EV (starts at 3.2) that
# strands on link 1 -> 2; 4 is a gas car; 5 is a 40 kWh EV to node 4
# (no charger); 6 is a 40 kWh EV with a home charger (starts full)
# to node 5. Checks: event energies, the queue wait when agent 2
# arrives while agent 1 still charges, the stranded record, the
# station summary, and that output_agent.csv is byte-identical with
# charging on and off (charging never interferes with traffic).
import csv
import os
import shutil
import sys
import types

TRIM_DIR = os.path.join(os.path.dirname(__file__), "..", "pysim")
_pkg = types.ModuleType("pysim_trimmed")
_pkg.__path__ = [TRIM_DIR]
sys.modules["pysim_trimmed"] = _pkg
from pysim_trimmed.api import run_pysim  # noqa: E402

BASE = "/tmp/charging_case"
BASE_NOCH = "/tmp/charging_case_noch"


def write_inputs(base, with_charger=True):
    if os.path.exists(base):
        shutil.rmtree(base)
    os.makedirs(base)
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
            "1,charging,1,2,0,0,100\n",
        "input_node.csv":
            "node_id,x,y\n1,0,0\n2,1,0\n3,2,0\n4,1,1\n5,0,1\n",
        "input_link_type.csv":
            "link_type,link_type_name,type_code\n1,Freeway,f\n",
        "input_link.csv":
            "from_node_id,to_node_id,link_type,direction,length,"
            "number_of_lanes,speed_limit,lane_cap,jam_density,wave_speed\n"
            "1,2,1,1,10.0,1,60,1800,180,12\n"
            "2,3,1,1,10.0,1,60,1800,180,12\n"
            "2,4,1,1,10.0,1,60,1800,180,12\n"
            "1,5,1,1,10.0,1,60,1800,180,12\n",
        "input_zone.csv":
            "zone_id\n1\n2\n3\n4\n5\n",
        "input_demand_type.csv":
            "demand_type,avg_VOT,percentage_of_pretrip_info,"
            "percentage_of_enroute_info\n1,10,0,0\n",
        "input_demand_file_list.csv":
            "file_sequence_no,file_name,format_type,start_time_in_min,"
            "end_time_in_min\n1,input_agent.csv,agent_csv,0,60\n",
        "optional_vehicle_type.csv":
            "vehicle_type,vehicle_type_name,PCE,battery_kwh,is_electric,"
            "consumption_kwh_per_km\n"
            "1,EV car,1,40,1,0.2\n"
            "2,EV small,1,4,1,0.2\n"
            "3,Gas car,1,0,0,0\n",
        "input_agent.csv":
            "agent_id,tour_id,from_zone_id,to_zone_id,"
            "from_origin_node_id,to_destination_node_id,"
            "departure_time_in_min,demand_type,vehicle_type,"
            "information_type,vehicle_age,value_of_time,"
            "path_node_sequence,home_charger\n"
            "1,-1,1,3,1,3,0.0,1,1,1,5,10,1;2;3;,0\n"
            "2,-1,1,3,1,3,1.0,1,1,1,5,10,1;2;3;,0\n"
            "3,-1,1,3,1,3,0.0,1,2,1,5,10,1;2;3;,0\n"
            "4,-1,1,3,1,3,0.0,1,3,1,5,10,1;2;3;,0\n"
            "5,-1,1,4,1,4,0.0,1,1,1,5,10,1;2;4;,0\n"
            "6,9,1,5,1,5,0.0,1,1,1,5,10,1;5;,1\n"
            "7,-1,1,3,1,3,0.5,1,1,1,5,10,1;2;3;,0\n",
    }
    if with_charger:
        files["input_charger.csv"] = (
            "charger_id,node_id,power_kw,plugs,kind\n"
            "FAST1,3,150,1,fast\n"
            "SLOW1,5,22,2,slow\n")
    for name, text in files.items():
        with open(os.path.join(base, name), "w", newline="") as fh:
            fh.write(text)


def read_csv(path):
    with open(path, newline="") as fh:
        rows = [r for r in csv.reader(fh) if r]
    header = [h.strip() for h in rows[0]]
    return [dict(zip(header, r)) for r in rows[1:]]


def close(actual, expected, tol=0.05):
    assert abs(float(actual) - expected) <= tol, (actual, expected)


def main():
    write_inputs(BASE)
    result = run_pysim(BASE, "/tmp/charging_out")
    assert result["completed"] == 7, result
    assert result["charging"]["charges"] == 4, result
    assert result["charging"]["stranded"] == 1, result

    events = read_csv("/tmp/charging_out/output_charging.csv")
    assert len(events) == 4, events
    by_agent = {int(e["agent_id"]): e for e in events}
    # Agent 1: 20 miles = 6.43736 kWh, fast target 32 kWh.
    e1 = by_agent[1]
    assert e1["charger_id"] == "FAST1", e1
    close(e1["arrival_time_in_min"], 20.0)
    close(e1["wait_in_min"], 0.0)
    close(e1["energy_kwh"], 6.43736)
    close(e1["soc_before_kwh"], 25.56264)
    close(e1["soc_after_kwh"], 32.0)
    # Agent 7 arrives at 20.5, while agent 1 still charges until
    # 22.575: it waits 2.075 min even though its id is higher than
    # agent 2's (FIFO is by arrival, not agent id).
    e7 = by_agent[7]
    close(e7["arrival_time_in_min"], 20.5)
    close(e7["wait_in_min"], 2.07494)
    close(e7["energy_kwh"], 6.43736)
    # Agent 2 arrives at 21.0, third in line: waits until 25.15.
    e2 = by_agent[2]
    close(e2["arrival_time_in_min"], 21.0)
    close(e2["wait_in_min"], 4.14989)
    close(e2["energy_kwh"], 6.43736)
    # Agent 6: home charger, starts full; 10 miles = 3.21868 kWh;
    # slow target is full 40 kWh.
    e6 = by_agent[6]
    assert e6["charger_id"] == "SLOW1", e6
    assert e6["tour_id"] == "9", e6
    close(e6["arrival_time_in_min"], 10.0)
    close(e6["wait_in_min"], 0.0)
    close(e6["energy_kwh"], 3.21868)
    close(e6["soc_before_kwh"], 36.78132)
    close(e6["soc_after_kwh"], 40.0)
    # Agents 4 (gas) and 5 (no charger at destination) charge nowhere.
    assert 4 not in by_agent and 5 not in by_agent, by_agent

    stranded = read_csv("/tmp/charging_out/output_stranded.csv")
    assert len(stranded) == 1, stranded
    s = stranded[0]
    assert s["agent_id"] == "3", s
    assert s["stranded_node_id"] == "2", s
    close(s["trip_energy_kwh"], 6.43736)
    close(s["soc_start_kwh"], 3.2)

    summary = read_csv("/tmp/charging_out/output_charger_summary.csv")
    by_site = {r["charger_id"]: r for r in summary}
    fast = by_site["FAST1"]
    assert fast["charges_served"] == "3", fast
    close(fast["total_energy_kwh"], 19.31208)
    close(fast["mean_wait_min"], 2.07494)
    close(fast["max_wait_min"], 4.14989)
    assert fast["max_queue_length"] == "2", fast
    slow = by_site["SLOW1"]
    assert slow["charges_served"] == "1", slow
    close(slow["total_energy_kwh"], 3.21868)
    assert slow["max_queue_length"] == "0", slow

    # Traffic untouched: agent file byte-identical with charging off.
    run_pysim(BASE, "/tmp/charging_out_off", charging=False)
    with open("/tmp/charging_out/output_agent.csv", "rb") as fh:
        on = fh.read()
    with open("/tmp/charging_out_off/output_agent.csv", "rb") as fh:
        off = fh.read()
    assert on == off, "charging changed traffic output"
    assert not os.path.exists("/tmp/charging_out_off/output_charging.csv")

    # No charger file: no charging outputs, traffic still runs.
    write_inputs(BASE_NOCH, with_charger=False)
    result = run_pysim(BASE_NOCH, "/tmp/charging_out_noch")
    assert result["charging"] is None, result
    assert result["completed"] == 7, result
    assert not os.path.exists("/tmp/charging_out_noch/output_charging.csv")

    # Config override: everyone starts full, so only agent 6 (slow
    # target 40, arriving below it) still charges.
    result = run_pysim(BASE, "/tmp/charging_out_full",
                       charging={"initial_soc_no_home": 1.0})
    assert result["charging"]["charges"] == 1, result
    print("CHARGING PASS: 4 events, FIFO queue (max 2, max wait 4.15 "
          "min), 1 stranded, traffic byte-identical, auto/off/override "
          "modes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
