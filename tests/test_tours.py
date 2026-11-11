# Multi-leg tour tests for the charging-fork pysim (slice 1b).
#
# Tiny network: nodes 1-5 with 10-mile links at 60 mph in both
# directions (1<->2, 2<->3, 2<->4, 1<->5), so every link takes 10
# minutes free-flow and uses 10 * 1.60934 * 0.2 = 3.21868 kWh for an
# EV. Checks: a late arrival cascading through a dwell leg and being
# reset by a following fixed leg; SoC persisting across legs and days
# with no home charging applied yet; an intrazonal instant leg; and
# that charging + tour input raises ProgramStop.
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
from pysim_trimmed.cpp import ProgramStop  # noqa: E402

BASE = "/tmp/tours_case"
BASE_2DAY = "/tmp/tours_case_2day"
BASE_INTRA = "/tmp/tours_case_intra"
BASE_CHG = "/tmp/tours_case_chg"

AGENT_HEADER = (
    "agent_id,tour_id,leg_index,from_zone_id,to_zone_id,"
    "from_origin_node_id,to_destination_node_id,"
    "departure_time_in_min,departure_mode,dwell_min,trip_purpose,"
    "demand_type,vehicle_type,information_type,vehicle_age,"
    "value_of_time,path_node_sequence,home_charger,archetype,"
    "enroute_target_soc\n")


def write_inputs(base, end, agent_rows, with_charger=False,
                 isolated_node=None, activity=None):
    if os.path.exists(base):
        shutil.rmtree(base)
    os.makedirs(base)
    links = (
        "1,2,1,1,10.0,1,60,1800,180,12\n"
        "2,1,1,1,10.0,1,60,1800,180,12\n"
        "2,3,1,1,10.0,1,60,1800,180,12\n"
        "3,2,1,1,10.0,1,60,1800,180,12\n"
        "2,4,1,1,10.0,1,60,1800,180,12\n"
        "4,2,1,1,10.0,1,60,1800,180,12\n"
        "1,5,1,1,10.0,1,60,1800,180,12\n"
        "5,1,1,1,10.0,1,60,1800,180,12\n")
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
            "1,tours,1,2,0,0,100\n",
        "input_node.csv":
            "node_id,x,y\n1,0,0\n2,1,0\n3,2,0\n4,1,1\n5,0,1\n",
        "input_link_type.csv":
            "link_type,link_type_name,type_code\n1,Freeway,f\n",
        "input_link.csv":
            "from_node_id,to_node_id,link_type,direction,length,"
            "number_of_lanes,speed_limit,lane_cap,jam_density,wave_speed\n"
            + links,
        "input_zone.csv":
            "zone_id\n1\n2\n3\n4\n5\n",
        "input_demand_type.csv":
            "demand_type,avg_VOT,percentage_of_pretrip_info,"
            "percentage_of_enroute_info\n1,10,0,0\n",
        "input_demand_file_list.csv":
            "file_sequence_no,file_name,format_type,start_time_in_min,"
            "end_time_in_min\n1,input_agent.csv,agent_csv,0,%d\n" % end,
        "optional_vehicle_type.csv":
            "vehicle_type,vehicle_type_name,PCE,battery_kwh,is_electric,"
            "consumption_kwh_per_km\n"
            "1,EV car,1,40,1,0.2\n"
            "3,Gas car,1,0,0,0\n",
        "input_agent.csv": AGENT_HEADER + "".join(agent_rows),
    }
    if with_charger:
        files["input_charger.csv"] = (
            "charger_id,node_id,power_kw,plugs,kind\n"
            "FAST1,3,150,1,fast\n")
    if isolated_node is not None:
        # A node with no links at all, in a zone of its own: no path
        # can ever reach it.
        files["input_node.csv"] += "%d,5,5\n" % isolated_node
        files["input_zone.csv"] += "%d\n" % isolated_node
    if activity is not None:
        files["input_activity_location.csv"] = (
            "zone_id,node_id,external_OD_flag\n" + activity)
    for name, text in files.items():
        with open(os.path.join(base, name), "w", newline="") as fh:
            fh.write(text)


def leg_row(agent_id, leg_index, from_zone, to_zone, from_node, to_node,
            sched, mode, dwell, purpose, vtype=3, path="", home=0):
    return (
        "%d,-1,%d,%d,%d,%d,%d,%.1f,%s,%.1f,%s,1,%d,1,5,10,%s,%d,,\n"
        % (agent_id, leg_index, from_zone, to_zone, from_node, to_node,
           sched, mode, dwell, purpose, vtype, path, home))


def read_leg_rows(path):
    # Leg columns are the last 13 fields of every row (full and short
    # rows differ in length, so positional-from-the-tail parsing is
    # the only layout that works for both).
    with open(path, newline="") as fh:
        rows = [r for r in csv.reader(fh) if r]
    header = [h.strip() for h in rows[0]]
    assert header[-13:] == [
        "leg_index", "departure_mode", "trip_purpose", "day",
        "scheduled_departure_in_min", "actual_departure_in_min",
        "scheduled_arrival_in_min", "lateness_in_min", "soc_start_kwh",
        "soc_end_kwh", "stranded_flag", "tow_flag",
        "charged_en_route"], header[-13:]
    out = []
    for r in rows[1:]:
        tail = r[-13:]
        out.append({
            "agent_id": int(r[0]),
            "departure": float(r[9]),
            "arrival": float(r[10]),
            "complete": r[11],
            "travel": float(r[12]),
            "node_size": int(r[20]) if len(r) == 41 else int(r[18]),
            "leg_index": int(tail[0]),
            "departure_mode": tail[1],
            "purpose": tail[2],
            "day": int(tail[3]),
            "scheduled": float(tail[4]),
            "actual_dep": float(tail[5]),
            "scheduled_arr": float(tail[6]),
            "lateness": float(tail[7]),
            "soc_start": float(tail[8]),
            "soc_end": float(tail[9]),
            "stranded": int(tail[10]),
            "tow": int(tail[11]),
            "charged": int(tail[12]),
        })
    return out


def close(actual, expected, tol=0.05):
    assert abs(float(actual) - expected) <= tol, (actual, expected)


def test_cascade():
    # 40 background agents on 1->3 at t=0 queue the tour agent's
    # first leg late; the dwell leg inherits the delay, the fixed
    # leg resets it.
    rows = []
    for i in range(100, 140):
        rows.append(leg_row(i, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work",
                            path="1;2;3;"))
    rows.append(leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work",
                        path="1;2;3;"))
    rows.append(leg_row(1, 1, 3, 1, 3, 1, 50.0, "dwell", 30, "home",
                        path="3;2;1;"))
    rows.append(leg_row(1, 2, 1, 5, 1, 5, 120.0, "fixed", 0, "other",
                        path="1;5;"))
    rows.append(leg_row(1, 3, 5, 1, 5, 1, 140.0, "dwell", 10, "home",
                        path="5;1;"))
    write_inputs(BASE, 300, rows)
    result = run_pysim(BASE, "/tmp/tours_out")
    assert result["agents"] == 41, result
    assert result["completed"] == 41, result
    legs = [r for r in read_leg_rows("/tmp/tours_out/output_agent.csv")
            if r["agent_id"] == 1]
    assert len(legs) == 4, legs
    assert [r["leg_index"] for r in legs] == [0, 1, 2, 3], legs
    assert all(r["complete"] == "c" for r in legs), legs
    leg0, leg1, leg2, leg3 = legs
    # Leg 0 free-flow would arrive at 20.0; the queue pushes it late.
    assert leg0["arrival"] > 20.5, leg0
    close(leg0["travel"], leg0["arrival"] - 0.0, 0.2)
    # Dwell leg: departs arrival + 30, so the lateness cascades.
    close(leg1["actual_dep"], leg0["arrival"] + 30.0, 0.2)
    close(leg1["lateness"], leg1["actual_dep"] - 50.0, 0.05)
    assert leg1["lateness"] > 0.5, leg1
    close(leg1["travel"], 20.0, 0.2)
    # Fixed leg: scheduled 120 is after the dwell leg's arrival, so
    # it departs on time and the lateness resets to ~0.
    close(leg2["actual_dep"], 120.0, 0.15)
    close(leg2["lateness"], 0.0, 0.15)
    close(leg2["arrival"], 130.0, 0.15)
    # Following dwell leg is back on its nominal schedule.
    close(leg3["actual_dep"], 140.0, 0.15)
    close(leg3["lateness"], 0.0, 0.15)
    close(leg3["arrival"], 150.0, 0.15)
    print("cascade: leg0 arrival %.2f, dwell-leg lateness %.2f, "
          "fixed-leg lateness %.2f"
          % (leg0["arrival"], leg1["lateness"], leg2["lateness"]))


def test_two_day_soc():
    # EV, no home charger: starts at 32 kWh. Each 20-mile leg uses
    # 6.43736 kWh. Overnight dwell of 960 min, no charging applied.
    rows = [
        leg_row(1, 0, 1, 3, 1, 3, 480.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=0),
        leg_row(1, 1, 3, 1, 3, 1, 1460.0, "dwell", 960, "home", vtype=1,
                path="3;2;1;", home=0),
        leg_row(1, 2, 1, 3, 1, 3, 1500.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=0),
    ]
    write_inputs(BASE_2DAY, 3000, rows)
    result = run_pysim(BASE_2DAY, "/tmp/tours_out_2day")
    assert result["agents"] == 1 and result["completed"] == 1, result
    legs = read_leg_rows("/tmp/tours_out_2day/output_agent.csv")
    assert len(legs) == 3, legs
    leg0, leg1, leg2 = legs
    close(leg0["arrival"], 500.0)
    close(leg0["soc_start"], 32.0)
    close(leg0["soc_end"], 25.56264)
    assert leg0["day"] == 0, leg0
    # Day 2: departs 1460 (day 1), starts with exactly what leg 0
    # ended with -- no overnight top-up in slice 1.
    close(leg1["actual_dep"], 1460.0)
    assert leg1["day"] == 1, leg1
    close(leg1["soc_start"], leg0["soc_end"])
    close(leg1["soc_end"], 19.12528)
    close(leg2["actual_dep"], 1500.0)
    close(leg2["soc_start"], leg1["soc_end"])
    close(leg2["soc_end"], 12.68792)
    assert leg2["day"] == 1, leg2
    print("two-day SoC: leg0 end %.3f, leg1 start %.3f, leg2 end %.3f"
          % (leg0["soc_end"], leg1["soc_start"], leg2["soc_end"]))


def test_intrazonal_and_charging_guard():
    rows = [
        leg_row(2, 0, 1, 2, 1, 2, 0.0, "fixed", 0, "work",
                path="1;2;"),
        leg_row(2, 1, 2, 2, 2, 2, 15.0, "dwell", 5, "other", path=""),
        leg_row(2, 2, 2, 3, 2, 3, 60.0, "fixed", 0, "work",
                path="2;3;"),
    ]
    write_inputs(BASE_INTRA, 300, rows)
    result = run_pysim(BASE_INTRA, "/tmp/tours_out_intra")
    assert result["agents"] == 1 and result["completed"] == 1, result
    legs = read_leg_rows("/tmp/tours_out_intra/output_agent.csv")
    assert len(legs) == 3, legs
    leg0, leg1, leg2 = legs
    close(leg0["arrival"], 10.0)
    # Instant leg: departs and arrives at its scheduled 15.0, zero
    # travel, tour proceeds to the fixed leg.
    assert leg1["node_size"] == 0, leg1
    assert leg1["complete"] == "c", leg1
    close(leg1["actual_dep"], 15.0)
    close(leg1["arrival"], 15.0)
    close(leg1["travel"], 0.0)
    close(leg1["lateness"], 0.0)
    close(leg2["actual_dep"], 60.0)
    close(leg2["arrival"], 70.0)
    # Same tour with a charger file: slice 2 runs the inline
    # charging engine instead of the old post-pass guard (the
    # guard was removed when inline charging arrived). These are
    # gas cars, so no charges occur, but the decisions file is
    # written and the tour completes.
    write_inputs(BASE_CHG, 300, rows, with_charger=True)
    result = run_pysim(BASE_CHG, "/tmp/tours_out_chg")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 0, result
    assert os.path.exists("/tmp/tours_out_chg/output_decisions.csv")
    print("intrazonal: instant leg 15.0->15.0, tour completed; "
          "charging+tour runs the inline engine (decisions written)")


def test_pathless_no_path_raises():
    # Cross-zone OD to a node with no links at all: iteration 0
    # cannot find a path, so the run fails loud naming the leg.
    rows = [
        leg_row(9, 0, 1, 6, 1, 6, 0.0, "fixed", 0, "work", path=""),
    ]
    write_inputs("/tmp/tours_case_nopath", 300, rows, isolated_node=6)
    try:
        run_pysim("/tmp/tours_case_nopath", "/tmp/tours_out_nopath")
    except ProgramStop as exc:
        assert "agent 9" in str(exc) and "leg 0" in str(exc), exc
        assert "node 1" in str(exc) and "node 6" in str(exc), exc
    else:
        raise AssertionError("pathless leg with no route did not raise")
    print("no-path: agent 9 leg 0 1->6 raises ProgramStop")


# Activity locations putting nodes 1, 2 and 5 in zone 1, so a leg
# can be same-zone between distinct nodes.
SAME_ZONE_ACTIVITY = (
    "1,1,0\n1,2,0\n2,3,0\n3,4,0\n1,5,0\n")


def test_same_zone_distinct_node_preassigned():
    # Same-zone distinct-node leg WITH a preassigned path simulates
    # normally: 10 minutes of travel, not an instant completion.
    rows = [
        leg_row(3, 0, 1, 1, 1, 2, 0.0, "fixed", 0, "work",
                path="1;2;"),
        leg_row(3, 1, 1, 2, 2, 3, 20.0, "dwell", 5, "home",
                path="2;3;"),
    ]
    write_inputs("/tmp/tours_case_samezone", 300, rows,
                 activity=SAME_ZONE_ACTIVITY)
    result = run_pysim("/tmp/tours_case_samezone", "/tmp/tours_out_samezone")
    assert result["agents"] == 1 and result["completed"] == 1, result
    legs = read_leg_rows("/tmp/tours_out_samezone/output_agent.csv")
    assert len(legs) == 2, legs
    leg0, leg1 = legs
    assert leg0["complete"] == "c" and leg1["complete"] == "c", legs
    close(leg0["arrival"], 10.0)
    close(leg0["travel"], 10.0)
    close(leg1["actual_dep"], 15.0)
    close(leg1["arrival"], 25.0)
    print("same-zone preassigned: leg 1->2 travelled 10.0 min, "
          "tour completed")


def test_same_zone_distinct_node_pathless():
    # Same-zone distinct-node leg with NO path gets a free-flow
    # shortest path at iteration 0 and simulates (does not teleport).
    rows = [
        leg_row(3, 0, 1, 1, 1, 2, 0.0, "fixed", 0, "work", path=""),
        leg_row(3, 1, 1, 2, 2, 3, 20.0, "dwell", 5, "home",
                path="2;3;"),
    ]
    write_inputs("/tmp/tours_case_samezone2", 300, rows,
                 activity=SAME_ZONE_ACTIVITY)
    result = run_pysim("/tmp/tours_case_samezone2",
                       "/tmp/tours_out_samezone2")
    assert result["agents"] == 1 and result["completed"] == 1, result
    legs = read_leg_rows("/tmp/tours_out_samezone2/output_agent.csv")
    assert len(legs) == 2, legs
    leg0, leg1 = legs
    assert leg0["node_size"] == 2, leg0
    assert leg0["complete"] == "c" and leg1["complete"] == "c", legs
    close(leg0["arrival"], 10.0)
    close(leg0["travel"], 10.0)
    close(leg1["actual_dep"], 15.0)
    close(leg1["arrival"], 25.0)
    print("same-zone pathless: leg 1->2 assigned a path, travelled "
          "10.0 min, tour completed")


def test_trailing_instant_leg():
    # A mixed tour whose LAST leg is instant: the tour still
    # completes fully, with every leg row present and timed.
    # Reorder so the instant leg trails: real, real, instant.
    rows = [
        leg_row(4, 0, 1, 2, 1, 2, 0.0, "fixed", 0, "work",
                path="1;2;"),
        leg_row(4, 1, 2, 3, 2, 3, 20.0, "dwell", 5, "work",
                path="2;3;"),
        leg_row(4, 2, 3, 3, 3, 3, 40.0, "dwell", 5, "home", path=""),
    ]
    write_inputs("/tmp/tours_case_trailing", 300, rows)
    result = run_pysim("/tmp/tours_case_trailing", "/tmp/tours_out_trailing")
    assert result["agents"] == 1 and result["completed"] == 1, result
    legs = read_leg_rows("/tmp/tours_out_trailing/output_agent.csv")
    assert len(legs) == 3, legs
    leg0, leg1, leg2 = legs
    assert [r["leg_index"] for r in legs] == [0, 1, 2], legs
    assert all(r["complete"] == "c" for r in legs), legs
    close(leg0["arrival"], 10.0)
    close(leg1["actual_dep"], 15.0)
    close(leg1["arrival"], 25.0)
    # Instant leg trails: departs and arrives at its scheduled 30.0
    # (arrival 25.0 + dwell 5), zero travel.
    assert leg2["node_size"] == 0, leg2
    close(leg2["actual_dep"], 30.0)
    close(leg2["arrival"], 30.0)
    close(leg2["travel"], 0.0)
    print("trailing instant: real legs 10.0/25.0, instant leg "
          "30.0->30.0, tour completed")


def test_departure_at_zero():
    # A leg scheduled at 0.05 falls in departure bucket 0, so it is
    # generated at step time 0.0: actual departure 0.00, lateness
    # -0.05 -- not the old sentinel reading of actual 0.05 / 0.00.
    row = (
        "%d,-1,%d,%d,%d,%d,%d,%.2f,%s,%.1f,%s,1,%d,1,5,10,%s,%d,,\n"
        % (5, 0, 1, 3, 1, 3, 0.05, "fixed", 0, "work", 3, "1;2;3;", 0))
    write_inputs("/tmp/tours_case_dep0", 300, [row])
    result = run_pysim("/tmp/tours_case_dep0", "/tmp/tours_out_dep0")
    assert result["agents"] == 1 and result["completed"] == 1, result
    legs = read_leg_rows("/tmp/tours_out_dep0/output_agent.csv")
    assert len(legs) == 1, legs
    leg0 = legs[0]
    assert leg0["complete"] == "c", leg0
    close(leg0["actual_dep"], 0.0, 0.01)
    close(leg0["lateness"], -0.05, 0.01)
    # Arrival is 20.1, not 20.0: the first-link stamp is built from
    # the scheduled departure (0.05), so its 10.05 free-flow stamp
    # fires at the next step boundary, 10.1.
    close(leg0["arrival"], 20.1)
    print("departure-at-zero: scheduled 0.05, actual 0.00, "
          "lateness -0.05")


def test_mixed_mode_demand_files():
    # Two agent files in the demand list, one tour and one legacy:
    # the run fails loud instead of loading tour legs the legacy
    # writer would silently drop.
    rows = [
        leg_row(6, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work",
                path="1;2;3;"),
    ]
    base = "/tmp/tours_case_mixed"
    write_inputs(base, 300, rows)
    legacy_header = (
        "agent_id,tour_id,from_zone_id,to_zone_id,"
        "from_origin_node_id,to_destination_node_id,"
        "departure_time_in_min,demand_type,vehicle_type,"
        "information_type,vehicle_age,value_of_time,"
        "path_node_sequence,home_charger\n")
    legacy_row = "7,-1,1,3,1,3,0.0,1,3,1,5,10,1;2;3;,0\n"
    with open(os.path.join(base, "input_agent_legacy.csv"), "w",
              newline="") as fh:
        fh.write(legacy_header + legacy_row)
    with open(os.path.join(base, "input_demand_file_list.csv"), "w",
              newline="") as fh:
        fh.write(
            "file_sequence_no,file_name,format_type,start_time_in_min,"
            "end_time_in_min\n"
            "1,input_agent.csv,agent_csv,0,300\n"
            "2,input_agent_legacy.csv,agent_csv,0,300\n")
    try:
        run_pysim(base, "/tmp/tours_out_mixed")
    except ProgramStop as exc:
        assert "mix tour and legacy" in str(exc), exc
    else:
        raise AssertionError("mixed tour/legacy demand did not raise")
    print("mixed-mode: tour + legacy agent files raise ProgramStop")


def main():
    test_cascade()
    test_two_day_soc()
    test_intrazonal_and_charging_guard()
    test_pathless_no_path_raises()
    test_same_zone_distinct_node_preassigned()
    test_same_zone_distinct_node_pathless()
    test_trailing_instant_leg()
    test_departure_at_zero()
    test_mixed_mode_demand_files()
    print("TOURS PASS: cascade/dwell-reset, 2-day SoC persistence, "
          "intrazonal instant leg, charging+tour inline engine, "
          "no-path raise, "
          "same-zone legs, trailing instant leg, departure-at-zero, "
          "mixed-mode guard")
    return 0


if __name__ == "__main__":
    sys.exit(main())
