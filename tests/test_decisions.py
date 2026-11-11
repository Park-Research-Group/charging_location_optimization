# Pre-trip decision + inline dwell charging tests (slice 2).
#
# Tiny network: nodes 1-5 with 10-mile links at 60 mph in both
# directions (1<->2, 2<->3, 2<->4, 1<->5), so every link takes 10
# minutes free-flow and uses 10 * 1.60934 * 0.2 = 3.21868 kWh for
# an EV, and a 1->3 (or 3->1) leg uses 6.43736 kWh. Checks: a
# destination charge cut short by a short dwell (partial charge,
# next leg on time); home charging to full overnight (HOME event
# and HOME summary row); the three decision branches (none /
# destination / en-route, the last now executed: the leg detours,
# parks at the charger and completes); a queued-out destination charge when the plug
# is busy past the dwell window; and legacy preservation
# (test_charging.py unchanged, charging off / no charger file on
# tour inputs writes no charging files).
import csv
import os
import shutil
import subprocess
import sys
import types

TRIM_DIR = os.path.join(os.path.dirname(__file__), "..", "pysim")
_pkg = types.ModuleType("pysim_trimmed")
_pkg.__path__ = [TRIM_DIR]
sys.modules["pysim_trimmed"] = _pkg
from pysim_trimmed.api import run_pysim  # noqa: E402

BASE_PARTIAL = "/tmp/decisions_case_partial"
BASE_HOME = "/tmp/decisions_case_home"
BASE_NONE = "/tmp/decisions_case_none"
BASE_DEST = "/tmp/decisions_case_dest"
BASE_ENROUTE = "/tmp/decisions_case_enroute"
BASE_QUEUED = "/tmp/decisions_case_queued"
BASE_OFF = "/tmp/decisions_case_off"
BASE_NOCH = "/tmp/decisions_case_noch"
BASE_UNREACH = "/tmp/decisions_case_unreach"
BASE_ATTARGET = "/tmp/decisions_case_attarget"
BASE_ATTARGET_BUSY = "/tmp/decisions_case_attarget_busy"

AGENT_HEADER = (
    "agent_id,tour_id,leg_index,from_zone_id,to_zone_id,"
    "from_origin_node_id,to_destination_node_id,"
    "departure_time_in_min,departure_mode,dwell_min,trip_purpose,"
    "demand_type,vehicle_type,information_type,vehicle_age,"
    "value_of_time,path_node_sequence,home_charger,archetype,"
    "enroute_target_soc\n")


def write_inputs(base, end, agent_rows, chargers=None):
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
            "1,decisions,1,2,0,0,100\n",
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
            "2,EV small,1,20,1,0.2\n"
            "3,Gas car,1,0,0,0\n"
            "4,EV 21kWh,1,21,1,0.2\n",
        "input_agent.csv": AGENT_HEADER + "".join(agent_rows),
    }
    if chargers is not None:
        files["input_charger.csv"] = (
            "charger_id,node_id,power_kw,plugs,kind\n" + chargers)
    for name, text in files.items():
        with open(os.path.join(base, name), "w", newline="") as fh:
            fh.write(text)


def leg_row(agent_id, leg_index, from_zone, to_zone, from_node, to_node,
            sched, mode, dwell, purpose, vtype=3, path="", home=0,
            target=0.8):
    return (
        "%d,-1,%d,%d,%d,%d,%d,%.1f,%s,%.1f,%s,1,%d,1,5,10,%s,%d,,%.1f\n"
        % (agent_id, leg_index, from_zone, to_zone, from_node, to_node,
           sched, mode, dwell, purpose, vtype, path, home, target))


def read_csv(path):
    with open(path, newline="") as fh:
        rows = [r for r in csv.reader(fh) if r]
    header = [h.strip() for h in rows[0]]
    return [dict(zip(header, r)) for r in rows[1:]]


def read_leg_rows(path):
    with open(path, newline="") as fh:
        rows = [r for r in csv.reader(fh) if r]
    out = []
    for r in rows[1:]:
        tail = r[-13:]
        out.append({
            "agent_id": int(r[0]),
            "leg_index": int(tail[0]),
            "actual_dep": float(tail[5]),
            "lateness": float(tail[7]),
            "soc_start": float(tail[8]),
            "soc_end": float(tail[9]),
            "stranded": int(tail[10]),
        })
    return out


def close(actual, expected, tol=0.05):
    assert abs(float(actual) - expected) <= tol, (actual, expected)


def test_partial_charge():
    # 20 kWh EV, starts at 16. Leg 0 ends at the fast charger with
    # a 1-minute dwell before leg 1: target 16 kWh needs 2.575 min
    # at 150 kW, so only 1 min x 150 kW / 60 = 2.5 kWh is delivered.
    rows = [
        leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
        leg_row(1, 1, 3, 1, 3, 1, 21.0, "dwell", 1, "other", vtype=2,
                path="3;2;1;"),
        leg_row(1, 2, 1, 3, 1, 3, 60.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
    ]
    write_inputs(BASE_PARTIAL, 300, rows,
                 chargers="FAST1,3,150,1,fast\n")
    result = run_pysim(BASE_PARTIAL, "/tmp/decisions_out_partial")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 1, result
    assert "decisions" in result["outputs"], result
    events = read_csv("/tmp/decisions_out_partial/output_charging.csv")
    assert len(events) == 1, events
    e = events[0]
    assert e["charger_id"] == "FAST1" and e["leg_index"] == "0", e
    assert e["charged_en_route"] == "0", e
    close(e["arrival_time_in_min"], 20.0)
    close(e["charge_start_in_min"], 20.0)
    close(e["charge_end_in_min"], 21.0)
    close(e["energy_kwh"], 2.5)
    close(e["soc_before_kwh"], 9.56264)
    close(e["soc_after_kwh"], 12.06264)
    legs = read_leg_rows("/tmp/decisions_out_partial/output_agent.csv")
    leg0, leg1, leg2 = legs
    close(leg0["soc_end"], 12.06264)
    # The dwell leg still departs on time: charging never delays it.
    close(leg1["actual_dep"], 21.0)
    close(leg1["lateness"], 0.0)
    close(leg1["soc_start"], 12.06264)
    decisions = read_csv(
        "/tmp/decisions_out_partial/output_decisions.csv")
    assert len(decisions) == 3, decisions
    d0, d1, d2 = decisions
    assert d0["decision"] == "destination", d0
    close(d0["target_soc"], 0.8)
    close(d0["leg_energy_kwh"], 6.43736)
    close(d0["remaining_energy_kwh"], 19.31208)
    assert d0["destination_charger"] == "1", d0
    assert d0["outcome"] == "charged-partial", d0
    assert d1["decision"] == "none" and d1["outcome"] == "completed", d1
    # Leg 2 strands (5.625 kWh left for a 6.437 kWh leg) and its
    # decision is "none": it cannot reach the destination charger,
    # so no destination decision is made. Stranded wins.
    assert d2["decision"] == "none", d2
    assert d2["outcome"] == "stranded", d2
    assert leg2["stranded"] == 1, leg2
    stranded = read_csv(
        "/tmp/decisions_out_partial/output_stranded.csv")
    assert len(stranded) == 1, stranded
    assert stranded[0]["agent_id"] == "1", stranded[0]
    assert stranded[0]["stranded_node_id"] == "3", stranded[0]
    assert stranded[0]["leg_index"] == "2", stranded[0]
    close(stranded[0]["trip_energy_kwh"], 6.43736)
    close(stranded[0]["soc_start_kwh"], 5.62528)
    print("partial: 2.5 kWh in a 1-min dwell, leg1 lateness 0.00, "
          "leg2 stranded at node 3")


def test_unreachable_destination():
    # 21 kWh EV, starts at 16.8. Two legs (1->4, 4->1) deplete it
    # to 3.925 kWh, so leg 2 (1->3, 6.437 kWh) cannot be completed;
    # the fast charger at its destination is unreachable and no
    # en-route site is viable (the only site sits at the
    # destination node, so the en-route search excludes it).
    # Decision is "none", leg strands at node 3. (The first two
    # legs end at nodes with no charger, so no dwell charge
    # replenishes the battery on the way down.)
    rows = [
        leg_row(1, 0, 1, 4, 1, 4, 0.0, "fixed", 0, "work", vtype=4,
                path="1;2;4;"),
        leg_row(1, 1, 4, 1, 4, 1, 25.0, "dwell", 5, "other", vtype=4,
                path="4;2;1;"),
        leg_row(1, 2, 1, 3, 1, 3, 60.0, "fixed", 0, "work", vtype=4,
                path="1;2;3;"),
    ]
    write_inputs(BASE_UNREACH, 300, rows,
                 chargers="FAST1,3,150,1,fast\n")
    run_pysim(BASE_UNREACH, "/tmp/decisions_out_unreach")
    decisions = read_csv(
        "/tmp/decisions_out_unreach/output_decisions.csv")
    assert len(decisions) == 3, decisions
    d2 = decisions[2]
    assert d2["decision"] == "none", d2
    assert d2["outcome"] == "stranded", d2
    close(d2["soc_at_departure_kwh"], 3.92528)
    close(d2["leg_energy_kwh"], 6.43736)
    stranded = read_csv(
        "/tmp/decisions_out_unreach/output_stranded.csv")
    assert len(stranded) == 1, stranded
    assert stranded[0]["stranded_node_id"] == "3", stranded[0]
    assert stranded[0]["leg_index"] == "2", stranded[0]
    close(stranded[0]["soc_start_kwh"], 3.92528)
    print("unreachable destination: low SoC, dest charger present, "
          "no viable en-route site -> decision none, leg strands")


def test_home_charging():
    # 20 kWh EV with a home charger, starts full. Leg 0 tops up at
    # the fast charger; leg 1 ends at home with a 960-minute dwell
    # and charges to full at 11 kW before the next day's leg.
    rows = [
        leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;", home=1),
        leg_row(1, 1, 3, 1, 3, 1, 980.0, "dwell", 960, "home", vtype=2,
                path="3;2;1;", home=1),
        leg_row(1, 2, 1, 3, 1, 3, 1440.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;", home=1),
        leg_row(1, 3, 3, 1, 3, 1, 1465.0, "dwell", 5, "other", vtype=2,
                path="3;2;1;", home=1),
        leg_row(1, 4, 1, 3, 1, 3, 1500.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;", home=1),
    ]
    write_inputs(BASE_HOME, 3000, rows,
                 chargers="FAST1,3,150,1,fast\n")
    result = run_pysim(BASE_HOME, "/tmp/decisions_out_home")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 2, result
    events = read_csv("/tmp/decisions_out_home/output_charging.csv")
    by_charger = {e["charger_id"]: e for e in events}
    home = by_charger["HOME"]
    assert home["leg_index"] == "1", home
    close(home["arrival_time_in_min"], 1000.0)
    close(home["energy_kwh"], 10.43736)
    close(home["soc_before_kwh"], 9.56264)
    close(home["soc_after_kwh"], 20.0)
    close(home["charger_power_kw"], 11.0)
    fast = by_charger["FAST1"]
    # Leg 0 starts full (20, home charger) and ends at 13.563, so
    # it needs only 2.437 kWh to reach the 16 kWh fast target.
    close(fast["energy_kwh"], 2.43736)
    close(fast["soc_after_kwh"], 16.0)
    summary = read_csv(
        "/tmp/decisions_out_home/output_charger_summary.csv")
    by_site = {r["charger_id"]: r for r in summary}
    home_row = by_site["HOME"]
    assert home_row["charges_served"] == "1", home_row
    assert home_row["plugs"] == "1", home_row
    close(home_row["total_energy_kwh"], 10.43736)
    assert home_row["mean_wait_min"] == "0.00", home_row
    assert home_row["max_queue_length"] == "0", home_row
    assert home_row["utilization"] == "0.0000", home_row
    decisions = read_csv("/tmp/decisions_out_home/output_decisions.csv")
    assert len(decisions) == 5, decisions
    assert decisions[1]["decision"] == "destination", decisions[1]
    close(decisions[1]["target_soc"], 1.0)
    assert decisions[1]["outcome"] == "charged-destination", decisions[1]
    assert decisions[0]["outcome"] == "charged-destination", decisions[0]
    legs = read_leg_rows("/tmp/decisions_out_home/output_agent.csv")
    close(legs[2]["soc_start"], 20.0)
    print("home: HOME event 10.437 kWh to full 20.0, HOME summary "
          "row plugs 1, leg2 starts full")


def test_branches():
    # (a) none: 40 kWh EV, ample SoC, destination node 4 has no
    # charger and no en-route need.
    rows = [leg_row(1, 0, 1, 4, 1, 4, 0.0, "fixed", 0, "work", vtype=1,
                    path="1;2;4;")]
    write_inputs(BASE_NONE, 300, rows,
                 chargers="FAST1,3,150,1,fast\nSLOW1,5,22,2,slow\n")
    run_pysim(BASE_NONE, "/tmp/decisions_out_none")
    decisions = read_csv("/tmp/decisions_out_none/output_decisions.csv")
    assert len(decisions) == 1, decisions
    d = decisions[0]
    assert d["decision"] == "none", d
    assert d["destination_charger"] == "0", d
    assert d["enroute_charger_id"] == "", d
    close(d["target_soc"], 0.0)
    close(d["soc_at_departure_kwh"], 32.0)
    assert d["outcome"] == "completed", d
    # (b) destination: SoC after this leg is below the remaining
    # tour energy and the destination has a fast charger.
    rows = [
        leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
        leg_row(1, 1, 3, 1, 3, 1, 50.0, "dwell", 30, "other", vtype=2,
                path="3;2;1;"),
        leg_row(1, 2, 1, 3, 1, 3, 120.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
    ]
    write_inputs(BASE_DEST, 300, rows,
                 chargers="FAST1,3,150,1,fast\n")
    run_pysim(BASE_DEST, "/tmp/decisions_out_dest")
    decisions = read_csv("/tmp/decisions_out_dest/output_decisions.csv")
    assert decisions[0]["decision"] == "destination", decisions[0]
    close(decisions[0]["target_soc"], 0.8)
    assert decisions[0]["destination_charger"] == "1", decisions[0]
    # (c) en-route: 21 kWh EV depleted to 3.926 kWh by two legs, so
    # the third leg (6.437 kWh + 2.1 margin) needs a stop. The
    # charger at node 2 is reachable (3.219 kWh each way, 20 min
    # total) and beats the node-5 charger (40 min total). Slice 3a
    # executes the decision: the via path 1->2->3 is the direct
    # path here (node 2 is on it), the vehicle parks at node 2
    # and charges to the 16.8 kWh target, then completes the leg
    # instead of stranding. (Slice 2 recorded the decision but
    # did not execute it, so the leg stranded; that expectation
    # is superseded.)
    rows = [
        leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work", vtype=4,
                path="1;2;3;"),
        leg_row(1, 1, 3, 1, 3, 1, 25.0, "dwell", 5, "other", vtype=4,
                path="3;2;1;"),
        leg_row(1, 2, 1, 3, 1, 3, 60.0, "fixed", 0, "work", vtype=4,
                path="1;2;3;"),
    ]
    write_inputs(BASE_ENROUTE, 300, rows,
                 chargers="MID1,2,50,1,mid\nSLOW1,5,22,2,slow\n")
    run_pysim(BASE_ENROUTE, "/tmp/decisions_out_enroute")
    decisions = read_csv(
        "/tmp/decisions_out_enroute/output_decisions.csv")
    d0, d1, d2 = decisions
    assert d0["decision"] == "none", d0
    assert d1["decision"] == "none", d1
    assert d2["decision"] == "enroute", d2
    assert d2["enroute_charger_id"] == "MID1", d2
    close(d2["target_soc"], 0.8)
    close(d2["soc_at_departure_kwh"], 3.92528)
    assert d2["outcome"] == "completed", d2
    events = read_csv(
        "/tmp/decisions_out_enroute/output_charging.csv")
    assert len(events) == 1, events
    e = events[0]
    assert e["charger_id"] == "MID1" and e["leg_index"] == "2", e
    assert e["charged_en_route"] == "1", e
    close(e["arrival_time_in_min"], 70.0)
    close(e["charge_end_in_min"], 89.31)
    close(e["energy_kwh"], 16.0934)
    close(e["soc_before_kwh"], 0.7066)
    close(e["soc_after_kwh"], 16.8)
    legs = read_leg_rows("/tmp/decisions_out_enroute/output_agent.csv")
    close(legs[2]["soc_end"], 13.58132)
    assert legs[2]["stranded"] == 0, legs[2]
    agent_rows = read_csv("/tmp/decisions_out_enroute/output_agent.csv")
    leg2_row = [r for r in agent_rows
                if r["agent_id"] == "1" and r["leg_index"] == "2"][0]
    assert leg2_row["charged_en_route"] == "1", leg2_row
    stranded = read_csv(
        "/tmp/decisions_out_enroute/output_stranded.csv")
    assert len(stranded) == 0, stranded
    print("branches: none / destination / enroute(MID1, charged "
          "16.093 kWh at node 2, leg completed)")


def test_queued_out():
    # Two 20 kWh EVs share the single fast plug. Agent 1 charges
    # 20-21 (its 1-minute dwell); agent 2 arrives at 21.0 with a
    # zero-minute dwell, so the plug is not free before departure:
    # no event, outcome queued-out. Agent 2's leg 2 then strands
    # mid-leg (1->2, T = 70.0) with only the destination charger
    # FAST1 available, so it is towed there (truck 100.0, charge
    # 110.0-116.40, 16 kWh) and completes; agent 1's leg 2 strands
    # on its last link at the destination and is not towed.
    rows = [
        leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
        leg_row(1, 1, 3, 1, 3, 1, 21.0, "dwell", 1, "other", vtype=2,
                path="3;2;1;"),
        leg_row(1, 2, 1, 3, 1, 3, 60.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
        leg_row(2, 0, 1, 3, 1, 3, 1.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
        leg_row(2, 1, 3, 1, 3, 1, 21.0, "dwell", 0, "other", vtype=2,
                path="3;2;1;"),
        leg_row(2, 2, 1, 3, 1, 3, 60.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
    ]
    write_inputs(BASE_QUEUED, 300, rows,
                 chargers="FAST1,3,150,1,fast\n")
    result = run_pysim(BASE_QUEUED, "/tmp/decisions_out_queued")
    assert result["charging"]["charges"] == 2, result
    assert result["charging"]["stranded"] == 2, result
    events = read_csv("/tmp/decisions_out_queued/output_charging.csv")
    assert [int(e["agent_id"]) for e in events] == [1, 2], events
    assert events[1]["charger_id"] == "FAST1", events[1]
    assert events[1]["leg_index"] == "2", events[1]
    assert events[1]["tow_charge"] == "1", events[1]
    close(events[1]["arrival_time_in_min"], 110.0)
    close(events[1]["charge_end_in_min"], 116.40)
    close(events[1]["energy_kwh"], 16.0)
    close(events[1]["soc_after_kwh"], 16.0)
    stranded = read_csv(
        "/tmp/decisions_out_queued/output_stranded.csv")
    assert len(stranded) == 2, stranded
    tow_row = [s for s in stranded
               if s["agent_id"] == "2"][0]
    assert tow_row["stranded_node_id"] == "2", tow_row
    assert tow_row["tow_charger_id"] == "FAST1", tow_row
    close(tow_row["tow_start_in_min"], 100.0)
    close(tow_row["tow_end_in_min"], 116.40)
    close(tow_row["charge_kwh"], 16.0)
    assert tow_row["tow_blockage_link"] == "1->2", tow_row
    decisions = read_csv(
        "/tmp/decisions_out_queued/output_decisions.csv")
    by_key = {(int(d["agent_id"]), int(d["leg_index"])): d
              for d in decisions}
    assert by_key[(2, 0)]["decision"] == "destination", by_key[(2, 0)]
    assert by_key[(2, 0)]["outcome"] == "queued-out", by_key[(2, 0)]
    assert by_key[(1, 0)]["outcome"] == "charged-partial", by_key[(1, 0)]
    print("queued-out: agent 2 destination decision, no event, "
          "outcome queued-out; agent 2 leg 2 towed to FAST1 "
          "(100.0-116.40, 16 kWh)")


def test_at_target_free_plug():
    # 40 kWh EV with a home charger, starts full. Leg 0 arrives at
    # 33.563 kWh, above the 32 kWh fast target, but its decision is
    # still "destination": the remaining tour (six more legs) needs
    # more energy than the post-leg charge. No charge is needed, so
    # no event is recorded and the outcome is "at-target", not
    # "charged-destination". (The only event in the tour is leg 2,
    # which tops up from 20.689 kWh to the 32 kWh target.)
    rows = [
        leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=1),
        leg_row(1, 1, 3, 1, 3, 1, 50.0, "dwell", 30, "other", vtype=1,
                path="3;2;1;", home=1),
        leg_row(1, 2, 1, 3, 1, 3, 120.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=1),
        leg_row(1, 3, 3, 1, 3, 1, 170.0, "dwell", 30, "other", vtype=1,
                path="3;2;1;", home=1),
        leg_row(1, 4, 1, 3, 1, 3, 240.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=1),
        leg_row(1, 5, 3, 1, 3, 1, 290.0, "dwell", 30, "other", vtype=1,
                path="3;2;1;", home=1),
        leg_row(1, 6, 1, 3, 1, 3, 360.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=1),
    ]
    write_inputs(BASE_ATTARGET, 3000, rows,
                 chargers="FAST1,3,150,1,fast\n")
    result = run_pysim(BASE_ATTARGET, "/tmp/decisions_out_attarget")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 1, result
    decisions = read_csv(
        "/tmp/decisions_out_attarget/output_decisions.csv")
    assert len(decisions) == 7, decisions
    d0 = decisions[0]
    assert d0["decision"] == "destination", d0
    assert d0["outcome"] == "at-target", d0
    close(d0["soc_at_departure_kwh"], 40.0)
    events = read_csv(
        "/tmp/decisions_out_attarget/output_charging.csv")
    assert len(events) == 1, events
    assert events[0]["leg_index"] == "2", events[0]
    close(events[0]["soc_before_kwh"], 20.68792)
    close(events[0]["energy_kwh"], 11.31208)
    close(events[0]["soc_after_kwh"], 32.0)
    print("at-target free plug: leg0 destination decision, outcome "
          "at-target, no event; only leg2 charges (11.312 kWh)")


def test_at_target_busy_plug():
    # Same at-target agent 2, but agent 1 (no home charger, starts
    # at 32 kWh) charges on the single fast plug 20-22.58, past
    # agent 2's 21-minute departure. Agent 2 needs no charge, so it
    # needs no plug: outcome is "at-target", not "queued-out", and
    # no event is recorded for it.
    rows = [
        leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;"),
        leg_row(1, 1, 3, 1, 3, 1, 50.0, "dwell", 30, "other", vtype=1,
                path="3;2;1;"),
        leg_row(1, 2, 1, 3, 1, 3, 120.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;"),
        leg_row(1, 3, 3, 1, 3, 1, 170.0, "dwell", 30, "other", vtype=1,
                path="3;2;1;"),
        leg_row(1, 4, 1, 3, 1, 3, 240.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;"),
        leg_row(1, 5, 3, 1, 3, 1, 290.0, "dwell", 30, "other", vtype=1,
                path="3;2;1;"),
        leg_row(1, 6, 1, 3, 1, 3, 360.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;"),
        leg_row(2, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=1),
        leg_row(2, 1, 3, 1, 3, 1, 21.0, "dwell", 1, "other", vtype=1,
                path="3;2;1;", home=1),
        leg_row(2, 2, 1, 3, 1, 3, 60.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=1),
        leg_row(2, 3, 3, 1, 3, 1, 110.0, "dwell", 30, "other", vtype=1,
                path="3;2;1;", home=1),
        leg_row(2, 4, 1, 3, 1, 3, 180.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=1),
        leg_row(2, 5, 3, 1, 3, 1, 230.0, "dwell", 30, "other", vtype=1,
                path="3;2;1;", home=1),
        leg_row(2, 6, 1, 3, 1, 3, 300.0, "fixed", 0, "work", vtype=1,
                path="1;2;3;", home=1),
    ]
    write_inputs(BASE_ATTARGET_BUSY, 3000, rows,
                 chargers="FAST1,3,150,1,fast\n")
    run_pysim(BASE_ATTARGET_BUSY, "/tmp/decisions_out_attarget_busy")
    decisions = read_csv(
        "/tmp/decisions_out_attarget_busy/output_decisions.csv")
    by_key = {(int(d["agent_id"]), int(d["leg_index"])): d
              for d in decisions}
    assert by_key[(2, 0)]["decision"] == "destination", by_key[(2, 0)]
    assert by_key[(2, 0)]["outcome"] == "at-target", by_key[(2, 0)]
    assert by_key[(1, 0)]["outcome"] == "charged-destination", \
        by_key[(1, 0)]
    events = read_csv(
        "/tmp/decisions_out_attarget_busy/output_charging.csv")
    assert any(e["agent_id"] == "1" and e["leg_index"] == "0"
               for e in events), events
    assert not any(e["agent_id"] == "2" and e["leg_index"] == "0"
                   for e in events), events
    print("at-target busy plug: agent 2 outcome at-target (not "
          "queued-out), no event, while agent 1 holds the plug")


def test_legacy_preservation():
    # test_charging.py passes unchanged (subprocess, unmodified).
    here = os.path.dirname(os.path.abspath(__file__))
    proc = subprocess.run(
        [sys.executable, os.path.join(here, "test_charging.py")],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    # Charging disabled on a tour input: traffic runs, no charging
    # files, no decisions file.
    rows = [
        leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
        leg_row(1, 1, 3, 1, 3, 1, 21.0, "dwell", 1, "other", vtype=2,
                path="3;2;1;"),
        leg_row(1, 2, 1, 3, 1, 3, 60.0, "fixed", 0, "work", vtype=2,
                path="1;2;3;"),
    ]
    write_inputs(BASE_OFF, 300, rows,
                 chargers="FAST1,3,150,1,fast\n")
    if os.path.exists("/tmp/decisions_out_off"):
        shutil.rmtree("/tmp/decisions_out_off")
    result = run_pysim(BASE_OFF, "/tmp/decisions_out_off", charging=False)
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"] is None, result
    assert "decisions" not in result["outputs"], result
    for name in ("output_charging.csv", "output_charger_summary.csv",
                 "output_stranded.csv", "output_decisions.csv"):
        assert not os.path.exists(
            os.path.join("/tmp/decisions_out_off", name)), name
    # No charger file on a tour input: same, traffic still runs.
    write_inputs(BASE_NOCH, 300, rows, chargers=None)
    result = run_pysim(BASE_NOCH, "/tmp/decisions_out_noch")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"] is None, result
    assert not os.path.exists(
        "/tmp/decisions_out_noch/output_decisions.csv")
    print("legacy: test_charging.py PASS unchanged; charging=False "
          "and no-charger-file tour runs write no charging files")


def main():
    test_partial_charge()
    test_unreachable_destination()
    test_home_charging()
    test_branches()
    test_queued_out()
    test_at_target_free_plug()
    test_at_target_busy_plug()
    test_legacy_preservation()
    print("DECISIONS PASS: partial dwell charge, unreachable "
          "destination (none, stranded), home to full "
          "(HOME row), none/destination/enroute branches, "
          "queued-out, at-target (free and busy plug), "
          "legacy preservation")
    return 0


if __name__ == "__main__":
    sys.exit(main())
