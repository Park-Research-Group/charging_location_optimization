# Tow + stranded handling tests (slice 4).
#
# Tiny network: nodes 1-6 in a chain with 10-mile links at 60 mph
# in both directions (1<->2, 2<->3, 3<->4, 4<->5, 5<->6), a spur
# 3<->7 to a charger node, and an isolated node 8 with no links,
# so every link takes 10 minutes free-flow and uses
# 10 * 1.60934 * 0.2 = 3.21868 kWh for an EV. Checks: (1) a
# tiny-battery leg strands mid-leg and is towed to the nearest
# charger, charges to 0.8 x battery, drives to its original
# destination, and the next leg departs per the cascade rules;
# (2) no viable tow -- a charger that cannot be reached, and no
# charger file at all -- so the leg completes stranded in traffic
# as before; (3) a second stranding on the charger->dest drive is
# towed a second time (two stranded rows for the leg) and the leg
# eventually completes; (4) a charger AT the destination node is a
# tow destination for a stranded vehicle (it cannot drive there):
# the leg completes at the charge end with no post-tow drive;
# (5) the writer shows each tow jump in path_node_sequence /
# path_time_sequence (charger node + tow arrival time at the jump,
# sequences parallel in length, first stamp the leg-start buffer
# exit, sequence non-decreasing); (6) the 3-tow cap: a leg that
# would strand a fourth time completes in traffic instead.
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

BASE_TOW = "/tmp/tow_case_tow"
BASE_UNREACH = "/tmp/tow_case_unreach"
BASE_NOCH = "/tmp/tow_case_noch"
BASE_SECOND = "/tmp/tow_case_second"
BASE_DEST = "/tmp/tow_case_dest"
BASE_CAP = "/tmp/tow_case_cap"

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
        "3,4,1,1,10.0,1,60,1800,180,12\n"
        "4,3,1,1,10.0,1,60,1800,180,12\n"
        "4,5,1,1,10.0,1,60,1800,180,12\n"
        "5,4,1,1,10.0,1,60,1800,180,12\n"
        "5,6,1,1,10.0,1,60,1800,180,12\n"
        "6,5,1,1,10.0,1,60,1800,180,12\n"
        "3,7,1,1,10.0,1,60,1800,180,12\n"
        "7,3,1,1,10.0,1,60,1800,180,12\n")
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
            "1,tow,1,2,0,0,100\n",
        "input_node.csv":
            "node_id,x,y\n1,0,0\n2,1,0\n3,2,0\n4,3,0\n5,4,0\n"
            "6,5,0\n7,2,1\n8,9,9\n",
        "input_link_type.csv":
            "link_type,link_type_name,type_code\n1,Freeway,f\n",
        "input_link.csv":
            "from_node_id,to_node_id,link_type,direction,length,"
            "number_of_lanes,speed_limit,lane_cap,jam_density,wave_speed\n"
            + links,
        "input_zone.csv":
            "zone_id\n1\n2\n3\n4\n5\n6\n7\n8\n",
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
            "4,EV 21kWh,1,21,1,0.2\n"
            "5,EV 8kWh,1,8,1,0.2\n"
            "6,EV 4kWh,1,4,1,0.2\n",
        "input_agent.csv": AGENT_HEADER + "".join(agent_rows),
    }
    if chargers is not None:
        files["input_charger.csv"] = (
            "charger_id,node_id,power_kw,plugs,kind\n" + chargers)
    for name, text in files.items():
        with open(os.path.join(base, name), "w", newline="") as fh:
            fh.write(text)


def leg_row(agent_id, leg_index, from_zone, to_zone, from_node, to_node,
            sched, mode, dwell, purpose, vtype=1, path="", home=0,
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


def read_header(path):
    with open(path, newline="") as fh:
        return [h.strip() for h in next(csv.reader(fh))]


def leg_rows(path, agent_id, leg_index):
    rows = read_csv(path)
    return [r for r in rows
            if int(r["agent_id"]) == agent_id
            and int(r["leg_index"]) == leg_index][0]


def close(actual, expected, tol=0.05):
    assert abs(float(actual) - expected) <= tol, (actual, expected)


def depleting_legs(agent_id, first_dep, last_fixed_dep, vtype=1):
    # Eight alternating 1-link legs (1->2, 2->1, ...) that burn
    # 8 x 3.21868 kWh, then a fixed leg. With a 40 kWh EV starting
    # at 32, the fixed leg starts at 6.25056 kWh.
    rows = []
    node, other = 1, 2
    sched = first_dep
    for i in range(8):
        rows.append(leg_row(agent_id, i, node, other, node, other,
                            sched, "fixed" if i == 0 else "dwell",
                            0 if i == 0 else 5, "other", vtype=vtype,
                            path="%d;%d;" % (node, other)))
        sched = first_dep + 15 + 15 * i
        node, other = other, node
    # After 8 legs the vehicle is back at node 1.
    rows.append(leg_row(agent_id, 8, 1, 5, 1, 5, last_fixed_dep,
                        "fixed", 0, "work", vtype=vtype,
                        path="1;2;3;4;5;"))
    return rows


def test_tow_and_resume():
    # 40 kWh EV, charger MID7 at node 7 (50 kW, 1 plug). Leg 8
    # (1->5, 12.87472 kWh) starts at 6.25056 kWh: the en-route
    # search rejects MID7 (1->7 needs 9.65604 kWh, more than the
    # 6.25056 available), so the decision is "none" and the leg
    # drives direct, stranding on the 2->3 link at node 3
    # (T = 150.0). Nearest charger from node 3 is MID7 (3->7,
    # 10 min): the truck arrives 180.0, tows to node 7 by 190.0,
    # charges 0 -> 32 kWh at 50 kW (38.4 min, ends 228.4), and it
    # drives 7->3->4->5, arriving at 258.4 with 22.34396 kWh.
    # Leg 9 (5->6, dwell 5) then departs at 263.4, on the cascade
    # from the late arrival, and arrives at 273.4.
    rows = depleting_legs(1, 0.0, 130.0)
    rows.append(leg_row(1, 9, 5, 6, 5, 6, 200.0, "dwell", 5, "other",
                        vtype=1, path="5;6;"))
    write_inputs(BASE_TOW, 300, rows, "MID7,7,50,1,mid\n")
    result = run_pysim(BASE_TOW, "/tmp/tow_out_tow")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 1, result
    assert result["charging"]["stranded"] == 1, result
    assert "tow_charge" in read_header(
        "/tmp/tow_out_tow/output_charging.csv")
    assert "tow_charger_id" in read_header(
        "/tmp/tow_out_tow/output_stranded.csv")
    decisions = read_csv("/tmp/tow_out_tow/output_decisions.csv")
    d8 = [d for d in decisions if d["leg_index"] == "8"][0]
    assert d8["decision"] == "none", d8
    close(d8["soc_at_departure_kwh"], 6.25056)
    close(d8["leg_energy_kwh"], 12.87472)
    close(d8["remaining_energy_kwh"], 16.0934)
    assert d8["outcome"] == "stranded", d8
    events = read_csv("/tmp/tow_out_tow/output_charging.csv")
    assert len(events) == 1, events
    e = events[0]
    assert e["charger_id"] == "MID7" and e["leg_index"] == "8", e
    assert e["charged_en_route"] == "0", e
    assert e["tow_charge"] == "1", e
    close(e["arrival_time_in_min"], 190.0)
    close(e["wait_in_min"], 0.0)
    close(e["charge_start_in_min"], 190.0)
    close(e["charge_end_in_min"], 228.4)
    close(e["energy_kwh"], 32.0)
    close(e["soc_before_kwh"], 0.0)
    close(e["soc_after_kwh"], 32.0)
    row8 = leg_rows("/tmp/tow_out_tow/output_agent.csv", 1, 8)
    # The tow jump is visible in the travelled path: stranded at
    # 3, towed to 7, drove 7->3, so the charger node sits at the
    # jump position (the old doubled-3 reading hid the jump).
    assert row8["path_node_sequence"] == "1;2;3;7;3;4;5;", row8
    # Original path is the direct 1->2->3->4->5 (the writer
    # prefixes the travelled path's last node, so it reads
    # "5;2;3;4;5;").
    assert row8["org_path_node_sequence"] == "5;2;3;4;5;", row8
    assert row8["stranded_flag"] == "1", row8
    assert row8["tow_flag"] == "1", row8
    assert row8["charged_en_route"] == "0", row8
    close(row8["soc_start_kwh"], 6.25056)
    close(row8["soc_end_kwh"], 22.34396)
    close(row8["scheduled_arrival_in_min"], 180.0)
    close(row8["arrival_time_in_min"], 258.4, 0.3)
    close(row8["travel_time_in_min"], 128.4, 0.3)
    row9 = leg_rows("/tmp/tow_out_tow/output_agent.csv", 1, 9)
    assert row9["stranded_flag"] == "0", row9
    assert row9["tow_flag"] == "0", row9
    close(row9["actual_departure_in_min"], 263.4, 0.05)
    close(row9["arrival_time_in_min"], 273.4, 0.3)
    close(row9["soc_start_kwh"], 22.34396)
    close(row9["soc_end_kwh"], 19.12528)
    stranded = read_csv("/tmp/tow_out_tow/output_stranded.csv")
    assert len(stranded) == 1, stranded
    s = stranded[0]
    assert s["stranded_node_id"] == "3", s
    assert s["leg_index"] == "8", s
    close(s["trip_energy_kwh"], 12.87472)
    close(s["soc_start_kwh"], 6.25056)
    assert s["tow_charger_id"] == "MID7", s
    close(s["tow_start_in_min"], 180.0)
    close(s["tow_end_in_min"], 228.4)
    close(s["charge_kwh"], 32.0)
    assert s["tow_blockage_link"] == "2->3", s
    close(s["tow_blockage_start_in_min"], 150.0)
    close(s["tow_blockage_duration_min"], 30.0)
    summary = read_csv(
        "/tmp/tow_out_tow/output_charger_summary.csv")
    assert len(summary) == 1, summary
    assert summary[0]["charger_id"] == "MID7", summary[0]
    assert summary[0]["charges_served"] == "1", summary[0]
    close(summary[0]["total_energy_kwh"], 32.0)
    print("tow+resume: stranded at node 3 (150.0), towed to MID7 "
          "(180.0-228.4, 32 kWh), arrived 258.4, soc_end 22.344, "
          "leg 9 departed 263.4 on the cascade")


def test_no_viable_tow():
    # Same tour, but the only charger sits at isolated node 8: no
    # distance path reaches it from the stranding node, so there
    # is no tow. Leg 8 completes stranded in traffic exactly as
    # before slice 4 (arrives 170.0, soc_end 0.0), its stranded
    # row carries empty tow fields, and no tow charge is recorded.
    # The knock-on is real too: leg 8 ends at SoC 0, so leg 9
    # (5->6) strands on its only link as well, at its destination
    # node 6, and gets a second (also tow-less) stranded row.
    rows = depleting_legs(1, 0.0, 130.0)
    rows.append(leg_row(1, 9, 5, 6, 5, 6, 200.0, "dwell", 5, "other",
                        vtype=1, path="5;6;"))
    write_inputs(BASE_UNREACH, 300, rows, "ISO8,8,50,1,iso\n")
    result = run_pysim(BASE_UNREACH, "/tmp/tow_out_unreach")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 0, result
    assert result["charging"]["stranded"] == 2, result
    events = read_csv("/tmp/tow_out_unreach/output_charging.csv")
    assert len(events) == 0, events
    row8 = leg_rows("/tmp/tow_out_unreach/output_agent.csv", 1, 8)
    assert row8["path_node_sequence"] == "1;2;3;4;5;", row8
    assert row8["stranded_flag"] == "1", row8
    assert row8["tow_flag"] == "0", row8
    close(row8["soc_end_kwh"], 0.0)
    close(row8["arrival_time_in_min"], 170.0, 0.3)
    close(row8["travel_time_in_min"], 40.0, 0.3)
    row9 = leg_rows("/tmp/tow_out_unreach/output_agent.csv", 1, 9)
    assert row9["stranded_flag"] == "1", row9
    assert row9["tow_flag"] == "0", row9
    close(row9["soc_end_kwh"], 0.0)
    close(row9["arrival_time_in_min"], 185.0, 0.3)
    stranded = read_csv("/tmp/tow_out_unreach/output_stranded.csv")
    assert len(stranded) == 2, stranded
    s8, s9 = stranded
    assert s8["stranded_node_id"] == "3", s8
    assert s8["leg_index"] == "8", s8
    close(s8["trip_energy_kwh"], 12.87472)
    close(s8["soc_start_kwh"], 6.25056)
    assert s9["stranded_node_id"] == "6", s9
    assert s9["leg_index"] == "9", s9
    close(s9["trip_energy_kwh"], 3.21868)
    close(s9["soc_start_kwh"], 0.0)
    for s in stranded:
        assert s["tow_charger_id"] == "", s
        close(s["tow_start_in_min"], 0.0)
        close(s["tow_end_in_min"], 0.0)
        close(s["charge_kwh"], 0.0)
        assert s["tow_blockage_link"] == "", s
        close(s["tow_blockage_start_in_min"], 0.0)
        close(s["tow_blockage_duration_min"], 0.0)
    decisions = read_csv(
        "/tmp/tow_out_unreach/output_decisions.csv")
    by_leg = {int(d["leg_index"]): d for d in decisions}
    assert by_leg[8]["decision"] == "none", by_leg[8]
    assert by_leg[8]["outcome"] == "stranded", by_leg[8]
    assert by_leg[9]["outcome"] == "stranded", by_leg[9]
    print("no viable tow: charger at isolated node 8, leg 8 "
          "completes stranded in traffic (170.0, soc_end 0.000), "
          "empty tow fields, no event; leg 9 strands too (node 6)")


def test_no_charger_file():
    # Same tour with no charger file at all: no engine, no
    # charging files, no crash. The leg still strands and
    # completes in traffic (arrives 170.0, soc_end 0.0).
    rows = depleting_legs(1, 0.0, 130.0)
    rows.append(leg_row(1, 9, 5, 6, 5, 6, 200.0, "dwell", 5, "other",
                        vtype=1, path="5;6;"))
    write_inputs(BASE_NOCH, 300, rows, chargers=None)
    if os.path.exists("/tmp/tow_out_noch"):
        shutil.rmtree("/tmp/tow_out_noch")
    result = run_pysim(BASE_NOCH, "/tmp/tow_out_noch")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"] is None, result
    for name in ("output_charging.csv", "output_charger_summary.csv",
                 "output_stranded.csv", "output_decisions.csv"):
        assert not os.path.exists(
            os.path.join("/tmp/tow_out_noch", name)), name
    row8 = leg_rows("/tmp/tow_out_noch/output_agent.csv", 1, 8)
    assert row8["stranded_flag"] == "1", row8
    assert row8["tow_flag"] == "0", row8
    close(row8["soc_end_kwh"], 0.0)
    close(row8["arrival_time_in_min"], 170.0, 0.3)
    print("no charger file: no engine, no charging files, leg "
          "completes stranded in traffic (170.0, soc_end 0.000)")


def test_second_stranding():
    # 8 kWh EV (starts at 6.4), one leg 1->6 (16.0934 kWh), two
    # chargers: MID7 at node 7 and MID5 at node 5 (both 50 kW).
    # The en-route search rejects both (1->7 needs 9.65604 kWh,
    # 1->5 needs 12.87472, both more than 6.4), so the decision is
    # "none" and the leg drives direct. It strands on 2->3 at
    # node 3 (T = 20.0): nearest is MID7 (10 mi vs 20 to MID5),
    # truck 50.0, charge 0 -> 6.4 kWh (7.68 min, ends 67.68), and
    # it drives 7->3->4->5->6 (12.87472 kWh, more than 6.4), so it
    # strands AGAIN on 3->4 at node 4. Second tow: nearest is now
    # MID5 (10 mi vs 20 to MID7), truck 117.7, charge ends 135.38,
    # and it drives 5->6, completing at 145.4 with 3.18132 kWh.
    # Two stranded rows for the leg, two tow charge events.
    rows = [leg_row(1, 0, 1, 6, 1, 6, 0.0, "fixed", 0, "work",
                    vtype=5, path="1;2;3;4;5;6;")]
    write_inputs(BASE_SECOND, 300, rows,
                 "MID7,7,50,1,mid\nMID5,5,50,1,mid\n")
    result = run_pysim(BASE_SECOND, "/tmp/tow_out_second")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 2, result
    assert result["charging"]["stranded"] == 2, result
    decisions = read_csv(
        "/tmp/tow_out_second/output_decisions.csv")
    assert len(decisions) == 1, decisions
    d0 = decisions[0]
    assert d0["decision"] == "none", d0
    close(d0["soc_at_departure_kwh"], 6.4)
    close(d0["leg_energy_kwh"], 16.0934)
    assert d0["outcome"] == "stranded", d0
    events = read_csv("/tmp/tow_out_second/output_charging.csv")
    assert len(events) == 2, events
    assert all(e["tow_charge"] == "1" for e in events), events
    assert all(e["charged_en_route"] == "0" for e in events), events
    e1, e2 = events
    assert e1["charger_id"] == "MID7" and e1["leg_index"] == "0", e1
    close(e1["arrival_time_in_min"], 60.0, 0.3)
    close(e1["charge_start_in_min"], 60.0, 0.3)
    close(e1["charge_end_in_min"], 67.68, 0.3)
    close(e1["energy_kwh"], 6.4)
    close(e1["soc_after_kwh"], 6.4)
    assert e2["charger_id"] == "MID5" and e2["leg_index"] == "0", e2
    close(e2["arrival_time_in_min"], 127.7, 0.3)
    close(e2["charge_start_in_min"], 127.7, 0.3)
    close(e2["charge_end_in_min"], 135.38, 0.3)
    close(e2["energy_kwh"], 6.4)
    close(e2["soc_after_kwh"], 6.4)
    stranded = read_csv("/tmp/tow_out_second/output_stranded.csv")
    assert len(stranded) == 2, stranded
    s1, s2 = stranded
    assert s1["stranded_node_id"] == "3", s1
    assert s1["leg_index"] == "0", s1
    close(s1["trip_energy_kwh"], 16.0934)
    close(s1["soc_start_kwh"], 6.4)
    assert s1["tow_charger_id"] == "MID7", s1
    close(s1["tow_start_in_min"], 50.0, 0.3)
    close(s1["tow_end_in_min"], 67.68, 0.3)
    close(s1["charge_kwh"], 6.4)
    assert s1["tow_blockage_link"] == "2->3", s1
    close(s1["tow_blockage_start_in_min"], 20.0, 0.3)
    close(s1["tow_blockage_duration_min"], 30.0)
    assert s2["stranded_node_id"] == "4", s2
    assert s2["leg_index"] == "0", s2
    # Trip energy is the free-flow energy of the spliced path as
    # it stood at the second stranding (6 links, not the direct 5).
    close(s2["trip_energy_kwh"], 19.31208)
    close(s2["soc_start_kwh"], 6.4)
    assert s2["tow_charger_id"] == "MID5", s2
    close(s2["tow_start_in_min"], 117.7, 0.3)
    close(s2["tow_end_in_min"], 135.38, 0.3)
    close(s2["charge_kwh"], 6.4)
    assert s2["tow_blockage_link"] == "3->4", s2
    close(s2["tow_blockage_start_in_min"], 87.7, 0.3)
    close(s2["tow_blockage_duration_min"], 30.0)
    row = leg_rows("/tmp/tow_out_second/output_agent.csv", 1, 0)
    # Both tow jumps are visible: stranded at 3, towed to 7, drove
    # 7->3->4, stranded at 4, towed to 5, drove 5->6.
    assert row["path_node_sequence"] == "1;2;3;7;3;4;5;6;", row
    assert row["org_path_node_sequence"] == "6;2;3;4;5;6;", row
    assert row["stranded_flag"] == "1", row
    assert row["tow_flag"] == "1", row
    close(row["soc_end_kwh"], 3.18132)
    close(row["arrival_time_in_min"], 145.4, 0.3)
    close(row["travel_time_in_min"], 145.4, 0.3)
    print("second stranding: towed twice (MID7 then MID5, 6.4 kWh "
          "each), two stranded rows, leg completed 145.4, "
          "soc_end 3.181")


def test_tow_to_destination_charger():
    # 8 kWh EV (starts at 6.4), one leg 1->5 (12.87472 kWh), only
    # charger DEST5 at node 5 (50 kW). The en-route search rejects
    # it (1->5 needs 12.87472, more than 6.4), so the decision is
    # "none" and the leg drives direct, stranding on 2->3 at node 3
    # (T = 20.0). The destination charger IS a tow destination for
    # a stranded vehicle: truck 50.0, tow 3->4->5 (20 min) arrives
    # 70.0, charges 0 -> 6.4 kWh (7.68 min, ends 77.68), and the
    # leg completes there with no post-tow drive (soc_end 6.4).
    rows = [leg_row(1, 0, 1, 5, 1, 5, 0.0, "fixed", 0, "work",
                    vtype=5, path="1;2;3;4;5;")]
    write_inputs(BASE_DEST, 300, rows, "DEST5,5,50,1,mid\n")
    result = run_pysim(BASE_DEST, "/tmp/tow_out_dest")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 1, result
    assert result["charging"]["stranded"] == 1, result
    decisions = read_csv("/tmp/tow_out_dest/output_decisions.csv")
    assert len(decisions) == 1, decisions
    assert decisions[0]["decision"] == "none", decisions[0]
    assert decisions[0]["outcome"] == "stranded", decisions[0]
    events = read_csv("/tmp/tow_out_dest/output_charging.csv")
    assert len(events) == 1, events
    e = events[0]
    assert e["charger_id"] == "DEST5" and e["leg_index"] == "0", e
    assert e["tow_charge"] == "1", e
    close(e["arrival_time_in_min"], 70.0)
    close(e["wait_in_min"], 0.0)
    close(e["charge_start_in_min"], 70.0)
    close(e["charge_end_in_min"], 77.68)
    close(e["energy_kwh"], 6.4)
    close(e["soc_before_kwh"], 0.0)
    close(e["soc_after_kwh"], 6.4)
    stranded = read_csv("/tmp/tow_out_dest/output_stranded.csv")
    assert len(stranded) == 1, stranded
    s = stranded[0]
    assert s["stranded_node_id"] == "3", s
    close(s["soc_start_kwh"], 6.4)
    assert s["tow_charger_id"] == "DEST5", s
    close(s["tow_start_in_min"], 50.0)
    close(s["tow_end_in_min"], 77.68)
    close(s["charge_kwh"], 6.4)
    assert s["tow_blockage_link"] == "2->3", s
    close(s["tow_blockage_start_in_min"], 20.0)
    close(s["tow_blockage_duration_min"], 30.0)
    row = leg_rows("/tmp/tow_out_dest/output_agent.csv", 1, 0)
    assert row["stranded_flag"] == "1", row
    assert row["tow_flag"] == "1", row
    close(row["soc_end_kwh"], 6.4)
    close(row["arrival_time_in_min"], 77.68)
    close(row["travel_time_in_min"], 77.68)
    # The tow hop is appended at the end (no links are driven
    # after it): the sequence ends at the destination charger, and
    # the tow arrival time is its last stamp.
    assert row["path_node_sequence"] == "1;2;3;5;", row
    # The first stamp carries the writer's %4.1f padding (" 0.0").
    assert row["path_time_sequence"] == " 0.0;10.0;20.0;70.0;", row
    assert row["number_of_nodes"] == "4", row
    assert row["to_destination_node_id"] == "5", row
    print("tow to destination charger: stranded at node 3 (20.0), "
          "towed to DEST5 (50.0-77.68, 6.4 kWh), leg completed "
          "77.68 with no post-tow drive, soc_end 6.4")


def test_tow_path_sequences():
    # Same tour as test_tow_and_resume: leg 8 is towed from node 3
    # to MID7 (truck 180.0, tow arrival 190.0, charge ends 228.4,
    # arrives 258.4). The writer shows the jump: the charger node
    # 7 sits at the jump position in path_node_sequence, the tow
    # arrival time (190.0) sits at the same position in
    # path_time_sequence, the two sequences have equal lengths,
    # and the first time stamp is the leg-start buffer exit
    # (130.0), not the resume time (228.4) -- the sequence is
    # non-decreasing.
    rows = depleting_legs(1, 0.0, 130.0)
    rows.append(leg_row(1, 9, 5, 6, 5, 6, 200.0, "dwell", 5, "other",
                        vtype=1, path="5;6;"))
    write_inputs(BASE_TOW, 300, rows, "MID7,7,50,1,mid\n")
    result = run_pysim(BASE_TOW, "/tmp/tow_out_tow_seq")
    assert result["agents"] == 1 and result["completed"] == 1, result
    row8 = leg_rows("/tmp/tow_out_tow_seq/output_agent.csv", 1, 8)
    nodes = [int(n) for n in row8["path_node_sequence"].split(";")
             if n]
    times = [float(t) for t in row8["path_time_sequence"].split(";")
             if t]
    assert len(nodes) == len(times), (nodes, times)
    assert nodes == [1, 2, 3, 7, 3, 4, 5], nodes
    assert times == [130.0, 140.0, 150.0, 190.0, 238.4, 248.4,
                     258.4], times
    assert times[0] == 130.0, times
    assert all(b >= a for a, b in zip(times, times[1:])), times
    assert row8["number_of_nodes"] == "7", row8
    print("tow path sequences: jump visible (1;2;3;7;3;4;5;), tow "
          "arrival 190.0 at the jump, first stamp 130.0, "
          "non-decreasing, 7 nodes")


def test_tow_cap_three():
    # 4 kWh EV (starts at 3.2), one leg 1->6 (16.0934 kWh), two
    # chargers: MID7 at node 7 and MID5 at node 5 (both 50 kW).
    # The tow target is 3.2 kWh -- less than one link (3.21868) --
    # so every post-tow drive strands again on its first link:
    # tow 1 strands on 1->2 at node 2 (T = 10.0), towed to MID7;
    # tows 2 and 3 strand on 7->3 at node 3, towed back to MID7;
    # the 4th stranding hits the 3-tow cap and completes in
    # traffic. Exactly 3 tows, 4 stranded rows (3 tow + 1 untowed
    # cap row), the leg completes, no loop or crash.
    rows = [leg_row(1, 0, 1, 6, 1, 6, 0.0, "fixed", 0, "work",
                    vtype=6, path="1;2;3;4;5;6;")]
    write_inputs(BASE_CAP, 300, rows,
                 "MID7,7,50,1,mid\nMID5,5,50,1,mid\n")
    result = run_pysim(BASE_CAP, "/tmp/tow_out_cap")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 3, result
    assert result["charging"]["stranded"] == 4, result
    events = read_csv("/tmp/tow_out_cap/output_charging.csv")
    assert len(events) == 3, events
    assert all(e["tow_charge"] == "1" for e in events), events
    assert all(e["charger_id"] == "MID7" for e in events), events
    assert all(e["leg_index"] == "0" for e in events), events
    assert [e["energy_kwh"] for e in events] == ["3.200"] * 3, events
    stranded = read_csv("/tmp/tow_out_cap/output_stranded.csv")
    assert len(stranded) == 4, stranded
    assert [s["tow_charger_id"] for s in stranded] == [
        "MID7", "MID7", "MID7", ""], stranded
    assert [s["stranded_node_id"] for s in stranded] == [
        "2", "3", "3", "3"], stranded
    assert [s["leg_index"] for s in stranded] == ["0"] * 4, stranded
    # The cap row carries empty tow fields and the leg-start SoC.
    cap = stranded[3]
    close(cap["tow_start_in_min"], 0.0)
    close(cap["tow_end_in_min"], 0.0)
    close(cap["charge_kwh"], 0.0)
    assert cap["tow_blockage_link"] == "", cap
    close(cap["soc_start_kwh"], 3.2)
    row = leg_rows("/tmp/tow_out_cap/output_agent.csv", 1, 0)
    assert row["stranded_flag"] == "1", row
    assert row["tow_flag"] == "1", row
    close(row["soc_end_kwh"], 0.0)
    close(row["arrival_time_in_min"], 211.5, 0.3)
    print("tow cap: 3 tows (all MID7, 3.2 kWh), 4th stranding "
          "capped, 4 stranded rows, leg completed 211.5, "
          "soc_end 0.0")


def main():
    test_tow_and_resume()
    test_no_viable_tow()
    test_no_charger_file()
    test_second_stranding()
    test_tow_to_destination_charger()
    test_tow_path_sequences()
    test_tow_cap_three()
    print("TOW PASS: tow+resume (MID7, 32 kWh, leg 9 departed "
          "263.4 on the cascade), no viable tow (isolated charger, "
          "in-traffic completion), no charger file (no engine, no "
          "charging files), second stranding (two tows, two "
          "stranded rows, leg completed), tow to destination "
          "charger (DEST5, 6.4 kWh, completed 77.68, no post-tow "
          "drive), tow path sequences (jump visible, first stamp "
          "130.0, non-decreasing), tow cap (3 tows, 4 stranded "
          "rows, leg completed)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
