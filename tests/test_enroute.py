# En-route execution tests (slice 3a).
#
# Tiny network: nodes 1-5 with 10-mile links at 60 mph in both
# directions (1<->2, 2<->3, 2<->4, 1<->5), so every link takes 10
# minutes free-flow and uses 10 * 1.60934 * 0.2 = 3.21868 kWh for
# an EV. Checks: (1) a low-SoC leg detours via an off-route charger
# (path goes via the charger node, original path stays direct, the
# en-route event carries charged_en_route = 1, SoC at destination
# is target minus the charger->dest energy, and travel time covers
# detour + dwell + queue); (2) two agents sharing one plug, so the
# second waits (wait > 0, its leg arrives after the first agent's
# charge end); (3) an agent diverted off its via path by the
# reroute hook strands before reaching its charger (no en-route
# event, leg stranded, decision still recorded as enroute --
# slice 4 tows it to the charger instead, so it arrives late
# with tow = 1).
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

BASE_DETOUR = "/tmp/enroute_case_detour"
BASE_QUEUE = "/tmp/enroute_case_queue"
BASE_STRAND = "/tmp/enroute_case_strand"

AGENT_HEADER = (
    "agent_id,tour_id,leg_index,from_zone_id,to_zone_id,"
    "from_origin_node_id,to_destination_node_id,"
    "departure_time_in_min,departure_mode,dwell_min,trip_purpose,"
    "demand_type,vehicle_type,information_type,vehicle_age,"
    "value_of_time,path_node_sequence,home_charger,archetype,"
    "enroute_target_soc\n")


def write_inputs(base, end, agent_rows, chargers):
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
            "1,enroute,1,2,0,0,100\n",
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
            "4,EV 21kWh,1,21,1,0.2\n"
            "5,EV 50kWh,1,50,1,0.2\n",
        "input_agent.csv": AGENT_HEADER + "".join(agent_rows),
        "input_charger.csv":
            "charger_id,node_id,power_kw,plugs,kind\n" + chargers,
    }
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


def leg_rows(path, agent_id, leg_index):
    rows = read_csv(path)
    return [r for r in rows
            if int(r["agent_id"]) == agent_id
            and int(r["leg_index"]) == leg_index][0]


def close(actual, expected, tol=0.05):
    assert abs(float(actual) - expected) <= tol, (actual, expected)


def depleting_legs(agent_id, first_dep, last_fixed_dep, vtype=1):
    # Eight alternating 1-link legs (1->2, 2->1, ...) that burn
    # 8 x 3.21868 kWh, then a fixed leg. Legs 1-7 leave on a
    # 5-minute dwell after the previous arrival; the last leg
    # leaves at last_fixed_dep. With a 40 kWh EV starting at 32,
    # the last leg starts at 6.25056 kWh.
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
    rows.append(leg_row(agent_id, 8, 1, 2, 1, 2, last_fixed_dep,
                        "fixed", 0, "work", vtype=vtype, path="1;2;"))
    return rows


def test_detour():
    # 40 kWh EV, charger MID5 at node 5 (50 kW, 1 plug). Leg 8
    # (1->2, direct 3.219 kWh + 4.0 margin) starts at 6.25056 kWh
    # and detours 1->5->1->2 (30 min, 9.656 kWh). It reaches node 5
    # at 140.0 with 3.03188 kWh, charges 28.96812 kWh to the 32 kWh
    # target (34.76 min), resumes at 174.8, and arrives at node 2
    # at 194.76 with 25.56264 kWh.
    rows = depleting_legs(1, 0.0, 130.0)
    write_inputs(BASE_DETOUR, 300, rows, "MID5,5,50,1,mid\n")
    result = run_pysim(BASE_DETOUR, "/tmp/enroute_out_detour")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 1, result
    decisions = read_csv(
        "/tmp/enroute_out_detour/output_decisions.csv")
    d8 = [d for d in decisions if d["leg_index"] == "8"][0]
    assert d8["decision"] == "enroute", d8
    assert d8["enroute_charger_id"] == "MID5", d8
    close(d8["soc_at_departure_kwh"], 6.25056)
    # The planning energy stays the direct-path estimate.
    close(d8["leg_energy_kwh"], 3.21868)
    assert d8["outcome"] == "completed", d8
    events = read_csv("/tmp/enroute_out_detour/output_charging.csv")
    assert len(events) == 1, events
    e = events[0]
    assert e["charger_id"] == "MID5" and e["leg_index"] == "8", e
    assert e["charged_en_route"] == "1", e
    close(e["arrival_time_in_min"], 140.0)
    close(e["wait_in_min"], 0.0)
    close(e["charge_start_in_min"], 140.0)
    close(e["charge_end_in_min"], 174.76)
    close(e["energy_kwh"], 28.96812)
    close(e["soc_before_kwh"], 3.03188)
    close(e["soc_after_kwh"], 32.0)
    row = leg_rows("/tmp/enroute_out_detour/output_agent.csv", 1, 8)
    assert row["path_node_sequence"] == "1;5;1;2;", row
    # Original path is the direct 1->2 (the writer prefixes the
    # travelled path's last node, so it reads "2;2;").
    assert "5" not in row["org_path_node_sequence"], row
    assert row["charged_en_route"] == "1", row
    assert row["stranded_flag"] == "0", row
    close(row["soc_start_kwh"], 6.25056)
    close(row["soc_end_kwh"], 25.56264)
    # Scheduled arrival uses the via free-flow time (130 + 30).
    close(row["scheduled_arrival_in_min"], 160.0)
    # Travel covers detour (30) + dwell (34.76), not the direct 10.
    close(row["arrival_time_in_min"], 194.76, 0.3)
    close(row["travel_time_in_min"], 64.76, 0.3)
    print("detour: via 1;5;1;2, 28.968 kWh at node 5, leg travel "
          "64.76 min, soc_end 25.563")


def test_one_plug_queue():
    # Two 40 kWh EVs on the detour tour, the second offset by 5
    # minutes, sharing the single plug at node 5. Agent 1 charges
    # 140.0-174.76; agent 2 arrives at 145.0, waits 29.76 min,
    # charges 174.76-209.52, and arrives at node 2 at 229.52.
    rows = depleting_legs(1, 0.0, 130.0) + depleting_legs(2, 5.0, 135.0)
    write_inputs(BASE_QUEUE, 300, rows, "MID5,5,50,1,mid\n")
    result = run_pysim(BASE_QUEUE, "/tmp/enroute_out_queue")
    assert result["agents"] == 2 and result["completed"] == 2, result
    assert result["charging"]["charges"] == 2, result
    events = read_csv("/tmp/enroute_out_queue/output_charging.csv")
    assert len(events) == 2, events
    by_agent = {int(e["agent_id"]): e for e in events}
    e1, e2 = by_agent[1], by_agent[2]
    assert e1["charger_id"] == "MID5" and e2["charger_id"] == "MID5"
    assert e1["charged_en_route"] == "1", e1
    assert e2["charged_en_route"] == "1", e2
    close(e1["charge_end_in_min"], 174.76)
    close(e2["arrival_time_in_min"], 145.0)
    close(e2["wait_in_min"], 29.76, 0.3)
    close(e2["charge_start_in_min"], 174.76, 0.3)
    close(e2["charge_end_in_min"], 209.52, 0.3)
    close(e2["energy_kwh"], 28.96812)
    row2 = leg_rows("/tmp/enroute_out_queue/output_agent.csv", 2, 8)
    assert row2["charged_en_route"] == "1", row2
    # The second leg arrives after the first agent's charge end.
    assert float(row2["arrival_time_in_min"]) > \
        float(e1["charge_end_in_min"]), (row2, e1)
    close(row2["arrival_time_in_min"], 229.52, 0.3)
    close(row2["soc_end_kwh"], 25.56264)
    print("one-plug queue: agent 2 waits 29.76 min, charges "
          "174.76-209.52, arrives 229.52")


def test_strand_before_charger():
    # 50 kWh EV, charger MID4 at node 4. Nine 1-link legs burn
    # 28.96812 kWh, so leg 9 (5->3, direct 9.656 kWh + 5.0 margin)
    # starts at 11.03188 kWh and detours 5->1->2->4->2->3 via the
    # charger. The reroute hook diverts it at node 1 onto
    # 1->2->1->2->4->2->3, and it strands on the second 1->2 link
    # (at node 2, T = 190.0) before ever reaching node 4: no
    # en-route event, decision still enroute -- but slice 4 tows
    # it. Nearest charger from node 2 is MID4 (2->4, 10 min):
    # truck arrives 220.0, tows to node 4 by 230.0, charges 0 -> 40
    # kWh at 50 kW (48 min, ends 278.0), and it drives 4->2->3,
    # arriving at 298.0 with 33.56264 kWh. The leg is flagged
    # tow = 1 and stranded = 1, and the stranded row carries the
    # tow fields. The path node sequence shows the tow jump as a
    # doubled node (2;2: stranded at 2, towed to 4, drove 4->2).
    rows = []
    node, other = 1, 2
    for i in range(8):
        rows.append(leg_row(1, i, node, other, node, other,
                            float(15 * i),
                            "fixed" if i == 0 else "dwell",
                            0 if i == 0 else 5, "other", vtype=5,
                            path="%d;%d;" % (node, other)))
        node, other = other, node
    rows.append(leg_row(1, 8, 1, 5, 1, 5, 120.0, "dwell", 5, "other",
                        vtype=5, path="1;5;"))
    rows.append(leg_row(1, 9, 5, 3, 5, 3, 150.0, "fixed", 0, "work",
                        vtype=5, path="5;1;2;3;"))
    write_inputs(BASE_STRAND, 300, rows, "MID4,4,50,1,mid\n")
    diverted = set()

    def callback(veh, next_node, world):
        if veh.agent_id != 1 or next_node in diverted:
            return None
        if not veh.legs or \
                veh.legs[veh.current_leg].leg_index != 9:
            return None
        if next_node != world.node_index[1]:
            return None
        diverted.add(next_node)
        m = world.link_map
        return [m[(1, 2)], m[(2, 1)], m[(1, 2)], m[(2, 4)],
                m[(4, 2)], m[(2, 3)]]

    result = run_pysim(BASE_STRAND, "/tmp/enroute_out_strand",
                       reroute_callback=callback)
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 1, result
    decisions = read_csv(
        "/tmp/enroute_out_strand/output_decisions.csv")
    d9 = [d for d in decisions if d["leg_index"] == "9"][0]
    assert d9["decision"] == "enroute", d9
    assert d9["enroute_charger_id"] == "MID4", d9
    close(d9["soc_at_departure_kwh"], 11.03188)
    assert d9["outcome"] == "stranded", d9
    events = read_csv("/tmp/enroute_out_strand/output_charging.csv")
    assert len(events) == 1, events
    e = events[0]
    assert e["charger_id"] == "MID4" and e["leg_index"] == "9", e
    assert e["charged_en_route"] == "0", e
    assert e["tow_charge"] == "1", e
    close(e["arrival_time_in_min"], 230.0)
    close(e["wait_in_min"], 0.0)
    close(e["charge_start_in_min"], 230.0)
    close(e["charge_end_in_min"], 278.0)
    close(e["energy_kwh"], 40.0)
    close(e["soc_before_kwh"], 0.0)
    close(e["soc_after_kwh"], 40.0)
    row = leg_rows("/tmp/enroute_out_strand/output_agent.csv", 1, 9)
    # The tow jump is visible: stranded at 2, towed to 4, drove
    # 4->2->3, so the charger node sits at the jump position (the
    # old doubled-2 reading hid the jump).
    assert row["path_node_sequence"] == "5;1;2;1;2;4;2;3;", row
    assert row["stranded_flag"] == "1", row
    assert row["tow_flag"] == "1", row
    assert row["charged_en_route"] == "0", row
    close(row["soc_end_kwh"], 33.56264)
    close(row["arrival_time_in_min"], 298.0, 0.3)
    close(row["travel_time_in_min"], 148.0, 0.3)
    stranded = read_csv("/tmp/enroute_out_strand/output_stranded.csv")
    assert len(stranded) == 1, stranded
    s = stranded[0]
    assert s["stranded_node_id"] == "2", s
    assert s["leg_index"] == "9", s
    close(s["soc_start_kwh"], 11.03188)
    close(s["trip_energy_kwh"], 22.53076)
    assert s["tow_charger_id"] == "MID4", s
    close(s["tow_start_in_min"], 220.0)
    close(s["tow_end_in_min"], 278.0)
    close(s["charge_kwh"], 40.0)
    assert s["tow_blockage_link"] == "1->2", s
    close(s["tow_blockage_start_in_min"], 190.0)
    close(s["tow_blockage_duration_min"], 30.0)
    print("strand before charger: diverted 5;1;2;1;2;4;2;3, no "
          "en-route event, towed to MID4 (220.0-278.0, 40 kWh), "
          "arrived 298.0, soc_end 33.563")


def test_blocked_retry_deducts_once():
    # Auditor probe P0, as a regression test: one 40 kWh EV on a
    # 1->3 leg via node 2, SoC forced to 30.0 kWh, with the next
    # link (2->3) at zero in-capacity, so the 1->2 discharge is
    # retried. The 1->2 link (3.21868 kWh) must be deducted
    # exactly once, not once per retry: SoC stays 26.78132 after
    # two blocked attempts, and is still 26.78132 once the next
    # link frees up and the vehicle moves on. Drives
    # _discharge_link directly: a traffic-built block needs the
    # downstream link full, which the stage-1 storage gate
    # prevents, so the zero in-capacity is set by hand.
    rows = [leg_row(1, 0, 1, 3, 1, 3, 0.0, "fixed", 0, "work",
                    vtype=1, path="1;2;3;")]
    write_inputs("/tmp/enroute_case_blocked", 300, rows,
                 "MID4,4,50,1,mid\n")
    from pysim_trimmed.network import World  # noqa: E402
    from pysim_trimmed.loading import Simulator  # noqa: E402
    from pysim_trimmed.charging import (  # noqa: E402
        ChargingEngine, DEFAULT_CONFIG)
    from pysim_trimmed.cpp import f32  # noqa: E402
    world = World("/tmp/enroute_case_blocked")
    world.reroute_callback = None
    world.read_inputs()
    world.charging_engine = ChargingEngine(world, dict(DEFAULT_CONFIG))
    sim = Simulator(world)
    sim.assign_paths_iteration_zero()
    veh = world.vehicles[1]
    leg = veh.legs[0]
    veh.begin_leg(leg)
    veh.soc_kwh = 30.0
    leg.soc_start_kwh = 30.0
    link12 = world.links[world.link_map[(1, 2)]]
    link23 = world.links[world.link_map[(2, 3)]]
    link12.exit_queue.append([1, -1, f32(10.0)])
    link12.link_out_capacity = 10
    link23.link_in_capacity = 0
    node2 = world.nodes[world.node_index[2]]
    sim._discharge_link(link12, node2, 10.0, 100)
    sim._discharge_link(link12, node2, 10.1, 101)
    close(veh.soc_kwh, 30.0 - 3.21868)
    assert len(link12.exit_queue) == 1, link12.exit_queue
    link23.link_in_capacity = 1
    sim._discharge_link(link12, node2, 10.2, 102)
    close(veh.soc_kwh, 30.0 - 3.21868)
    assert len(link12.exit_queue) == 0, link12.exit_queue
    assert len(link23.entrance_queue) == 1, link23.entrance_queue
    print("blocked retry: 1->2 deducted once over two blocked "
          "attempts, soc 26.78132, moved on unblocking")


def test_strand_into_charger():
    # Auditor probe P3, as a regression test: a 40 kWh EV that
    # strands ON the link into its en-route charger. Four 1-link
    # legs burn 12.87472 kWh and a 1->5 leg 3.21868 more, so leg 5
    # (5->3) starts at 15.9066 kWh and, with the 0.25 en-route
    # margin, decides enroute via MID4 (direct 9.65604 + 10.0
    # margin). The reroute hook diverts it at node 1 onto
    # 1->2->1->2->4->2->3, and it strands on the 2->4 link into
    # the charger (at node 4, T = 140.0), 0.1868 kWh short. The
    # stop is cancelled -- no en-route event -- but slice 4 tows
    # it: the stranded node IS the charger node, so the tow is
    # distance 0, the truck arrives 170.0, it charges 0 -> 32 kWh
    # at 50 kW (38.4 min, ends 208.4), and it drives 4->2->3,
    # arriving at 228.4 with 25.56264 kWh. The leg stays flagged
    # stranded (outcome "stranded") with tow = 1; it must not
    # charge to full on the way down and then drive the rest for
    # free -- the only charge is the tow charge at the charger.
    rows = []
    node, other = 1, 2
    for i in range(4):
        rows.append(leg_row(1, i, node, other, node, other,
                            float(15 * i),
                            "fixed" if i == 0 else "dwell",
                            0 if i == 0 else 5, "other", vtype=1,
                            path="%d;%d;" % (node, other)))
        node, other = other, node
    rows.append(leg_row(1, 4, 1, 5, 1, 5, 60.0, "dwell", 5, "other",
                        vtype=1, path="1;5;"))
    rows.append(leg_row(1, 5, 5, 3, 5, 3, 90.0, "fixed", 0, "work",
                        vtype=1, path="5;1;2;3;"))
    write_inputs("/tmp/enroute_case_strand_into", 300, rows,
                 "MID4,4,50,1,mid\n")
    diverted = set()

    def callback(veh, next_node, world):
        if veh.agent_id != 1 or next_node in diverted:
            return None
        if not veh.legs or \
                veh.legs[veh.current_leg].leg_index != 5:
            return None
        if next_node != world.node_index[1]:
            return None
        diverted.add(next_node)
        m = world.link_map
        return [m[(1, 2)], m[(2, 1)], m[(1, 2)], m[(2, 4)],
                m[(4, 2)], m[(2, 3)]]

    result = run_pysim("/tmp/enroute_case_strand_into",
                       "/tmp/enroute_out_strand_into",
                       reroute_callback=callback,
                       charging={"enroute_margin_frac": 0.25})
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 1, result
    decisions = read_csv(
        "/tmp/enroute_out_strand_into/output_decisions.csv")
    d5 = [d for d in decisions if d["leg_index"] == "5"][0]
    assert d5["decision"] == "enroute", d5
    assert d5["enroute_charger_id"] == "MID4", d5
    close(d5["soc_at_departure_kwh"], 15.9066)
    assert d5["outcome"] == "stranded", d5
    events = read_csv(
        "/tmp/enroute_out_strand_into/output_charging.csv")
    assert len(events) == 1, events
    e = events[0]
    assert e["charger_id"] == "MID4" and e["leg_index"] == "5", e
    assert e["charged_en_route"] == "0", e
    assert e["tow_charge"] == "1", e
    close(e["arrival_time_in_min"], 170.0)
    close(e["charge_start_in_min"], 170.0)
    close(e["charge_end_in_min"], 208.4)
    close(e["energy_kwh"], 32.0)
    close(e["soc_before_kwh"], 0.0)
    close(e["soc_after_kwh"], 32.0)
    row = leg_rows("/tmp/enroute_out_strand_into/output_agent.csv",
                   1, 5)
    # The tow jump is visible: stranded on 2->4 at node 4, towed
    # to the charger at 4, drove 4->2->3, so the hop doubles the
    # charger node (the splice is connected here -- the charger
    # node is the stranding link's to-node) and the tow arrival
    # time (170.0) sits at the same position.
    assert row["path_node_sequence"] == "5;1;2;1;2;4;4;2;3;", row
    assert row["stranded_flag"] == "1", row
    assert row["tow_flag"] == "1", row
    assert row["charged_en_route"] == "0", row
    close(row["soc_end_kwh"], 25.56264)
    close(row["arrival_time_in_min"], 228.4, 0.3)
    stranded = read_csv(
        "/tmp/enroute_out_strand_into/output_stranded.csv")
    assert len(stranded) == 1, stranded
    s = stranded[0]
    assert s["stranded_node_id"] == "4", s
    assert s["leg_index"] == "5", s
    close(s["soc_start_kwh"], 15.9066)
    close(s["trip_energy_kwh"], 22.53076)
    assert s["tow_charger_id"] == "MID4", s
    close(s["tow_start_in_min"], 170.0)
    close(s["tow_end_in_min"], 208.4)
    close(s["charge_kwh"], 32.0)
    assert s["tow_blockage_link"] == "2->4", s
    close(s["tow_blockage_start_in_min"], 140.0)
    close(s["tow_blockage_duration_min"], 30.0)
    print("strand into charger: diverted 5;1;2;1;2;4;2;3, no "
          "en-route event, towed at node 4 (170.0-208.4, 32 kWh), "
          "arrived 228.4, soc_end 25.563")


def stay_legs(mode, dwell5, sched5):
    # Six legs for the stay-and-finish tests (21 kWh EV, start
    # 16.8 kWh, 3.21868 kWh per 10-mile link). Legs 0-4 alternate
    # 1<->2 on dwell 0: each generates one step late (actual
    # departure 10.1, 20.1, ...), but the loading-buffer stamp
    # uses veh.departure = D_next, so arrivals stay round (10.0,
    # 20.0, 30.0, 40.0, 50.0) and leg 4 ends at node 2 with
    # 0.7066 kWh. Legs 0 and 2 decide "destination" at node 2 but
    # are queued-out (their next leg leaves at the arrival
    # instant, so the window is empty). Leg 4 decides
    # "destination" and charges at SLOW2 during leg 5's dwell
    # window; leg 5 (2->3, direct 3.21868 kWh + 2.1 margin) then
    # starts low enough to decide "enroute" via node 4.
    rows = []
    node, other = 1, 2
    for i in range(5):
        rows.append(leg_row(1, i, node, other, node, other,
                            float(10 * i),
                            "fixed" if i == 0 else "dwell",
                            0 if i == 0 else 0, "other", vtype=4,
                            path="%d;%d;" % (node, other)))
        node, other = other, node
    rows.append(leg_row(1, 5, 2, 3, 2, 3, sched5, mode, dwell5,
                        "work", vtype=4, path="2;3;"))
    return rows


def test_stay_and_finish():
    # SLOW2 at node 2 (22 kW), MID4 at node 4 (50 kW). Leg 5's
    # dwell is 11 min, so leg 4 charges [50.0, 61.0): 11 min at
    # 22 kW adds 4.03333 kWh, soc 0.7066 -> 4.73993. Leg 5 then
    # needs E = 16.8 - 4.73993 = 12.06007 kWh to its 0.8 target:
    # time_here = 12.06007 / 22 * 60 = 32.89 min, detour = 30 - 10
    # = 20 min, time_enroute = 20 + 12.06007 / 50 * 60 = 34.47
    # min. Staying is faster, the direct leg (3.21868 kWh) fits
    # in the 16.8 kWh target, and the re-decision at target is
    # "none" (node 3 has no charger), so the vehicle stays: it
    # charges [61.0, 93.89) at SLOW2, departs at the 93.9 step
    # (the first step at or after the stay end), and drives
    # direct 2->3 with no en-route event.
    rows = stay_legs("dwell", 11, 61.0)
    write_inputs("/tmp/enroute_case_stay", 300, rows,
                 "SLOW2,2,22,1,slow\nMID4,4,50,1,mid\n")
    result = run_pysim("/tmp/enroute_case_stay",
                       "/tmp/enroute_out_stay")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 2, result
    decisions = read_csv(
        "/tmp/enroute_out_stay/output_decisions.csv")
    d4 = [d for d in decisions if d["leg_index"] == "4"][0]
    assert d4["decision"] == "destination", d4
    assert d4["outcome"] == "charged-partial", d4
    d5 = [d for d in decisions if d["leg_index"] == "5"][0]
    assert d5["decision"] == "none", d5
    assert d5["enroute_charger_id"] == "", d5
    close(d5["soc_at_departure_kwh"], 16.8)
    close(d5["target_soc"], 0.0)
    events = read_csv("/tmp/enroute_out_stay/output_charging.csv")
    assert len(events) == 2, events
    assert all(e["charger_id"] == "SLOW2" for e in events), events
    assert all(e["charged_en_route"] == "0" for e in events), events
    assert all(e["leg_index"] == "4" for e in events), events
    e1, e2 = events
    close(e1["arrival_time_in_min"], 50.0)
    close(e1["charge_start_in_min"], 50.0)
    close(e1["charge_end_in_min"], 61.0)
    close(e1["energy_kwh"], 4.03333)
    close(e1["soc_before_kwh"], 0.7066)
    close(e1["soc_after_kwh"], 4.73993)
    close(e2["arrival_time_in_min"], 61.0)
    close(e2["wait_in_min"], 0.0)
    close(e2["charge_start_in_min"], 61.0)
    close(e2["charge_end_in_min"], 93.89)
    close(e2["energy_kwh"], 12.06007)
    close(e2["soc_before_kwh"], 4.73993)
    close(e2["soc_after_kwh"], 16.8)
    row = leg_rows("/tmp/enroute_out_stay/output_agent.csv", 1, 5)
    assert row["path_node_sequence"] == "2;3;", row
    assert row["charged_en_route"] == "0", row
    assert row["stranded_flag"] == "0", row
    close(row["actual_departure_in_min"], 93.9, 0.05)
    # The departure is the first step at or after the stay end
    # (93.89), never before it.
    assert float(row["actual_departure_in_min"]) >= 93.89, row
    close(row["arrival_time_in_min"], 103.9, 0.3)
    close(row["soc_start_kwh"], 16.8)
    close(row["soc_end_kwh"], 13.58132)
    print("stay-and-finish: stayed at SLOW2 to 16.8 kWh, departed "
          "93.9, direct 2->3, no en-route event")


def test_stay_fixed_leg():
    # Same setup, but leg 5 is FIXED at 61.0: fixed legs never
    # stay, so it leaves on schedule and detours via MID4. It
    # reaches node 4 at 71.0 with 1.52125 kWh, charges 15.27875
    # kWh to 16.8 (18.33 min at 50 kW), and arrives at node 3
    # with 10.36264 kWh.
    rows = stay_legs("fixed", 0, 61.0)
    write_inputs("/tmp/enroute_case_stay_fixed", 300, rows,
                 "SLOW2,2,22,1,slow\nMID4,4,50,1,mid\n")
    result = run_pysim("/tmp/enroute_case_stay_fixed",
                       "/tmp/enroute_out_stay_fixed")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 2, result
    decisions = read_csv(
        "/tmp/enroute_out_stay_fixed/output_decisions.csv")
    d5 = [d for d in decisions if d["leg_index"] == "5"][0]
    assert d5["decision"] == "enroute", d5
    assert d5["enroute_charger_id"] == "MID4", d5
    close(d5["soc_at_departure_kwh"], 4.73993)
    events = read_csv(
        "/tmp/enroute_out_stay_fixed/output_charging.csv")
    assert len(events) == 2, events
    enroute = [e for e in events if e["charged_en_route"] == "1"]
    assert len(enroute) == 1, events
    e = enroute[0]
    assert e["charger_id"] == "MID4" and e["leg_index"] == "5", e
    close(e["arrival_time_in_min"], 71.0)
    close(e["charge_start_in_min"], 71.0)
    close(e["charge_end_in_min"], 89.33, 0.3)
    close(e["energy_kwh"], 15.27875)
    close(e["soc_before_kwh"], 1.52125)
    close(e["soc_after_kwh"], 16.8)
    row = leg_rows("/tmp/enroute_out_stay_fixed/output_agent.csv",
                   1, 5)
    assert row["path_node_sequence"] == "2;4;2;3;", row
    assert row["charged_en_route"] == "1", row
    close(row["actual_departure_in_min"], 61.0)
    close(row["soc_end_kwh"], 10.36264)
    print("stay fixed leg: left on schedule, en-route via MID4, "
          "15.279 kWh, soc_end 10.363")


def test_stay_slower_here():
    # Dwell charger very slow (SLOW2, 11 kW), en-route charger
    # fast (FAST4, 150 kW). Leg 5's dwell is 20 min, so leg 4
    # charges [50.0, 70.0): 20 min at 11 kW adds 3.66667 kWh,
    # soc 0.7066 -> 4.37327. Leg 5 needs E = 12.42673 kWh:
    # time_here = 12.42673 / 11 * 60 = 67.78 min against
    # time_enroute = 20 + 12.42673 / 150 * 60 = 24.97 min, so
    # staying is slower and the leg detours via FAST4. It reaches
    # node 4 at 80.0 with 1.15459 kWh, charges 15.64541 kWh to
    # 16.8 (6.26 min at 150 kW), and arrives with 10.36264 kWh.
    rows = stay_legs("dwell", 20, 70.0)
    write_inputs("/tmp/enroute_case_stay_slower", 300, rows,
                 "SLOW2,2,11,1,slow\nFAST4,4,150,1,fast\n")
    result = run_pysim("/tmp/enroute_case_stay_slower",
                       "/tmp/enroute_out_stay_slower")
    assert result["agents"] == 1 and result["completed"] == 1, result
    assert result["charging"]["charges"] == 2, result
    decisions = read_csv(
        "/tmp/enroute_out_stay_slower/output_decisions.csv")
    d5 = [d for d in decisions if d["leg_index"] == "5"][0]
    assert d5["decision"] == "enroute", d5
    assert d5["enroute_charger_id"] == "FAST4", d5
    close(d5["soc_at_departure_kwh"], 4.37327)
    events = read_csv(
        "/tmp/enroute_out_stay_slower/output_charging.csv")
    assert len(events) == 2, events
    dwell = [e for e in events if e["charged_en_route"] == "0"]
    assert len(dwell) == 1 and dwell[0]["charger_id"] == "SLOW2", \
        events
    close(dwell[0]["energy_kwh"], 3.66667)
    enroute = [e for e in events if e["charged_en_route"] == "1"]
    assert len(enroute) == 1, events
    e = enroute[0]
    assert e["charger_id"] == "FAST4" and e["leg_index"] == "5", e
    close(e["arrival_time_in_min"], 80.0)
    close(e["charge_start_in_min"], 80.0)
    close(e["charge_end_in_min"], 86.26, 0.3)
    close(e["energy_kwh"], 15.64541)
    close(e["soc_before_kwh"], 1.15459)
    close(e["soc_after_kwh"], 16.8)
    row = leg_rows("/tmp/enroute_out_stay_slower/output_agent.csv",
                   1, 5)
    assert row["path_node_sequence"] == "2;4;2;3;", row
    assert row["charged_en_route"] == "1", row
    close(row["actual_departure_in_min"], 70.0)
    close(row["soc_end_kwh"], 10.36264)
    print("stay slower here: left at 70.0, en-route via FAST4, "
          "15.645 kWh, soc_end 10.363")


def main():
    test_detour()
    test_one_plug_queue()
    test_strand_before_charger()
    test_blocked_retry_deducts_once()
    test_strand_into_charger()
    test_stay_and_finish()
    test_stay_fixed_leg()
    test_stay_slower_here()
    print("ENROUTE PASS: detour via charger (28.968 kWh, "
          "charged_en_route 1), one-plug queue (29.76 min wait), "
          "strand before charger (towed to MID4, 40 kWh, "
          "arrived 298.0), "
          "blocked retry (1->2 deducted once, soc 26.78132), "
          "strand into charger (towed at node 4, 32 kWh, "
          "arrived 228.4), "
          "stay-and-finish (stayed at SLOW2 to 16.8 kWh, direct "
          "2->3), fixed leg (left on schedule, en-route via "
          "MID4), slower here (left at 70.0, en-route via FAST4)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
