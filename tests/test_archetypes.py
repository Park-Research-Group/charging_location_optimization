# Archetype trigger tests (v5, Phase 1).
#
# Tiny bidirectional network: chain 1-2-3-4-5-6 with 10-mile links
# at 60 mph, plus branches 2-7, 7-3 and 1-7 (10 miles each), so
# every link takes 10 minutes free-flow and uses
# 10 * 1.60934 * 0.2 = 3.21868 kWh for an EV. Battery 40 kWh,
# home_charger = 0, so every agent starts at 32 kWh.
#
# Org-path note: the v4 writer renders org_path_node_sequence as
# the travelled path's last node (the destination) followed by the
# original links' to-nodes (see test_tow.py's "6;2;3;4;5;6"), not
# the original node sequence the design's case 1 prints. That is
# pre-existing v4 behaviour, unchanged by v5; the assertions here
# use the writer's actual format.
# Hand-checks follow the design's rules (not its case-6/7
# narratives, which do not follow from them -- see the notes in
# test_plan_ahead_pending / test_plan_ahead_detour): the
# plan-ahead soc < remaining gap is invariant across leg
# completions (SoC and remaining both drop by the completed leg's
# energy), so the trigger fires at the FIRST completion where it
# holds, and a leg that itself stopped en route cannot trigger
# again on completion (gate 4). Likewise the design's liquid
# case-1 numbers (threshold 0.25 after draining to 12.69 kWh)
# let the v4 generation en-route rule pre-empt the liquid
# trigger, so the detour proof uses threshold 0.60 with one
# drain leg (SoC 25.56 at generation, above leg_energy + margin,
# then crossing 24 kWh after the first link). Design case 10 as
# specified cannot exist (v4 rule 2 fires iff soc < remaining,
# exactly the shortfall condition, so a completable trigger leg
# with a charger at its arrival node always decided
# "destination"): the forced-charge branch is reachable only
# via decision "destination" + outcome "queued-out", and that
# is the path test_plan_ahead_at_node_queued_out exercises.
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

AGENT_HEADER = (
    "agent_id,tour_id,leg_index,from_zone_id,to_zone_id,"
    "from_origin_node_id,to_destination_node_id,"
    "departure_time_in_min,departure_mode,dwell_min,trip_purpose,"
    "demand_type,vehicle_type,information_type,vehicle_age,"
    "value_of_time,path_node_sequence,home_charger,archetype,"
    "enroute_target_soc,soc_threshold_frac,walk_tolerance_m\n")

CHAIN_LINKS = (
    "1,2,1,1,10.0,1,60,1800,180,12\n"
    "2,1,1,1,10.0,1,60,1800,180,12\n"
    "2,3,1,1,10.0,1,60,1800,180,12\n"
    "3,2,1,1,10.0,1,60,1800,180,12\n"
    "3,4,1,1,10.0,1,60,1800,180,12\n"
    "4,3,1,1,10.0,1,60,1800,180,12\n"
    "4,5,1,1,10.0,1,60,1800,180,12\n"
    "5,4,1,1,10.0,1,60,1800,180,12\n"
    "5,6,1,1,10.0,1,60,1800,180,12\n"
    "6,5,1,1,10.0,1,60,1800,180,12\n")

BRANCH_LINKS = (
    "2,7,1,1,10.0,1,60,1800,180,12\n"
    "7,2,1,1,10.0,1,60,1800,180,12\n"
    "7,3,1,1,10.0,1,60,1800,180,12\n"
    "3,7,1,1,10.0,1,60,1800,180,12\n"
    "1,7,1,1,10.0,1,60,1800,180,12\n"
    "7,1,1,1,10.0,1,60,1800,180,12\n")

SHORT_LINKS = (
    "6,7,1,1,0.3,1,60,1800,180,12\n"
    "7,6,1,1,0.3,1,60,1800,180,12\n")


def write_inputs(base, end, agent_rows, chargers, extra_links=""):
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
            "1,archetypes,1,2,0,0,100\n",
        "input_node.csv":
            "node_id,x,y\n1,0,0\n2,1,0\n3,2,0\n4,3,0\n5,4,0\n"
            "6,5,0\n7,2,1\n",
        "input_link_type.csv":
            "link_type,link_type_name,type_code\n1,Freeway,f\n",
        "input_link.csv":
            "from_node_id,to_node_id,link_type,direction,length,"
            "number_of_lanes,speed_limit,lane_cap,jam_density,wave_speed\n"
            + CHAIN_LINKS + extra_links,
        "input_zone.csv":
            "zone_id\n1\n2\n3\n4\n5\n6\n7\n",
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
            "3,Gas car,1,0,0,0\n",
        "input_agent.csv": AGENT_HEADER + "".join(agent_rows),
        "input_charger.csv":
            "charger_id,node_id,power_kw,plugs,kind\n" + chargers,
    }
    for name, text in files.items():
        with open(os.path.join(base, name), "w", newline="") as fh:
            fh.write(text)


def leg_row(agent_id, leg_index, from_node, to_node, sched, mode,
            dwell, purpose, archetype="", thresh=0.20, walk=500.0,
            vtype=1, path="", home=0, target=0.8):
    return (
        "%d,-1,%d,%d,%d,%d,%d,%.1f,%s,%.1f,%s,1,%d,1,5,10,%s,%d,%s,"
        "%.1f,%.4f,%.1f\n"
        % (agent_id, leg_index, from_node, to_node, from_node,
           to_node, sched, mode, dwell, purpose, vtype, path, home,
           archetype, target, thresh, walk))


def read_csv(path):
    with open(path, newline="") as fh:
        rows = [r for r in csv.reader(fh) if r]
    header = [h.strip() for h in rows[0]]
    return header, [dict(zip(header, r)) for r in rows[1:]]


def decisions(out_dir, agent_id, leg_index):
    _h, rows = read_csv(os.path.join(out_dir, "output_decisions.csv"))
    return [r for r in rows
            if int(r["agent_id"]) == agent_id
            and int(r["leg_index"]) == leg_index][0]


def agent_row(out_dir, agent_id, leg_index):
    _h, rows = read_csv(os.path.join(out_dir, "output_agent.csv"))
    return [r for r in rows
            if int(r["agent_id"]) == agent_id
            and int(r["leg_index"]) == leg_index][0]


def events(out_dir):
    _h, rows = read_csv(os.path.join(out_dir, "output_charging.csv"))
    return rows


def close(actual, expected, tol=0.05):
    assert abs(float(actual) - expected) <= tol, (actual, expected)


def assert_outputs_identical(out_a, out_b):
    for name in ("output_agent.csv", "output_decisions.csv",
                 "output_charging.csv", "output_charger_summary.csv",
                 "output_stranded.csv"):
        with open(os.path.join(out_a, name), "rb") as fh:
            a = fh.read()
        with open(os.path.join(out_b, name), "rb") as fh:
            b = fh.read()
        assert a == b, "%s differs between %s and %s" % (name, out_a, out_b)


def run_case(base, out_dir, rows, chargers, extra_links=BRANCH_LINKS,
             end=600):
    write_inputs(base, end, rows, chargers, extra_links)
    result = run_pysim(base, out_dir)
    assert result["agents"] == 1 and result["completed"] == 1, result
    return result


def liquid_rows(thresh=0.60, archetype="liquid"):
    # Two one-link drain legs 1->2, 2->1 (SoC 32 -> 25.56264;
    # each is a single link, so no mid-leg evaluation runs on
    # them), then the trigger leg 1->6. Dwell 5 between legs.
    # The drain must end at node 1 (where the trigger leg
    # starts): the design's three 1->3 legs do not chain.
    return [
        leg_row(1, 0, 1, 2, 0.0, "fixed", 0, "other",
                archetype=archetype, thresh=thresh, path="1;2;"),
        leg_row(1, 1, 2, 1, 15.0, "dwell", 5, "other",
                archetype=archetype, thresh=thresh, path="2;1;"),
        leg_row(1, 2, 1, 6, 30.0, "dwell", 5, "work",
                archetype=archetype, thresh=thresh,
                path="1;2;3;4;5;6;"),
    ]


def plan_ahead_rows(archetype="plan_ahead"):
    # 1->3 (25.56264), 3->1 (19.12528), 1->6, 6->4. The trigger
    # fires at the first leg completion (node 3), never again:
    # the leg that stopped en route cannot re-trigger (gate 4).
    return [
        leg_row(1, 0, 1, 3, 0.0, "fixed", 0, "other",
                archetype=archetype, path="1;2;3;"),
        leg_row(1, 1, 3, 1, 15.0, "dwell", 5, "other",
                archetype=archetype, path="3;2;1;"),
        leg_row(1, 2, 1, 6, 30.0, "dwell", 5, "work",
                archetype=archetype, path="1;2;3;4;5;6;"),
        leg_row(1, 3, 6, 4, 45.0, "dwell", 5, "other",
                archetype=archetype, path="6;5;4;"),
    ]


def test_liquid_detour():
    # Trigger leg starts at 25.56264 (above leg_energy 16.0934 +
    # margin 4.0, so generation decides "none"). After link 1->2
    # SoC is 22.34396 < 24.0 (threshold 0.60 x 40), so the liquid
    # trigger fires at node 2 and splices via charger MID7 at
    # node 7. Charge at node 7: arrives at 19.12528, charges
    # 12.87472 kWh to the 32 kWh target, then drives 7->3->4->5->6
    # (12.87472 kWh) and arrives with 19.12528 kWh.
    run_case("/tmp/arch_case_liquid", "/tmp/arch_out_liquid",
             liquid_rows(), "MID7,7,50,1,mid\n")
    row = agent_row("/tmp/arch_out_liquid", 1, 2)
    assert row["path_node_sequence"] == "1;2;7;3;4;5;6;", row
    assert row["org_path_node_sequence"] == "6;2;3;4;5;6;", row
    assert row["charged_en_route"] == "1", row
    assert row["stranded_flag"] == "0", row
    close(row["soc_end_kwh"], 19.12528)
    evs = [e for e in events("/tmp/arch_out_liquid")
           if e["leg_index"] == "2"]
    assert len(evs) == 1, evs
    e = evs[0]
    assert e["charger_id"] == "MID7" and e["charged_en_route"] == "1", e
    close(e["energy_kwh"], 12.87472)
    close(e["soc_before_kwh"], 19.12528)
    close(e["soc_after_kwh"], 32.0)
    d = decisions("/tmp/arch_out_liquid", 1, 2)
    assert d["trigger"] == "liquid", d
    assert d["trigger_charger_id"] == "MID7", d
    assert d["decision"] == "none", d
    assert d["enroute_charger_id"] == "", d
    assert d["outcome"] == "completed", d
    print("liquid detour: via 1;2;7;3;4;5;6, 12.875 kWh at MID7, "
          "soc_end 19.125, trigger liquid")


def test_liquid_no_stop():
    # Same fixture, charger isolated at node 7 (no branch links):
    # the nearest charger is unreachable, so no stop is planned,
    # the leg runs direct, and the outputs are byte-identical to
    # a blank-archetype control on the same fixture.
    run_case("/tmp/arch_case_liquid_nostop",
             "/tmp/arch_out_liquid_nostop", liquid_rows(),
             "ISO7,7,50,1,iso\n", extra_links="")
    run_case("/tmp/arch_case_liquid_nostop_blank",
             "/tmp/arch_out_liquid_nostop_blank",
             liquid_rows(archetype=""), "ISO7,7,50,1,iso\n",
             extra_links="")
    assert events("/tmp/arch_out_liquid_nostop") == [], \
        events("/tmp/arch_out_liquid_nostop")
    row = agent_row("/tmp/arch_out_liquid_nostop", 1, 2)
    assert row["path_node_sequence"] == "1;2;3;4;5;6;", row
    assert row["org_path_node_sequence"] == "", row
    d = decisions("/tmp/arch_out_liquid_nostop", 1, 2)
    assert d.get("trigger", "") == "", d
    assert_outputs_identical("/tmp/arch_out_liquid_nostop",
                             "/tmp/arch_out_liquid_nostop_blank")
    print("liquid no-stop: no event, direct path, outputs identical "
          "to blank control")


def test_liquid_at_current_node():
    # Charger AT the trigger node 2: the vehicle parks in place
    # in the same discharge call (charging event arrival = the
    # link 1->2 arrival, 40.0), with no path change.
    run_case("/tmp/arch_case_liquid_atnode",
             "/tmp/arch_out_liquid_atnode", liquid_rows(),
             "AT2,2,50,1,mid\n")
    row = agent_row("/tmp/arch_out_liquid_atnode", 1, 2)
    assert row["path_node_sequence"] == "1;2;3;4;5;6;", row
    assert row["org_path_node_sequence"] == "", row
    evs = [e for e in events("/tmp/arch_out_liquid_atnode")
           if e["leg_index"] == "2"]
    assert len(evs) == 1, evs
    e = evs[0]
    assert e["charger_id"] == "AT2" and e["charged_en_route"] == "1", e
    close(e["arrival_time_in_min"], 40.0)
    close(e["energy_kwh"], 9.65604)
    close(e["soc_before_kwh"], 22.34396)
    d = decisions("/tmp/arch_out_liquid_atnode", 1, 2)
    assert d["trigger"] == "liquid", d
    assert d["trigger_charger_id"] == "AT2", d
    print("liquid at-node: parked at node 2 at 40.0, 9.656 kWh, "
          "no path change")


def test_plan_ahead_pending():
    # Charger at node 2 (ON the direct path). Leg 0 completes at
    # node 3 with SoC 25.56264 < remaining 28.96812 (legs 1-3),
    # so the trigger fires there and leg 1 routes via node 2:
    # it arrives at node 2 with 22.34396, charges 9.65604 kWh
    # to 32, and reaches node 1 with 28.78132. Leg 1 cannot
    # re-trigger on completion (gate 4: it stopped en route),
    # so legs 2-3 run direct with no trigger.
    run_case("/tmp/arch_case_planahead", "/tmp/arch_out_planahead",
             plan_ahead_rows(), "MID2,2,50,1,mid\n")
    d0 = decisions("/tmp/arch_out_planahead", 1, 0)
    assert d0["trigger"] == "plan_ahead", d0
    assert d0["trigger_charger_id"] == "MID2", d0
    d1 = decisions("/tmp/arch_out_planahead", 1, 1)
    assert d1["trigger"] == "plan_ahead", d1
    assert d1["trigger_charger_id"] == "MID2", d1
    assert d1["decision"] == "enroute", d1
    assert d1["enroute_charger_id"] == "MID2", d1
    row1 = agent_row("/tmp/arch_out_planahead", 1, 1)
    assert row1["path_node_sequence"] == "3;2;1;", row1
    evs = [e for e in events("/tmp/arch_out_planahead")
           if e["leg_index"] == "1"]
    assert len(evs) == 1, evs
    close(evs[0]["energy_kwh"], 9.65604)
    assert evs[0]["charged_en_route"] == "1", evs[0]
    d2 = decisions("/tmp/arch_out_planahead", 1, 2)
    assert d2["trigger"] == "", d2
    assert d2["decision"] == "none", d2
    row2 = agent_row("/tmp/arch_out_planahead", 1, 2)
    assert row2["path_node_sequence"] == "1;2;3;4;5;6;", row2
    row3 = agent_row("/tmp/arch_out_planahead", 1, 3)
    close(row3["soc_end_kwh"], 6.25056)
    print("plan-ahead pending: trigger at leg 0, leg 1 via node 2 "
          "(9.656 kWh), legs 2-3 direct")


def test_plan_ahead_detour():
    # Charger at node 7 (off the direct path): leg 1's travelled
    # path 3;7;1 proves the detour against org path 3;2;1 (the
    # charger->dest leg is the direct 7->1 link).
    run_case("/tmp/arch_case_planahead_detour",
             "/tmp/arch_out_planahead_detour", plan_ahead_rows(),
             "MID7,7,50,1,mid\n")
    d0 = decisions("/tmp/arch_out_planahead_detour", 1, 0)
    assert d0["trigger"] == "plan_ahead", d0
    d1 = decisions("/tmp/arch_out_planahead_detour", 1, 1)
    assert d1["trigger"] == "plan_ahead", d1
    assert d1["decision"] == "enroute", d1
    row1 = agent_row("/tmp/arch_out_planahead_detour", 1, 1)
    assert row1["path_node_sequence"] == "3;7;1;", row1
    assert row1["org_path_node_sequence"] == "1;2;1;", row1
    evs = [e for e in events("/tmp/arch_out_planahead_detour")
           if e["leg_index"] == "1"]
    assert len(evs) == 1, evs
    assert evs[0]["charger_id"] == "MID7", evs[0]
    close(evs[0]["energy_kwh"], 9.65604)
    d2 = decisions("/tmp/arch_out_planahead_detour", 1, 2)
    assert d2["trigger"] == "", d2
    print("plan-ahead detour: leg 1 via 3;7;1 (org 3;2;1), "
          "9.656 kWh at MID7")


def test_plan_ahead_no_trigger():
    # Shorter tour (no leg 3): at leg 0 completion remaining is
    # 22.53076 < SoC 25.56264, and at leg 1 completion remaining
    # is 16.0934 < SoC 19.12528, so nothing triggers and the run
    # is byte-identical to a blank-archetype control (no trigger
    # columns appear in output_decisions.csv).
    rows = plan_ahead_rows()[:3]
    run_case("/tmp/arch_case_planahead_no",
             "/tmp/arch_out_planahead_no", rows,
             "MID2,2,50,1,mid\n")
    run_case("/tmp/arch_case_planahead_no_blank",
             "/tmp/arch_out_planahead_no_blank",
             plan_ahead_rows("")[:3], "MID2,2,50,1,mid\n")
    header, _rows = read_csv(
        "/tmp/arch_out_planahead_no/output_decisions.csv")
    assert "trigger" not in header, header
    assert_outputs_identical("/tmp/arch_out_planahead_no",
                             "/tmp/arch_out_planahead_no_blank")
    print("plan-ahead no-trigger: no trigger anywhere, outputs "
          "identical to blank control")


def test_opportunistic_stop():
    # Single leg 1->6, charger at node 7 joined to the destination
    # node 6 by a 0.3-mile link (482.8 m <= 500 m tolerance).
    # Origin->charger is 1;2;3;4;5;6;7 (16.18996 kWh), charger->dest
    # is 7->6 (0.09656 kWh): the leg detours via node 7, charges
    # 16.18996 kWh, and arrives with 31.90344 kWh.
    rows = [leg_row(1, 0, 1, 6, 0.0, "fixed", 0, "work",
                    archetype="opportunistic", walk=500.0,
                    path="1;2;3;4;5;6;")]
    run_case("/tmp/arch_case_opp", "/tmp/arch_out_opp", rows,
             "OPP7,7,50,1,mid\n", extra_links=SHORT_LINKS)
    row = agent_row("/tmp/arch_out_opp", 1, 0)
    assert row["path_node_sequence"] == "1;2;3;4;5;6;7;6;", row
    assert row["org_path_node_sequence"] == "6;2;3;4;5;6;", row
    assert row["charged_en_route"] == "1", row
    close(row["soc_end_kwh"], 31.90344)
    evs = events("/tmp/arch_out_opp")
    assert len(evs) == 1, evs
    assert evs[0]["charger_id"] == "OPP7", evs[0]
    assert evs[0]["charged_en_route"] == "1", evs[0]
    close(evs[0]["energy_kwh"], 16.18996)
    d = decisions("/tmp/arch_out_opp", 1, 0)
    assert d["trigger"] == "opportunistic", d
    assert d["trigger_charger_id"] == "OPP7", d
    assert d["decision"] == "enroute", d
    print("opportunistic stop: via 1;2;3;4;5;6;7;6, 16.190 kWh at "
          "OPP7, soc_end 31.903")


def test_opportunistic_too_far():
    # Same fixture with a 100 m tolerance (482.8 > 100): no stop,
    # direct leg, outputs byte-identical to a blank control.
    rows = [leg_row(1, 0, 1, 6, 0.0, "fixed", 0, "work",
                    archetype="opportunistic", walk=100.0,
                    path="1;2;3;4;5;6;")]
    blank = [leg_row(1, 0, 1, 6, 0.0, "fixed", 0, "work",
                     path="1;2;3;4;5;6;")]
    run_case("/tmp/arch_case_opp_far", "/tmp/arch_out_opp_far", rows,
             "OPP7,7,50,1,mid\n", extra_links=SHORT_LINKS)
    run_case("/tmp/arch_case_opp_far_blank",
             "/tmp/arch_out_opp_far_blank", blank,
             "OPP7,7,50,1,mid\n", extra_links=SHORT_LINKS)
    assert events("/tmp/arch_out_opp_far") == [], \
        events("/tmp/arch_out_opp_far")
    row = agent_row("/tmp/arch_out_opp_far", 1, 0)
    assert row["path_node_sequence"] == "1;2;3;4;5;6;", row
    d = decisions("/tmp/arch_out_opp_far", 1, 0)
    assert d.get("trigger", "") == "", d
    assert_outputs_identical("/tmp/arch_out_opp_far",
                             "/tmp/arch_out_opp_far_blank")
    print("opportunistic too far: no event, direct path, outputs "
          "identical to blank control")


def test_opportunistic_at_destination():
    # Charger AT the destination node 6: no splice, no trigger
    # record; the v4 destination rules govern (here: no charge,
    # SoC 32 is above every threshold), outputs identical to the
    # blank control.
    rows = [leg_row(1, 0, 1, 6, 0.0, "fixed", 0, "work",
                    archetype="opportunistic", walk=500.0,
                    path="1;2;3;4;5;6;")]
    blank = [leg_row(1, 0, 1, 6, 0.0, "fixed", 0, "work",
                     path="1;2;3;4;5;6;")]
    run_case("/tmp/arch_case_opp_dest", "/tmp/arch_out_opp_dest", rows,
             "DEST6,6,50,1,mid\n", extra_links="")
    run_case("/tmp/arch_case_opp_dest_blank",
             "/tmp/arch_out_opp_dest_blank", blank,
             "DEST6,6,50,1,mid\n", extra_links="")
    assert events("/tmp/arch_out_opp_dest") == [], \
        events("/tmp/arch_out_opp_dest")
    d = decisions("/tmp/arch_out_opp_dest", 1, 0)
    assert d.get("trigger", "") == "", d
    assert_outputs_identical("/tmp/arch_out_opp_dest",
                             "/tmp/arch_out_opp_dest_blank")
    print("opportunistic at destination: no splice, no trigger, "
          "outputs identical to blank control")


def test_unknown_archetype():
    # "bogus" falls through to the v4 default rules: outputs
    # byte-identical to the blank control on the liquid fixture.
    run_case("/tmp/arch_case_bogus", "/tmp/arch_out_bogus",
             liquid_rows(archetype="bogus"), "MID7,7,50,1,mid\n")
    run_case("/tmp/arch_case_bogus_blank", "/tmp/arch_out_bogus_blank",
             liquid_rows(archetype=""), "MID7,7,50,1,mid\n")
    assert_outputs_identical("/tmp/arch_out_bogus",
                             "/tmp/arch_out_bogus_blank")
    print("unknown archetype: outputs identical to blank control")


def test_non_ev():
    # Gas car (vehicle type 3) with a liquid archetype: no
    # triggers, no events, outputs identical to the blank control.
    rows = [leg_row(1, 0, 1, 2, 0.0, "fixed", 0, "other",
                    archetype="liquid", thresh=0.60, vtype=3,
                    path="1;2;"),
            leg_row(1, 1, 2, 1, 15.0, "dwell", 5, "other",
                    archetype="liquid", thresh=0.60, vtype=3,
                    path="2;1;"),
            leg_row(1, 2, 1, 6, 30.0, "dwell", 5, "work",
                    archetype="liquid", thresh=0.60, vtype=3,
                    path="1;2;3;4;5;6;")]
    blank = [leg_row(1, 0, 1, 2, 0.0, "fixed", 0, "other", vtype=3,
                     path="1;2;"),
             leg_row(1, 1, 2, 1, 15.0, "dwell", 5, "other", vtype=3,
                     path="2;1;"),
             leg_row(1, 2, 1, 6, 30.0, "dwell", 5, "work", vtype=3,
                     path="1;2;3;4;5;6;")]
    run_case("/tmp/arch_case_gas", "/tmp/arch_out_gas", rows,
             "MID7,7,50,1,mid\n")
    run_case("/tmp/arch_case_gas_blank", "/tmp/arch_out_gas_blank",
             blank, "MID7,7,50,1,mid\n")
    assert events("/tmp/arch_out_gas") == [], events("/tmp/arch_out_gas")
    assert_outputs_identical("/tmp/arch_out_gas",
                             "/tmp/arch_out_gas_blank")
    print("non-EV: no triggers, no events, outputs identical to "
          "blank control")


def test_determinism():
    # Same liquid-detour input twice: every output file identical.
    run_case("/tmp/arch_case_determinism",
             "/tmp/arch_out_determinism_a", liquid_rows(),
             "MID7,7,50,1,mid\n")
    run_case("/tmp/arch_case_determinism",
             "/tmp/arch_out_determinism_b", liquid_rows(),
             "MID7,7,50,1,mid\n")
    assert_outputs_identical("/tmp/arch_out_determinism_a",
                             "/tmp/arch_out_determinism_b")
    print("determinism: liquid-detour run twice, outputs identical")


def test_nearest_map_spot_check():
    # Design 12.3: the map's time/site/path agree with brute-force
    # per-site find_best_path, on the tiny fixture and on a 15x15
    # grid (225 nodes, 5 chargers, some one-way links).
    from pysim_trimmed.network import World  # noqa: E402
    from pysim_trimmed.loading import Simulator  # noqa: E402
    from pysim_trimmed.charging import (  # noqa: E402
        ChargingEngine, DEFAULT_CONFIG)
    from pysim_trimmed.sp import find_best_path  # noqa: E402

    def check(base):
        world = World(base)
        world.read_inputs()
        world.charging_engine = ChargingEngine(
            world, dict(DEFAULT_CONFIG))
        sim = Simulator(world)
        sim.assign_paths_iteration_zero()
        engine = world.charging_engine
        engine.build_nearest_map()
        assert engine.nearest_map is not None, base
        ff_net = engine._ff_net()
        n_checked = 0
        for node_id, node in enumerate(world.nodes):
            best_time = None
            best_sites = []
            for site in engine.sites:
                if node_id == site.node_id:
                    node_size, cost = 2, 0.0
                else:
                    node_size, _links, cost = find_best_path(
                        world, ff_net, node.zone_id, node_id, 0,
                        world.nodes[site.node_id].zone_id,
                        site.node_id, 1, 10.0)
                if node_size < 2:
                    continue
                if best_time is None or cost < best_time - 1e-9:
                    best_time = cost
                    best_sites = [site]
                elif abs(cost - best_time) <= 1e-9:
                    best_sites.append(site)
            m = engine.nearest_map
            if best_time is None:
                assert m.site_index[node_id] == -1, node_id
                continue
            assert abs(m.time_min[node_id] - best_time) <= 0.02, (
                node_id, m.time_min[node_id], best_time)
            assert m.nearest_site(node_id) in best_sites, node_id
            links = m.path_to_charger(node_id)
            assert links is not None, node_id
            node_now = node_id
            time_sum = 0.0
            dist_sum = 0.0
            for link_no in links:
                link = world.links[link_no]
                assert link.from_id == node_now, (node_id, link_no)
                node_now = link.to_id
                time_sum += float(link.fftt)
                dist_sum += float(link.length) * 1.60934
            assert node_now == m.nearest_site(node_id).node_id, node_id
            assert abs(time_sum - m.time_min[node_id]) <= 0.02, node_id
            assert abs(dist_sum - m.dist_km[node_id]) <= 1e-6, node_id
            n_checked += 1
        return n_checked

    write_inputs("/tmp/arch_case_map", 600, liquid_rows(),
                 "MID7,7,50,1,mid\n", BRANCH_LINKS)
    n1 = check("/tmp/arch_case_map")
    build_grid("/tmp/arch_case_grid", 15)
    n2 = check("/tmp/arch_case_grid")
    assert n1 + n2 >= 200, (n1, n2)
    print("nearest map: %d + %d nodes agree with brute force" % (n1, n2))


def build_grid(base, size):
    # size x size grid, 5-mile bidirectional links at 60 mph; a
    # deterministic subset is one-way (eastward-only odd rows,
    # southward-only even columns); chargers at 5 scattered nodes;
    # one liquid leg so the map builds.
    if os.path.exists(base):
        shutil.rmtree(base)
    os.makedirs(base)
    links = ""
    for r in range(size):
        for c in range(size):
            node = r * size + c + 1
            if c + 1 < size:
                links += "%d,%d,1,1,5.0,1,60,1800,180,12\n" % (
                    node, node + 1)
                if r % 2 == 0:
                    links += "%d,%d,1,1,5.0,1,60,1800,180,12\n" % (
                        node + 1, node)
            if r + 1 < size:
                links += "%d,%d,1,1,5.0,1,60,1800,180,12\n" % (
                    node, node + size)
                if c % 2 == 1:
                    links += "%d,%d,1,1,5.0,1,60,1800,180,12\n" % (
                        node + size, node)
    n = size * size
    chargers = [1, n // 4, n // 2, 3 * n // 4, n]
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
            "1,grid,1,2,0,0,100\n",
        "input_node.csv": "node_id,x,y\n" + "".join(
            "%d,%d,%d\n" % (i, (i - 1) % size, (i - 1) // size)
            for i in range(1, n + 1)),
        "input_link_type.csv":
            "link_type,link_type_name,type_code\n1,Freeway,f\n",
        "input_link.csv":
            "from_node_id,to_node_id,link_type,direction,length,"
            "number_of_lanes,speed_limit,lane_cap,jam_density,wave_speed\n"
            + links,
        "input_zone.csv": "zone_id\n" + "".join(
            "%d\n" % i for i in range(1, n + 1)),
        "input_demand_type.csv":
            "demand_type,avg_VOT,percentage_of_pretrip_info,"
            "percentage_of_enroute_info\n1,10,0,0\n",
        "input_demand_file_list.csv":
            "file_sequence_no,file_name,format_type,start_time_in_min,"
            "end_time_in_min\n1,input_agent.csv,agent_csv,0,600\n",
        "optional_vehicle_type.csv":
            "vehicle_type,vehicle_type_name,PCE,battery_kwh,is_electric,"
            "consumption_kwh_per_km\n1,EV car,1,40,1,0.2\n",
        "input_agent.csv": AGENT_HEADER + "".join(
            leg_row(1, 0, 1, 2, 0.0, "fixed", 0, "other",
                    archetype="liquid", thresh=0.60, path="1;2;")),
        "input_charger.csv": "charger_id,node_id,power_kw,plugs,kind\n"
            + "".join("G%d,%d,50,1,mid\n" % (i, node)
                      for i, node in enumerate(chargers)),
    }
    for name, text in files.items():
        with open(os.path.join(base, name), "w", newline="") as fh:
            fh.write(text)


def test_fallback_no_map():
    # Design 12.3: a connector ("c") link makes multi_run_valid
    # false, so no map is built -- and the liquid trigger still
    # fires via the per-site fallback search. The connector is a
    # 100-mile 1->6 link (type 2, code "c") that is never on a
    # shortest path, so the run is otherwise the detour fixture.
    base = "/tmp/arch_case_fallback"
    write_inputs(base, 600, liquid_rows(), "MID7,7,50,1,mid\n",
                 BRANCH_LINKS)
    with open(os.path.join(base, "input_link_type.csv"), "w",
              newline="") as fh:
        fh.write("link_type,link_type_name,type_code\n"
                 "1,Freeway,f\n2,Connector,c\n")
    with open(os.path.join(base, "input_link.csv"), "a",
              newline="") as fh:
        fh.write("1,6,2,1,100.0,1,60,1800,180,12\n")
    result = run_pysim(base, "/tmp/arch_out_fallback")
    assert result["agents"] == 1 and result["completed"] == 1, result
    from pysim_trimmed.network import World  # noqa: E402
    from pysim_trimmed.loading import Simulator  # noqa: E402
    from pysim_trimmed.charging import (  # noqa: E402
        ChargingEngine, DEFAULT_CONFIG)
    world = World(base)
    world.read_inputs()
    world.charging_engine = ChargingEngine(world, dict(DEFAULT_CONFIG))
    sim = Simulator(world)
    sim.assign_paths_iteration_zero()
    world.charging_engine.build_nearest_map()
    assert world.charging_engine.nearest_map is None, \
        "map built despite connector link"
    row = agent_row("/tmp/arch_out_fallback", 1, 2)
    assert row["path_node_sequence"] == "1;2;7;3;4;5;6;", row
    assert row["charged_en_route"] == "1", row
    close(row["soc_end_kwh"], 19.12528)
    evs = [e for e in events("/tmp/arch_out_fallback")
           if e["leg_index"] == "2"]
    assert len(evs) == 1, evs
    close(evs[0]["energy_kwh"], 12.87472)
    d = decisions("/tmp/arch_out_fallback", 1, 2)
    assert d["trigger"] == "liquid", d
    print("fallback: no map (connector link), liquid trigger still "
          "fires via per-site search")


def test_liquid_then_strands():
    # Design case 5: the liquid trigger fires mid-leg (splice via
    # the charger at node 5), then the reroute callback diverts the
    # vehicle off its via path onto a loop (3->2->1->2->3->4->5->6)
    # that burns the remaining charge: it strands on the 4->5 link,
    # 0.19 kWh short of the charger. The stop is cancelled (no
    # en-route event), the trigger stays recorded, and the v4 tow
    # takes it to MID5 (distance 0: stranded ON the charger node),
    # charges 0 -> 32 kWh, and it drives 5->6.
    write_inputs("/tmp/arch_case_liquid_strand", 600, liquid_rows(),
                 "MID5,5,50,1,mid\n", BRANCH_LINKS)
    diverted = set()

    def callback(veh, next_node, world):
        if veh.agent_id != 1 or next_node in diverted:
            return None
        if not veh.legs or \
                veh.legs[veh.current_leg].leg_index != 2:
            return None
        if next_node != world.node_index[3]:
            return None
        diverted.add(next_node)
        m = world.link_map
        return [m[(3, 2)], m[(2, 1)], m[(1, 2)], m[(2, 3)],
                m[(3, 4)], m[(4, 5)], m[(5, 6)]]

    result = run_pysim("/tmp/arch_case_liquid_strand",
                       "/tmp/arch_out_liquid_strand",
                       reroute_callback=callback)
    assert result["agents"] == 1 and result["completed"] == 1, result
    d = decisions("/tmp/arch_out_liquid_strand", 1, 2)
    assert d["trigger"] == "liquid", d
    assert d["trigger_charger_id"] == "MID5", d
    assert d["decision"] == "none", d
    assert d["outcome"] == "stranded", d
    evs = events("/tmp/arch_out_liquid_strand")
    assert len(evs) == 1, evs
    e = evs[0]
    assert e["charger_id"] == "MID5" and e["tow_charge"] == "1", e
    assert e["charged_en_route"] == "0", e
    assert e["leg_index"] == "2", e
    close(e["energy_kwh"], 32.0)
    close(e["arrival_time_in_min"], 140.0)
    close(e["charge_end_in_min"], 178.4)
    row = agent_row("/tmp/arch_out_liquid_strand", 1, 2)
    assert row["path_node_sequence"] == "1;2;3;2;1;2;3;4;5;5;6;", row
    assert row["stranded_flag"] == "1", row
    assert row["tow_flag"] == "1", row
    assert row["charged_en_route"] == "0", row
    close(row["soc_end_kwh"], 28.78132)
    close(row["arrival_time_in_min"], 188.4)
    _h, srows = read_csv(
        "/tmp/arch_out_liquid_strand/output_stranded.csv")
    assert len(srows) == 1, srows
    s = srows[0]
    assert s["stranded_node_id"] == "5", s
    assert s["leg_index"] == "2", s
    assert s["tow_charger_id"] == "MID5", s
    assert s["tow_blockage_link"] == "4->5", s
    close(s["tow_blockage_start_in_min"], 110.0)
    print("liquid then strands: diverted off the via at node 3, "
          "stranded on 4->5, towed at node 5 (140.0-178.4, 32 kWh), "
          "trigger liquid still recorded")


def test_plan_ahead_at_node_queued_out():
    # Design case 10 via the queued-out path (see the header note:
    # the as-specified "none"-decision fixture cannot exist). The
    # only charger is a slow 7 kW site AT node 1. A 20 kWh
    # background agent occupies its single plug [10.0, 71.9)
    # (7.219 kWh to full), so the plan-ahead agent's leg 1 (2->1)
    # decides "destination" but is queued out (plug busy past its
    # 30.0 next departure), and the forced charge queues out the
    # same way. Assert: trigger recorded, outcome queued-out, no
    # charging event for the leg, no pending via (leg 2 runs direct
    # on v4 rules). Leg 0 records nothing: the charger is leg 1's
    # destination, so the at-destination sub-case applies there.
    # Leg 3 strands and is towed to AT1 (v4 tow mechanics, covered
    # by test_tow.py); the test does not depend on it.
    rows = [
        leg_row(1, 0, 1, 2, 0.0, "fixed", 0, "other",
                archetype="plan_ahead", path="1;2;"),
        leg_row(1, 1, 2, 1, 15.0, "dwell", 5, "other",
                archetype="plan_ahead", path="2;1;"),
        leg_row(1, 2, 1, 6, 30.0, "dwell", 5, "work",
                archetype="plan_ahead", path="1;2;3;4;5;6;"),
        leg_row(1, 3, 6, 1, 45.0, "dwell", 5, "other",
                archetype="plan_ahead", path="6;5;4;3;2;1;"),
        leg_row(2, 0, 2, 1, 0.0, "fixed", 0, "other", vtype=2,
                path="2;1;"),
        leg_row(2, 1, 1, 6, 10.0, "dwell", 300, "work", vtype=2,
                path="1;2;3;4;5;6;"),
    ]
    write_inputs("/tmp/arch_case_planahead_queued", 600, rows,
                 "AT1,1,7,1,slow\n", BRANCH_LINKS)
    result = run_pysim("/tmp/arch_case_planahead_queued",
                       "/tmp/arch_out_planahead_queued")
    assert result["agents"] == 2 and result["completed"] == 2, result
    # The plug really was busy past leg 1's 30.0 departure: the
    # background agent's leg-0 charge runs [10.0, 71.9).
    bg = [e for e in events("/tmp/arch_out_planahead_queued")
          if e["agent_id"] == "2"]
    assert len(bg) == 1, bg
    assert bg[0]["charger_id"] == "AT1", bg[0]
    close(bg[0]["energy_kwh"], 7.21868)
    close(bg[0]["charge_end_in_min"], 71.87)
    d0 = decisions("/tmp/arch_out_planahead_queued", 1, 0)
    assert d0["trigger"] == "", d0
    d1 = decisions("/tmp/arch_out_planahead_queued", 1, 1)
    assert d1["trigger"] == "plan_ahead", d1
    assert d1["trigger_charger_id"] == "AT1", d1
    assert d1["decision"] == "destination", d1
    assert d1["outcome"] == "queued-out", d1
    mine = [e for e in events("/tmp/arch_out_planahead_queued")
            if e["agent_id"] == "1" and e["leg_index"] == "1"]
    assert mine == [], mine
    d2 = decisions("/tmp/arch_out_planahead_queued", 1, 2)
    assert d2["trigger"] == "", d2
    assert d2["decision"] == "none", d2
    row2 = agent_row("/tmp/arch_out_planahead_queued", 1, 2)
    assert row2["path_node_sequence"] == "1;2;3;4;5;6;", row2
    print("plan-ahead at-node queued-out: destination charge and "
          "forced charge both queued out (plug busy to 71.9), "
          "trigger recorded, leg 2 direct")


def test_plan_ahead_pending_not_stolen_by_stay():
    # Design case 16: the pending leg's origin (node 6) has a dwell
    # charger from the previous leg -- but it is a HOME charge, not
    # a site, so the trigger's nearest-site lookup still returns
    # the slow site at node 7 and a pending via is created. Leg 1
    # (6->4, dwell) consumes it (decision enroute, trigger
    # plan_ahead) while last_dwell_charge is still the HOME charge
    # at node 6: without the v5 guard in try_stay_and_finish the
    # stay would fire (staying at 11 kW to 32 kWh takes 39.1 min
    # vs 61.8 min via the 7 kW site, and the re-decision at target
    # is "none"), so the via path and the un-topped-up starting
    # charge prove the guard. Leg 0: 1->6, home purpose, starts
    # full (home charger), arrives with 23.907 and HOME-charges
    # [50.0, 55.0) to 24.823. (Leg 3 later takes a plain v4 stay at
    # HOME -- it carries no trigger, so the stay machinery firing
    # there is expected and not asserted here.)
    rows = [
        leg_row(1, 0, 1, 6, 0.0, "fixed", 0, "home",
                archetype="plan_ahead", path="1;2;3;4;5;6;", home=1),
        leg_row(1, 1, 6, 4, 55.0, "dwell", 5, "work",
                archetype="plan_ahead", path="6;5;4;", home=1),
        leg_row(1, 2, 4, 6, 140.0, "dwell", 5, "other",
                archetype="plan_ahead", path="4;5;6;", home=1),
        leg_row(1, 3, 6, 1, 165.0, "dwell", 5, "other",
                archetype="plan_ahead", path="6;5;4;3;2;1;", home=1),
    ]
    run_case("/tmp/arch_case_stay_guard", "/tmp/arch_out_stay_guard",
             rows, "SLOW7,7,7,1,slow\n",
             extra_links=BRANCH_LINKS + SHORT_LINKS)
    d0 = decisions("/tmp/arch_out_stay_guard", 1, 0)
    assert d0["trigger"] == "plan_ahead", d0
    assert d0["trigger_charger_id"] == "SLOW7", d0
    assert d0["decision"] == "destination", d0
    assert d0["outcome"] == "charged-partial", d0
    d1 = decisions("/tmp/arch_out_stay_guard", 1, 1)
    assert d1["trigger"] == "plan_ahead", d1
    assert d1["trigger_charger_id"] == "SLOW7", d1
    assert d1["decision"] == "enroute", d1
    assert d1["enroute_charger_id"] == "SLOW7", d1
    row1 = agent_row("/tmp/arch_out_stay_guard", 1, 1)
    assert row1["path_node_sequence"] == "6;7;3;4;", row1
    assert row1["org_path_node_sequence"] == "4;5;4;", row1
    assert row1["charged_en_route"] == "1", row1
    # No stay: the leg left on schedule with the post-HOME-charge
    # SoC (a stay would have topped it to 32 and departed ~94).
    close(row1["soc_start_kwh"], 24.82331)
    close(row1["actual_departure_in_min"], 55.0)
    close(row1["soc_end_kwh"], 25.56264)
    home0 = [e for e in events("/tmp/arch_out_stay_guard")
             if e["leg_index"] == "0"]
    assert len(home0) == 1, home0
    assert home0[0]["charger_id"] == "HOME", home0[0]
    close(home0[0]["energy_kwh"], 0.91667)
    ev1 = [e for e in events("/tmp/arch_out_stay_guard")
           if e["leg_index"] == "1"]
    assert len(ev1) == 1, ev1
    assert ev1[0]["charger_id"] == "SLOW7", ev1[0]
    assert ev1[0]["charged_en_route"] == "1", ev1[0]
    close(ev1[0]["energy_kwh"], 7.27325)
    close(ev1[0]["soc_before_kwh"], 24.72675)
    close(ev1[0]["soc_after_kwh"], 32.0)
    print("plan-ahead vs stay: pending via kept (6;7;3;4, 7.273 kWh "
          "at SLOW7), no stay top-up (soc_start 24.823, dep 55.0)")


def main():
    test_liquid_detour()
    test_liquid_no_stop()
    test_liquid_at_current_node()
    test_plan_ahead_pending()
    test_plan_ahead_detour()
    test_plan_ahead_no_trigger()
    test_opportunistic_stop()
    test_opportunistic_too_far()
    test_opportunistic_at_destination()
    test_unknown_archetype()
    test_non_ev()
    test_determinism()
    test_nearest_map_spot_check()
    test_fallback_no_map()
    test_liquid_then_strands()
    test_plan_ahead_at_node_queued_out()
    test_plan_ahead_pending_not_stolen_by_stay()
    print("ARCHETYPES PASS: liquid detour/no-stop/at-node, "
          "plan-ahead pending/detour/no-trigger, opportunistic "
          "stop/too-far/at-destination, unknown archetype, non-EV, "
          "determinism, nearest-map spot-check, no-map fallback, "
          "liquid-then-strands, plan-ahead at-node queued-out, "
          "plan-ahead pending kept over stay-and-finish")
    return 0


if __name__ == "__main__":
    sys.exit(main())
