# build_dtalite_agents.py -- convert the Swedish activity plans
# (Zenodo 7549847 file 1) into a DTALite-format multi-leg tour
# agent file for the trimmed PySim charging fork, plus the
# optional_vehicle_type.csv the charging layer reads.
#
# Inputs (auto-found under input_files/ next to this script, or
# passed explicitly): 1_activity_plans.csv, 3_home_charger_access.csv,
# 4_car_fleet.csv, and input_node.csv from build_dtalite_network.py.
# Outputs (into --out, default dtalite_agents/): input_agent.csv and
# optional_vehicle_type.csv.
#
# Week model: the data holds ONE weekday plan per person, so the same
# plan is repeated for day = 0..days-1 (default 7). There is no
# weekend variation in the data; the week is a flat repetition of
# that single day. Times are absolute minutes from week start
# (0..days*1440, day = time // 1440).
#
# Per leg, from consecutive activities A -> B in the flat week chain:
#   origin/destination = the nodes nearest A/B's POINT_X/Y (snapped
#       against input_node.csv, SWEREF99TM metres -- no lat/lon
#       conversion anywhere; the simulation stays in this CRS).
#   Pairs whose snapped nodes are the same are skipped (the person
#       stays put; the overnight home->home pair across midnight is
#       the common case), and the dwell of the next emitted leg
#       accumulates the act_time of every skipped same-node activity
#       in the run ending at A.
#   departure_time_in_min = day_of_B * 1440 + B.dep_time * 60.
#       dep_time is the departure time of the trip that ENDS at B
#       (i.e. the trip from A departs at B's dep_time), in decimal
#       hours -- this is an interpretation of the plans file, not
#       something the file states outright.
#   trip_purpose = B.act_purpose (the destination activity's purpose;
#       the charging decision module uses it for home charging).
#       The plans doc lists only home/work/other, but the data also
#       has e.g. school: anything outside home/work/other is mapped
#       to "other", because the loader only accepts those three.
#       The run prints the raw purpose counts so the mapping can be
#       revisited later (e.g. school as a fixed departure like work).
#   departure_mode = "fixed" when A.act_purpose is home or work
#       (leave at the scheduled clock time), "dwell" when it is other
#       (leave after the dwell at the origin).
#   home_charger is joined from file 3 by person (0 if absent);
#   vehicle_type from file 4 car class by person: B -> 1 (40 kWh),
#   C -> 2 (60 kWh), D -> 3 (100 kWh), default 1 if absent.
#   archetype is blank (names are pending Eli + David); the
#   enroute_target_soc is 0.8 for every leg; path_node_sequence is
#   empty, so PySim assigns a free-flow shortest path at departure.
#
# Only the csv module and numpy are needed (scipy is used for
# snapping when importable, with a chunked numpy fallback otherwise).
# The plans file is ~200 MB, so it is streamed row by row; only the
# parsed per-person activity lists are held in memory.
#
# Usage:
#   python build_dtalite_agents.py [--plans PATH] [--home PATH]
#       [--fleet PATH] [--nodes PATH] [--out DIR] [--days N]
import argparse
import csv
import os
import sys
from collections import Counter
from pathlib import Path

import numpy as np

SNAP_WARN_M = 500.0
SNAP_WARN_CAP = 20
FLEET_WARN_CAP = 5
ENROUTE_TARGET_SOC = 0.8
CONSUMPTION_KWH_PER_KM = 0.18  # v1 assumption from the charging layer
DEMAND_TYPE = 1
INFORMATION_TYPE = 1
VALUE_OF_TIME = 10
VEHICLE_AGE = 5
CAR_CLASS_TO_TYPE = {"B": 1, "C": 2, "D": 3}
CAR_CLASS_TO_KWH = {"B": 40, "C": 60, "D": 100}

AGENT_HEADER = [
    "agent_id", "leg_index", "from_origin_node_id",
    "to_destination_node_id", "departure_time_in_min", "departure_mode",
    "dwell_min", "trip_purpose", "archetype", "enroute_target_soc",
    "home_charger", "vehicle_type", "demand_type", "information_type",
    "value_of_time", "vehicle_age", "tour_id", "path_node_sequence",
    "from_zone_id", "to_zone_id",
]

VEHICLE_TYPES = [
    (1, "EV 40 kWh", 1, 40, 1, CONSUMPTION_KWH_PER_KM),
    (2, "EV 60 kWh", 1, 60, 1, CONSUMPTION_KWH_PER_KM),
    (3, "EV 100 kWh", 1, 100, 1, CONSUMPTION_KWH_PER_KM),
]


def find_input(here, given, pattern, label):
    if given:
        return Path(given)
    matches = sorted(here.glob("input_files/" + pattern))
    if not matches:
        raise SystemExit(
            "no %s under input_files/; pass --%s PATH"
            % (pattern, label))
    if len(matches) > 1:
        print("multiple %s files, using %s" % (label, matches[0]))
    return matches[0]


def find_nodes(here, given):
    if given:
        return Path(given)
    for candidate in (here / "dtalite_inputs" / "input_node.csv",
                      here / "input_node.csv"):
        if candidate.exists():
            return candidate
    raise SystemExit(
        "input_node.csv not found (looked in dtalite_inputs/ next to "
        "this script); pass --nodes PATH")


def _num(value):
    # Short, stable rendering for a minute value: 510.0, not 510.0000001.
    value = round(float(value), 2)
    if value == int(value):
        return str(int(value))
    return str(value)


class Snapper:
    # Nearest-node lookup over input_node.csv. Prefers
    # scipy.spatial.cKDTree; otherwise a chunked numpy search that
    # never materialises more than ~5M distance entries at once.
    def __init__(self, nodes_path):
        ids, xs, ys = [], [], []
        with open(nodes_path, newline="", encoding="utf-8") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                ids.append(int(row["node_id"]))
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))
        if not ids:
            raise SystemExit("no nodes in %s" % nodes_path)
        self.ids = ids
        self.xy = np.array(list(zip(xs, ys)), dtype=float)
        self.tree = None
        try:
            from scipy.spatial import cKDTree
            self.tree = cKDTree(self.xy)
        except ImportError:
            print("scipy not importable; using chunked numpy snapping")

    def snap(self, points):
        # points: list of (x, y). Returns (node_ids, distances_m).
        pts = np.array(points, dtype=float)
        if self.tree is not None:
            dist, idx = self.tree.query(pts)
            return [self.ids[i] for i in idx], [float(d) for d in dist]
        node_ids, dists = [], []
        n_nodes = len(self.ids)
        # Keep each distance block under ~5M entries.
        batch = max(1, min(len(pts), 5_000_000 // max(1, n_nodes)))
        node_sq = np.sum(self.xy ** 2, axis=1)
        for start in range(0, len(pts), batch):
            block = pts[start:start + batch]
            d2 = (np.sum(block ** 2, axis=1)[:, None] + node_sq[None, :]
                  - 2.0 * block @ self.xy.T)
            idx = np.argmin(d2, axis=1)
            node_ids.extend(self.ids[i] for i in idx)
            dists.extend(float(np.sqrt(max(d2[r, i], 0.0)))
                         for r, i in enumerate(idx))
        return node_ids, dists


def read_home_chargers(path):
    chargers = {}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            try:
                person = int(row["person"])
            except (TypeError, ValueError):
                continue
            chargers[person] = 1 if str(row.get("home_charger", "0")
                                        ).strip() == "1" else 0
    return chargers


def read_fleet(path):
    fleet = {}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            try:
                person = int(row["person"])
            except (TypeError, ValueError):
                continue
            car = str(row.get("car", "")).strip().upper()
            if car in CAR_CLASS_TO_TYPE:
                fleet[person] = CAR_CLASS_TO_TYPE[car]
    return fleet


def read_plans(path):
    # Streams the plans file row by row. Returns {person: [activity,
    # ...]} with activities sorted by act_id; each activity is a dict
    # of the parsed fields this converter needs. Non-car rows are
    # skipped. A missing POINT_X/Y fails loud naming person/act_id.
    persons = {}
    counts = Counter()
    warned_purposes = set()
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            mode = str(row.get("mode", "")).strip().lower()
            if mode and mode != "car":
                counts["skipped_non_car"] += 1
                continue
            try:
                person = int(row["person"])
                act_id = int(row["act_id"])
            except (TypeError, ValueError):
                raise SystemExit(
                    "bad person/act_id in plans row: %r" % (row,))
            x_raw = str(row.get("POINT_X", "")).strip()
            y_raw = str(row.get("POINT_Y", "")).strip()
            if not x_raw or not y_raw:
                raise SystemExit(
                    "person %d act_id %d is missing POINT_X/Y"
                    % (person, act_id))
            purpose = str(row.get("act_purpose", "")).strip().lower()
            counts["raw_purpose:" + purpose] += 1
            if purpose not in ("home", "work", "other"):
                # The plans doc lists only home/work/other, but the
                # data also has e.g. school. The loader only accepts
                # home/work/other, so anything else becomes "other"
                # (dwell departure); the raw counts are printed by
                # main so the mapping can be revisited later.
                if purpose not in warned_purposes:
                    warned_purposes.add(purpose)
                    print("WARNING: unknown act_purpose %r, mapped to "
                          "'other' (person %d act_id %d)"
                          % (row.get("act_purpose"), person, act_id),
                          file=sys.stderr)
                purpose = "other"
            dep_raw = str(row.get("dep_time", "")).strip()
            act_time_raw = str(row.get("act_time", "")).strip()
            persons.setdefault(person, []).append({
                "act_id": act_id, "x": float(x_raw), "y": float(y_raw),
                "purpose": purpose,
                "dep_time": float(dep_raw) if dep_raw else None,
                "act_time": float(act_time_raw) if act_time_raw else 0.0,
            })
            counts["activities"] += 1
    for acts in persons.values():
        acts.sort(key=lambda a: a["act_id"])
    return persons, counts


def build_legs(person, acts, days, home_charger, vehicle_type):
    # Flat week chain: the day's activities repeated days times.
    # Returns the leg rows (as dicts) for one person, validated.
    n_day = len(acts)
    if n_day < 2:
        return []
    # Snap each distinct day-plan activity once; reuse across days.
    # Snapper fills in node/distance below (see main).
    chain = []
    for day in range(days):
        for act in acts:
            chain.append((day, act))
    legs = []
    # Run of consecutive same-node activities ending at the current
    # position: its node and the summed act_time (the dwell of the
    # next emitted leg out of that node).
    run_node = chain[0][1]["node"]
    run_dwell = chain[0][1]["act_time"]
    for pos in range(len(chain) - 1):
        day_a, act_a = chain[pos]
        day_b, act_b = chain[pos + 1]
        node_a = act_a["node"]
        node_b = act_b["node"]
        if node_b == node_a:
            # Staying put: extend the run at this node.
            run_dwell += act_b["act_time"]
            continue
        if act_b["dep_time"] is None:
            raise SystemExit(
                "person %d act_id %d is missing dep_time (needed for "
                "the leg out of node %d)" % (person, act_b["act_id"],
                                             node_a))
        departure = day_b * 1440 + act_b["dep_time"] * 60.0
        legs.append({
            "agent_id": person, "leg_index": len(legs),
            "from_origin_node_id": node_a,
            "to_destination_node_id": node_b,
            "departure_time_in_min": departure,
            "departure_mode": ("fixed" if act_a["purpose"] in
                               ("home", "work") else "dwell"),
            "dwell_min": run_dwell,
            "trip_purpose": act_b["purpose"],
            "archetype": "", "enroute_target_soc": ENROUTE_TARGET_SOC,
            "home_charger": home_charger, "vehicle_type": vehicle_type,
            "demand_type": DEMAND_TYPE,
            "information_type": INFORMATION_TYPE,
            "value_of_time": VALUE_OF_TIME, "vehicle_age": VEHICLE_AGE,
            "tour_id": person, "path_node_sequence": "",
            "from_zone_id": node_a, "to_zone_id": node_b,
        })
        run_node = node_b
        run_dwell = act_b["act_time"]
    # Validate before the caller writes anything for this person.
    week_end = days * 1440
    prev_to = None
    for i, leg in enumerate(legs):
        if leg["leg_index"] != i:
            raise SystemExit(
                "person %d: leg_index %d out of sequence at position %d"
                % (person, leg["leg_index"], i))
        if prev_to is not None and \
                leg["from_origin_node_id"] != prev_to:
            raise SystemExit(
                "person %d leg %d does not chain: from node %d, "
                "previous leg ended at %d"
                % (person, i, leg["from_origin_node_id"], prev_to))
        if not 0 <= leg["departure_time_in_min"] <= week_end:
            raise SystemExit(
                "person %d leg %d departure %s outside [0, %d]"
                % (person, i, leg["departure_time_in_min"], week_end))
        prev_to = leg["to_destination_node_id"]
    return legs


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--plans", default=None,
                    help="activity plans csv (default: auto-find "
                    "input_files/*activity_plans.csv)")
    ap.add_argument("--home", default=None,
                    help="home charger access csv (default: auto-find "
                    "input_files/*home_charger_access.csv)")
    ap.add_argument("--fleet", default=None,
                    help="car fleet csv (default: auto-find "
                    "input_files/*car_fleet.csv)")
    ap.add_argument("--nodes", default=None,
                    help="input_node.csv from build_dtalite_network.py "
                    "(default: dtalite_inputs/input_node.csv next to "
                    "this script)")
    ap.add_argument("--out", default=None,
                    help="output dir (default: dtalite_agents/ next to "
                    "this script)")
    ap.add_argument("--days", type=int, default=7,
                    help="days to repeat the weekday plan (default 7)")
    args = ap.parse_args()
    if args.days < 1:
        raise SystemExit("--days must be >= 1")
    plans_path = find_input(here, args.plans, "*activity_plans.csv",
                            "plans")
    home_path = find_input(here, args.home, "*home_charger_access.csv",
                           "home")
    fleet_path = find_input(here, args.fleet, "*car_fleet.csv", "fleet")
    nodes_path = find_nodes(here, args.nodes)
    out_dir = Path(args.out) if args.out else here / "dtalite_agents"
    out_dir.mkdir(parents=True, exist_ok=True)
    print("plans:", plans_path)
    print("home chargers:", home_path)
    print("fleet:", fleet_path)
    print("nodes:", nodes_path)

    snapper = Snapper(nodes_path)
    chargers = read_home_chargers(home_path)
    fleet = read_fleet(fleet_path)
    persons, counts = read_plans(plans_path)
    print("persons in plans: %d, activities kept: %d, non-car rows "
          "skipped: %d" % (len(persons), counts["activities"],
                           counts["skipped_non_car"]))
    raw_purposes = sorted(
        (key[len("raw_purpose:"):], value)
        for key, value in counts.items()
        if key.startswith("raw_purpose:"))
    print("raw act_purposes in plans: " + ", ".join(
        "%s %d" % (name, value) for name, value in raw_purposes))

    # Snap every distinct day-plan activity once.
    snap_distances = []
    snap_warned = 0
    flat_acts = [act for acts in persons.values() for act in acts]
    node_ids, dists = snapper.snap([(a["x"], a["y"]) for a in flat_acts])
    for act, node_id, dist in zip(flat_acts, node_ids, dists):
        act["node"] = node_id
        snap_distances.append(dist)
        if dist > SNAP_WARN_M:
            if snap_warned < SNAP_WARN_CAP:
                print("WARNING: snap distance %.0f m > %.0f m "
                      "(person/activity snap, x=%.1f y=%.1f -> node %d)"
                      % (dist, SNAP_WARN_M, act["x"], act["y"], node_id),
                      file=sys.stderr)
                snap_warned += 1

    agent_path = out_dir / "input_agent.csv"
    # Write to a temp file and move it into place only on success, so
    # a failure halfway through (a bad row for a later person) never
    # leaves a partial input_agent.csv behind.
    tmp_path = out_dir / "input_agent.csv.tmp"
    stats = Counter()
    missing_fleet = set()
    missing_home = set()
    fleet_warned = 0
    try:
        with open(tmp_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh, lineterminator="\r\n")
            writer.writerow(AGENT_HEADER)
            for person in sorted(persons):
                acts = persons[person]
                if person not in chargers:
                    missing_home.add(person)
                home_charger = chargers.get(person, 0)
                if person not in fleet:
                    missing_fleet.add(person)
                    if fleet_warned < FLEET_WARN_CAP:
                        print("WARNING: person %d not in fleet file; "
                              "vehicle_type 1 (40 kWh) assumed" % person,
                              file=sys.stderr)
                        fleet_warned += 1
                vehicle_type = fleet.get(person, 1)
                legs = build_legs(person, acts, args.days, home_charger,
                                  vehicle_type)
                for leg in legs:
                    writer.writerow([
                        leg["agent_id"], leg["leg_index"],
                        leg["from_origin_node_id"],
                        leg["to_destination_node_id"],
                        _num(leg["departure_time_in_min"]),
                        leg["departure_mode"], _num(leg["dwell_min"]),
                        leg["trip_purpose"], leg["archetype"],
                        leg["enroute_target_soc"], leg["home_charger"],
                        leg["vehicle_type"], leg["demand_type"],
                        leg["information_type"], leg["value_of_time"],
                        leg["vehicle_age"], leg["tour_id"],
                        leg["path_node_sequence"], leg["from_zone_id"],
                        leg["to_zone_id"],
                    ])
                    stats["legs"] += 1
                    stats["purpose_" + leg["trip_purpose"]] += 1
                    stats["mode_" + leg["departure_mode"]] += 1
                    stats["vtype_%d" % leg["vehicle_type"]] += 1
                if legs:
                    stats["agents_with_legs"] += 1
    except BaseException:
        tmp_path.unlink(missing_ok=True)
        raise
    os.replace(tmp_path, agent_path)

    with open(out_dir / "optional_vehicle_type.csv", "w", newline="",
              encoding="utf-8") as fh:
        writer = csv.writer(fh, lineterminator="\r\n")
        writer.writerow(["vehicle_type", "vehicle_type_name", "PCE",
                         "battery_kwh", "is_electric",
                         "consumption_kwh_per_km"])
        for row in VEHICLE_TYPES:
            writer.writerow(row)

    print("wrote %s" % out_dir)
    print("agents with legs: %d of %d persons, legs: %d "
          "(%.2f legs/agent/day)"
          % (stats["agents_with_legs"], len(persons), stats["legs"],
             stats["legs"] / max(1, len(persons)) / args.days))
    if snap_distances:
        over = sum(1 for d in snap_distances if d > SNAP_WARN_M)
        print("snap distance (m): mean %.1f, max %.1f, > %.0f m: %d "
              "of %d activities"
              % (sum(snap_distances) / len(snap_distances),
                 max(snap_distances), SNAP_WARN_M, over,
                 len(snap_distances)))
        if snap_warned >= SNAP_WARN_CAP:
            print("WARNING: snap warnings capped at %d lines; %d "
                  "activities exceeded %.0f m in total"
                  % (SNAP_WARN_CAP, over, SNAP_WARN_M))
    print("trip purposes: home %d, work %d, other %d"
          % (stats["purpose_home"], stats["purpose_work"],
             stats["purpose_other"]))
    print("departure modes: fixed %d, dwell %d"
          % (stats["mode_fixed"], stats["mode_dwell"]))
    print("vehicle types: " + ", ".join(
        "type %d: %d legs" % (vtype, stats["vtype_%d" % vtype])
        for vtype, _, _, _, _, _ in VEHICLE_TYPES))
    print("persons missing from fleet file: %d, missing from home "
          "charger file: %d" % (len(missing_fleet), len(missing_home)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
