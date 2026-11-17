#!/usr/bin/env python3
"""Build real-charger inputs for the PySim Swedish run.

Reads the (NOBIL station, OSM POI) pair file (one .xlsx sheet, one
row per pair within 1 km walk) plus a Swedish network node file
(input_node.csv, SWEREF99TM x/y), and writes two PySim inputs:

  input_charger.csv      one row per power level per station
                         (charger_id, node_id, power_kw, plugs, kind)
  input_poi_charger.csv  one row per (POI, charger) pair, for the
                         opportunistic walk test
                         (poi_osm_id, poi_node_id, poi_category,
                         poi_type, charger_id, walk_dist_m,
                         straight_dist_m, parking_fee, network, price)

Rules (Eli's answers, Nov 2026):
  - Stations are deduplicated by station_id (first row wins; the
    station-level fields are constant across a station's rows).
  - vehicle_scope != car is dropped (the truck/bus stations).
  - Stations with no power info (power_kw_max missing) are dropped.
  - Stations with no ports at all are dropped.
  - Every kept charger snaps to its nearest node (cKDTree);
    chargers beyond --max-snap-km (default 2.0 km) are dropped.
  - Level 2 is ONE power level for all ports: --level2-kw
    (default 22.0 kW). Per-station L2 power is NOT parsed.
  - Level 3 uses the station's power_kw_max. Level 1 is emitted
    only when it is the station's only level (2 stations in the
    file), at power_kw_max.
  - kind is "fast" when power_kw >= 50, else "slow" (the random
    generator's convention).
  - charger_id is the bare station id for single-level stations,
    and station_id + _L1/_L2/_L3 for multi-row stations. The POI
    file references the bare station id.
  - The POI file keeps ALL categories and uses the TRUE walk
    distance from the pair file (not the road-network distance).
    Rows with no POI data are dropped from the POI file only; the
    charger itself is still kept. POI rows for dropped stations
    are dropped too (they would reference a charger that is not
    in the run). price and parking_fee are carried through for
    later use; PySim does not use them yet.
  - opening_hours is ignored (99% 24/7, the rest give no hours).

Snap distances are in metres (SWEREF99TM). The report printed at
the end lists the snap-distance distribution, stations kept and
dropped with reasons, charger rows by power level, the POI row
count, and the flagged outliers.

The two input files are optional: when omitted, the .xlsx is found
in input_files (preferring a name containing "charger" or "poi")
and input_node.csv is found in the dtalite input directory
(--dtalite-dir, default dtalite_inputs). Pass them explicitly to
use other files. The two csv files are written to the dtalite
input directory by default.

Requires pyproj and scipy (pip install pyproj scipy); both are
hard requirements, with a clear error when missing.
"""

import argparse
import csv
import glob
import os
import sys

try:
    import pyproj
except ImportError:
    sys.stderr.write(
        "build_chargers_real.py: pyproj is required "
        "(pip install pyproj)\n")
    sys.exit(2)

try:
    from scipy.spatial import cKDTree
except ImportError:
    sys.stderr.write(
        "build_chargers_real.py: scipy is required "
        "(pip install scipy)\n")
    sys.exit(2)

try:
    import openpyxl
except ImportError:
    sys.stderr.write(
        "build_chargers_real.py: openpyxl is required "
        "(pip install openpyxl)\n")
    sys.exit(2)

FAST_THRESHOLD_KW = 50.0
SNAP_BANDS_M = (100.0, 500.0, 1000.0, 2000.0, 5000.0)


def fmt_num(value):
    # 22.0 -> "22", 3.6 -> "3.6": the existing charger files
    # write integral powers without a decimal point.
    if value is None:
        return ""
    if isinstance(value, float) and value == int(value):
        return str(int(value))
    return "%g" % value


def fmt_str(value):
    if value is None:
        return ""
    return str(value)


def read_nodes(path):
    # {node_id: (x, y)} from a DTALite input_node.csv. First
    # occurrence wins; a repeated node id is warned about.
    nodes = {}
    with open(path, newline="", encoding="utf-8-sig") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            try:
                node_id = int(row["node_id"])
                x = float(row["x"])
                y = float(row["y"])
            except (KeyError, TypeError, ValueError):
                raise SystemExit(
                    "bad node row in %s: %r" % (path, row))
            if node_id in nodes:
                sys.stderr.write(
                    "warning: duplicate node_id %d in %s, "
                    "keeping the first\n" % (node_id, path))
                continue
            nodes[node_id] = (x, y)
    if not nodes:
        raise SystemExit("no nodes read from %s" % path)
    return nodes


def find_xlsx(folder):
    cands = sorted(glob.glob(os.path.join(folder, "*.xlsx")))
    if not cands:
        raise SystemExit("no .xlsx file in %s" % folder)
    named = [c for c in cands
             if "charger" in os.path.basename(c).lower()
             or "poi" in os.path.basename(c).lower()]
    if len(named) == 1:
        return named[0]
    if len(cands) == 1:
        return cands[0]
    raise SystemExit("several .xlsx files in %s, pass one "
                     "explicitly: %s" % (folder, ", ".join(cands)))


def find_nodes(folder):
    path = os.path.join(folder, "input_node.csv")
    if not os.path.isfile(path):
        raise SystemExit("no input_node.csv in %s" % folder)
    return path


def main(argv):
    parser = argparse.ArgumentParser(
        description="Build input_charger.csv and "
                    "input_poi_charger.csv from the NOBIL/OSM "
                    "pair file and a Swedish node file.")
    parser.add_argument("xlsx", nargs="?", default=None,
                        help="pair file (.xlsx); when omitted, "
                             "found in --input-files")
    parser.add_argument("nodes", nargs="?", default=None,
                        help="input_node.csv (SWEREF99TM); when "
                             "omitted, found in --dtalite-dir")
    parser.add_argument("--input-files", default="input_files",
                        help="folder to search for the .xlsx when it is "
                             "omitted (default: input_files)")
    parser.add_argument("--dtalite-dir", default="dtalite_inputs",
                        help="folder to search for input_node.csv when "
                             "nodes is omitted (default: dtalite_inputs)")
    parser.add_argument("--out-dir", default=None,
                        help="where to write the two csv files "
                             "(default: the dtalite input directory)")
    parser.add_argument("--sheet", default="in",
                        help="xlsx sheet name (default: in)")
    parser.add_argument("--level2-kw", type=float, default=22.0,
                        help="the one Level 2 power in kW "
                             "(default: 22.0)")
    parser.add_argument("--max-snap-km", type=float, default=2.0,
                        help="drop chargers snapping farther than "
                             "this (default: 2.0 km)")
    args = parser.parse_args(argv)
    if args.level2_kw <= 0:
        parser.error("--level2-kw must be positive")
    if args.max_snap_km <= 0:
        parser.error("--max-snap-km must be positive")
    if args.out_dir is None:
        args.out_dir = args.dtalite_dir

    if args.xlsx is None:
        args.xlsx = find_xlsx(args.input_files)
        print("xlsx: %s" % args.xlsx)
    if args.nodes is None:
        args.nodes = find_nodes(args.dtalite_dir)
        print("nodes: %s" % args.nodes)
    nodes = read_nodes(args.nodes)
    node_ids = sorted(nodes)
    coords = [nodes[n] for n in node_ids]
    tree = cKDTree(coords)
    project = pyproj.Transformer.from_crs(
        "EPSG:4326", "EPSG:3006", always_xy=True).transform

    wb = openpyxl.load_workbook(args.xlsx, read_only=True,
                                data_only=True)
    if args.sheet not in wb.sheetnames:
        raise SystemExit("sheet %r not in %s (sheets: %s)"
                         % (args.sheet, args.xlsx,
                            ", ".join(wb.sheetnames)))
    ws = wb[args.sheet]
    rows = ws.iter_rows(values_only=True)
    try:
        header = next(rows)
    except StopIteration:
        raise SystemExit("sheet %r is empty" % args.sheet)
    col = {}
    for i, name in enumerate(header):
        if name is not None:
            col[str(name).strip()] = i
    needed = ("station_id", "vehicle_scope", "charger_lat",
              "charger_lon", "capacity", "level_1_ports",
              "level_2_ports", "level_3_ports", "power_kw_max",
              "network", "poi_osm_id", "poi_category", "poi_type",
              "poi_lat", "poi_lon", "walk_dist_m",
              "straight_dist_m", "parking_fee", "price")
    missing = [n for n in needed if n not in col]
    if missing:
        raise SystemExit("xlsx is missing columns: %s"
                         % ", ".join(missing))

    def val(row, name):
        return row[col[name]]

    # Pass 1: one record per station (first row wins), plus the
    # POI rows in file order. Station-level fields are constant
    # across a station's rows; a disagreement is warned about
    # and the first row's values are kept.
    stations = {}
    station_order = []
    poi_rows = []
    n_rows = 0
    n_null_poi = 0
    inconsistent = []
    for row in rows:
        n_rows += 1
        station_id = val(row, "station_id")
        if station_id is None:
            continue
        station_id = str(station_id)
        if station_id not in stations:
            stations[station_id] = {
                "vehicle_scope": val(row, "vehicle_scope"),
                "lat": val(row, "charger_lat"),
                "lon": val(row, "charger_lon"),
                "capacity": val(row, "capacity"),
                "l1": val(row, "level_1_ports") or 0,
                "l2": val(row, "level_2_ports") or 0,
                "l3": val(row, "level_3_ports") or 0,
                "power_kw_max": val(row, "power_kw_max"),
                "network": val(row, "network"),
            }
            station_order.append(station_id)
        else:
            first = stations[station_id]
            for key, name in (
                    ("lat", "charger_lat"), ("lon", "charger_lon"),
                    ("capacity", "capacity"),
                    ("l1", "level_1_ports"),
                    ("l2", "level_2_ports"),
                    ("l3", "level_3_ports"),
                    ("power_kw_max", "power_kw_max")):
                later = val(row, name) or 0 \
                    if name.startswith("level_") else val(row, name)
                if later != first[key] and not (
                        later is None and first[key] is None):
                    inconsistent.append(station_id)
                    break
        poi_osm_id = val(row, "poi_osm_id")
        if poi_osm_id is None:
            n_null_poi += 1
            continue
        poi_rows.append({
            "poi_osm_id": str(poi_osm_id),
            "poi_lat": val(row, "poi_lat"),
            "poi_lon": val(row, "poi_lon"),
            "poi_category": val(row, "poi_category"),
            "poi_type": val(row, "poi_type"),
            "charger_id": station_id,
            "walk_dist_m": val(row, "walk_dist_m"),
            "straight_dist_m": val(row, "straight_dist_m"),
            "parking_fee": val(row, "parking_fee"),
            "network": val(row, "network"),
            "price": val(row, "price"),
        })
    if inconsistent:
        sys.stderr.write(
            "warning: %d stations have inconsistent station-level "
            "fields across rows (first row kept): %s\n"
            % (len(set(inconsistent)),
               ", ".join(sorted(set(inconsistent))[:10])))

    # Pass 2: attribute filters, then the snap.
    dropped_scope = []
    dropped_no_power = []
    dropped_no_ports = []
    dropped_no_location = []
    kept = []
    for station_id in station_order:
        st = stations[station_id]
        if st["vehicle_scope"] != "car":
            dropped_scope.append(station_id)
        elif st["power_kw_max"] is None:
            dropped_no_power.append(station_id)
        elif not (st["l1"] or st["l2"] or st["l3"]):
            dropped_no_ports.append(station_id)
        elif st["lat"] is None or st["lon"] is None:
            dropped_no_location.append(station_id)
        else:
            kept.append(station_id)
    dropped = set(dropped_scope) | set(dropped_no_power) | \
        set(dropped_no_ports) | set(dropped_no_location)

    kept_x, kept_y = project(
        [stations[s]["lon"] for s in kept],
        [stations[s]["lat"] for s in kept])
    snap_dist, snap_idx = tree.query(list(zip(kept_x, kept_y)))
    max_snap_m = args.max_snap_km * 1000.0
    snap = {}
    dropped_snap = []
    for station_id, dist, idx in zip(kept, snap_dist, snap_idx):
        snap[station_id] = (float(dist), node_ids[int(idx)])
        if dist > max_snap_m:
            dropped_snap.append(station_id)
    dropped |= set(dropped_snap)
    final = [s for s in kept if s not in dropped_snap]

    # Charger rows: one per power level per kept station.
    charger_rows = []
    n_l1 = n_l2 = n_l3 = 0
    n_fast = n_slow = 0
    for station_id in final:
        st = stations[station_id]
        node_id = snap[station_id][1]
        levels = []
        if st["l1"] and not (st["l2"] or st["l3"]):
            levels.append(("L1", float(st["power_kw_max"]),
                           int(st["l1"])))
            n_l1 += 1
        if st["l2"]:
            levels.append(("L2", float(args.level2_kw),
                           int(st["l2"])))
            n_l2 += 1
        if st["l3"]:
            levels.append(("L3", float(st["power_kw_max"]),
                           int(st["l3"])))
            n_l3 += 1
        for level, power, plugs in levels:
            if len(levels) == 1:
                charger_id = station_id
            else:
                charger_id = "%s_%s" % (station_id, level)
            kind = "fast" if power >= FAST_THRESHOLD_KW else "slow"
            if kind == "fast":
                n_fast += 1
            else:
                n_slow += 1
            charger_rows.append({
                "charger_id": charger_id,
                "node_id": node_id,
                "power_kw": power,
                "plugs": plugs,
                "kind": kind,
            })
    charger_rows.sort(key=lambda r: (r["node_id"], r["charger_id"]))

    # POI rows: kept stations only, POI snapped to its nearest
    # node. A POI without coordinates cannot be snapped and is
    # dropped from the POI file (counted below).
    poi_kept = [p for p in poi_rows if p["charger_id"] not in dropped]
    # poi_rows already excludes the null-POI rows, so the rows
    # missing here are exactly the ones for dropped stations.
    poi_dropped_station = len(poi_rows) - len(poi_kept)
    snap_poi = [p for p in poi_kept
                if p["poi_lat"] is not None
                and p["poi_lon"] is not None]
    n_poi_no_location = len(poi_kept) - len(snap_poi)
    poi_x, poi_y = project(
        [p["poi_lon"] for p in snap_poi],
        [p["poi_lat"] for p in snap_poi])
    poi_dist, poi_idx = tree.query(list(zip(poi_x, poi_y)))
    poi_out = []
    for p, _dist, idx in zip(snap_poi, poi_dist, poi_idx):
        poi_out.append({
            "poi_osm_id": p["poi_osm_id"],
            "poi_node_id": node_ids[int(idx)],
            "poi_category": fmt_str(p["poi_category"]),
            "poi_type": fmt_str(p["poi_type"]),
            "charger_id": p["charger_id"],
            "walk_dist_m": fmt_num(p["walk_dist_m"]),
            "straight_dist_m": fmt_num(p["straight_dist_m"]),
            "parking_fee": fmt_str(p["parking_fee"]),
            "network": fmt_str(p["network"]),
            "price": fmt_num(p["price"])
                     if isinstance(p["price"], (int, float))
                     else fmt_str(p["price"]),
        })
    poi_out.sort(key=lambda r: (r["poi_node_id"], r["charger_id"],
                                r["poi_osm_id"]))

    os.makedirs(args.out_dir, exist_ok=True)
    charger_path = os.path.join(args.out_dir, "input_charger.csv")
    with open(charger_path, "w", newline="") as fh:
        writer = csv.writer(fh, lineterminator="\n")
        writer.writerow(["charger_id", "node_id", "power_kw",
                         "plugs", "kind"])
        for r in charger_rows:
            writer.writerow([r["charger_id"], r["node_id"],
                             fmt_num(r["power_kw"]), r["plugs"],
                             r["kind"]])
    poi_path = os.path.join(args.out_dir, "input_poi_charger.csv")
    with open(poi_path, "w", newline="") as fh:
        writer = csv.writer(fh, lineterminator="\n")
        writer.writerow(["poi_osm_id", "poi_node_id", "poi_category",
                         "poi_type", "charger_id", "walk_dist_m",
                         "straight_dist_m", "parking_fee", "network",
                         "price"])
        for r in poi_out:
            writer.writerow([r["poi_osm_id"], r["poi_node_id"],
                             r["poi_category"], r["poi_type"],
                             r["charger_id"], r["walk_dist_m"],
                             r["straight_dist_m"], r["parking_fee"],
                             r["network"], r["price"]])

    # Report.
    def band_counts(distances):
        bands = [0] * (len(SNAP_BANDS_M) + 1)
        for d in distances:
            placed = False
            for i, edge in enumerate(SNAP_BANDS_M):
                if d <= edge:
                    bands[i] += 1
                    placed = True
                    break
            if not placed:
                bands[-1] += 1
        return bands

    labels = ["<=100 m", "100-500 m", "500 m-1 km", "1-2 km",
              "2-5 km", ">5 km"]
    snap_bands = band_counts([snap[s][0] for s in kept])
    poi_bands = band_counts([float(d) for d in poi_dist])
    cap_mismatch = [s for s in final
                    if stations[s]["capacity"] is not None
                    and stations[s]["capacity"] !=
                    stations[s]["l1"] + stations[s]["l2"]
                    + stations[s]["l3"]]
    big = [s for s in final
           if stations[s]["capacity"] is not None
           and stations[s]["capacity"] >= 100]
    print("build_chargers_real.py: %s + %s" % (args.xlsx, args.nodes))
    print("input rows: %d, stations: %d" % (n_rows, len(stations)))
    print("stations dropped: scope != car: %d %s"
          % (len(dropped_scope), dropped_scope))
    print("stations dropped: no power info: %d %s"
          % (len(dropped_no_power), dropped_no_power))
    print("stations dropped: no ports: %d %s"
          % (len(dropped_no_ports), dropped_no_ports))
    print("stations dropped: no location: %d %s"
          % (len(dropped_no_location), dropped_no_location))
    print("stations dropped: snap > %.1f km: %d"
          % (args.max_snap_km, len(dropped_snap)))
    print("stations kept: %d" % len(final))
    print("charger snap-distance distribution "
          "(attribute-kept stations):")
    for label, count in zip(labels, snap_bands):
        print("  %s: %d" % (label, count))
    print("charger rows: %d (L1: %d, L2: %d at %.1f kW, L3: %d; "
          "fast: %d, slow: %d)"
          % (len(charger_rows), n_l1, n_l2, args.level2_kw, n_l3,
             n_fast, n_slow))
    print("POI rows in file: %d (null-POI rows dropped: %d; "
          "rows for dropped stations: %d; POI without location: %d)"
          % (len(poi_out), n_null_poi, poi_dropped_station,
             n_poi_no_location))
    print("POI snap-distance distribution (kept POI rows):")
    for label, count in zip(labels, poi_bands):
        print("  %s: %d" % (label, count))
    print("outliers: capacity >= 100 ports: %s"
          % ["%s (%d ports)" % (s, stations[s]["capacity"])
             for s in big])
    print("outliers: capacity != sum of ports: %d %s"
          % (len(cap_mismatch),
             ["%s (capacity %s, L1 %d + L2 %d + L3 %d)"
              % (s, fmt_num(stations[s]["capacity"]),
                 stations[s]["l1"], stations[s]["l2"],
                 stations[s]["l3"]) for s in cap_mismatch]))
    print("wrote %s (%d rows)" % (charger_path, len(charger_rows)))
    print("wrote %s (%d rows)" % (poi_path, len(poi_out)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
