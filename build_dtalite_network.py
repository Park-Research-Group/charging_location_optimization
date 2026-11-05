# build_dtalite_network.py -- convert the Swedish road network
# shapefile (Zenodo 7549847 file 5) into DTALite-format inputs for the
# trimmed PySim: input_node.csv, input_link.csv, plus input_link_type.csv
# and input_zone.csv (zones = nodes), a link_attributes.csv sidecar
# (slope, per-day count, original ids, unconverted units), and a
# node_lookup.csv (new integer id -> original node string) for later
# agent snapping.
#
# Units, verified 2026-11-05 against the data (the Zenodo doc labels
# freespeed "km/h" but the values are m/s: max 33.333 = 120 km/h):
#   length: meters -> miles (/ 1609.34)
#   freespeed: m/s -> mph (* 2.23694)
#   capacity: vehicles/hour for the whole link -> lane_cap =
#       capacity / permlanes
#   oneway 1 -> direction 1; anything else -> direction 2 (the loader
#       builds the reverse link for direction 2)
# Coordinates stay SWEREF99TM meters: the activity plans use the same
# CRS, so agent snapping later must use these coordinates, not lat/lon.
# jam_density (180) and wave_speed (12) are not in the data; they are
# the fixture defaults.
#
# Usage (from anywhere; paths resolve from this script's folder):
#   python build_dtalite_network.py [--shp PATH] [--out DIR]
#       [--bbox xmin,ymin,xmax,ymax]
# Defaults: --shp auto-finds input_files/*road_network_with_slope.shp;
# --out is dtalite_inputs/ next to this script. --bbox keeps only
# links whose midpoint falls inside the box (SWEREF99TM meters),
# for a spatial subset; nodes follow the kept links.
import argparse
import csv
import sys
from pathlib import Path

import shapefile

METER_TO_MILE = 1.0 / 1609.34
MPS_TO_MPH = 2.23694
JAM_DENSITY = 180
WAVE_SPEED = 12
COORD_TOL = 0.01  # meters; same node must agree across links


def find_shapefile(here, given):
    if given:
        return Path(given)
    matches = sorted(here.glob("old_data/*road_network_with_slope.shp"))
    if not matches:
        raise SystemExit(
            "no *road_network_with_slope.shp under input_files/; "
            "pass --shp PATH")
    if len(matches) > 1:
        print("multiple shapefiles, using", matches[0])
    return matches[0]


def parse_bbox(text):
    if not text:
        return None
    parts = [float(v) for v in text.split(",")]
    if len(parts) != 4:
        raise SystemExit("--bbox needs xmin,ymin,xmax,ymax")
    return parts


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--shp", default=None,
                    help="shapefile path (default: auto-find under "
                    "input_files/)")
    ap.add_argument("--out", default=None,
                    help="output dir (default: dtalite_inputs/ next to "
                    "this script)")
    ap.add_argument("--bbox", default=None,
                    help="xmin,ymin,xmax,ymax in SWEREF99TM meters; "
                    "keep links whose midpoint is inside")
    args = ap.parse_args()
    shp_path = find_shapefile(here, args.shp)
    out_dir = Path(args.out) if args.out else here / "dtalite_inputs"
    out_dir.mkdir(parents=True, exist_ok=True)
    bbox = parse_bbox(args.bbox)
    print("reading", shp_path)
    if bbox:
        print("bbox filter (midpoint inside):", bbox)

    sf = shapefile.Reader(str(shp_path))
    fields = [f[0] for f in sf.fields[1:]]
    idx = {name: i for i, name in enumerate(fields)}
    need = ("length", "freespeed", "capacity", "permlanes", "oneway",
            "modes", "link_id", "from_node", "to_node", "count", "slope")
    missing = [c for c in need if c not in idx]
    if missing:
        raise SystemExit("shapefile is missing columns: %s" % missing)

    node_id = {}    # original node string -> new integer id
    node_xy = {}    # new integer id -> (x, y)
    seen_pairs = set()
    counts = {"links": 0, "skipped_mode": 0, "skipped_speed": 0,
              "skipped_dup": 0, "skipped_bbox": 0, "oneway_1": 0,
              "oneway_0": 0, "coord_mismatch": 0, "lanes_fixed": 0}
    speeds_mph = []
    total_miles = 0.0

    def node_number(node_str, x, y):
        nid = node_id.get(node_str)
        if nid is None:
            nid = len(node_id) + 1
            node_id[node_str] = nid
            node_xy[nid] = (x, y)
            return nid
        ox, oy = node_xy[nid]
        if abs(ox - x) > COORD_TOL or abs(oy - y) > COORD_TOL:
            counts["coord_mismatch"] += 1
        return nid

    link_path = out_dir / "input_link.csv"
    attr_path = out_dir / "link_attributes.csv"
    with open(link_path, "w", newline="", encoding="utf-8") as lf, \
            open(attr_path, "w", newline="", encoding="utf-8") as af:
        link_w = csv.writer(lf, lineterminator="\r\n")
        link_w.writerow(["from_node_id", "to_node_id", "link_type",
                         "direction", "length", "number_of_lanes",
                         "speed_limit", "lane_cap", "jam_density",
                         "wave_speed"])
        attr_w = csv.writer(af, lineterminator="\r\n")
        attr_w.writerow(["link_id", "from_node_id", "to_node_id",
                         "from_node", "to_node", "length_m",
                         "freespeed_mps", "capacity", "permlanes",
                         "oneway", "count", "slope"])
        for n, rec in enumerate(sf.iterShapeRecords(), 1):
            if n % 100000 == 0:
                print("... %d links read" % n)
            r = rec.record
            row = {name: r[idx[name]] for name in need}
            if "car" not in str(row["modes"]):
                counts["skipped_mode"] += 1
                continue
            speed_mps = float(row["freespeed"])
            if speed_mps <= 0:
                counts["skipped_speed"] += 1
                continue
            pts = rec.shape.points
            if len(pts) < 2:
                counts["skipped_speed"] += 1
                continue
            if bbox:
                mx = (pts[0][0] + pts[-1][0]) / 2.0
                my = (pts[0][1] + pts[-1][1]) / 2.0
                if not (bbox[0] <= mx <= bbox[2]
                        and bbox[1] <= my <= bbox[3]):
                    counts["skipped_bbox"] += 1
                    continue
            from_n = node_number(str(row["from_node"]),
                                 pts[0][0], pts[0][1])
            to_n = node_number(str(row["to_node"]),
                               pts[-1][0], pts[-1][1])
            pair = (from_n, to_n)
            if pair in seen_pairs:
                # DTALite looks links up by (from, to); a duplicate
                # pair would silently shadow the first link.
                counts["skipped_dup"] += 1
                continue
            seen_pairs.add(pair)
            length_m = float(row["length"])
            lanes = int(row["permlanes"])
            if lanes < 1:
                lanes = 1
                counts["lanes_fixed"] += 1
            capacity = float(row["capacity"])
            lane_cap = int(round(capacity / lanes)) if capacity > 0 \
                else 1800
            oneway = int(row["oneway"])
            direction = 1 if oneway == 1 else 2
            counts["oneway_1" if direction == 1 else "oneway_0"] += 1
            speed_mph = speed_mps * MPS_TO_MPH
            length_mi = length_m * METER_TO_MILE
            speeds_mph.append(speed_mph)
            total_miles += length_mi
            link_w.writerow([from_n, to_n, 1, direction,
                             round(length_mi, 6), lanes,
                             round(speed_mph, 2), lane_cap,
                             JAM_DENSITY, WAVE_SPEED])
            attr_w.writerow([row["link_id"], from_n, to_n,
                             row["from_node"], row["to_node"],
                             length_m, speed_mps, row["capacity"],
                             row["permlanes"], oneway, row["count"],
                             row["slope"]])
            counts["links"] += 1

    with open(out_dir / "input_node.csv", "w", newline="",
              encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\r\n")
        w.writerow(["node_id", "x", "y"])
        for nid in sorted(node_xy):
            x, y = node_xy[nid]
            w.writerow([nid, round(x, 2), round(y, 2)])
    with open(out_dir / "node_lookup.csv", "w", newline="",
              encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\r\n")
        w.writerow(["node_id", "node_string"])
        for node_str, nid in sorted(node_id.items(), key=lambda kv: kv[1]):
            w.writerow([nid, node_str])
    with open(out_dir / "input_zone.csv", "w", newline="",
              encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\r\n")
        w.writerow(["zone_id"])
        for nid in sorted(node_xy):
            w.writerow([nid])
    with open(out_dir / "input_link_type.csv", "w", newline="",
              encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\r\n")
        w.writerow(["link_type", "link_type_name", "type_code"])
        w.writerow([1, "Freeway", "f"])

    print("wrote %s" % out_dir)
    print("links kept: %d, nodes: %d, total length: %.1f miles"
          % (counts["links"], len(node_xy), total_miles))
    print("oneway=1 (direction 1): %d, oneway=0 (direction 2): %d"
          % (counts["oneway_1"], counts["oneway_0"]))
    print("skipped: non-car %d, bad speed/geometry %d, duplicate "
          "(from,to) pairs %d, outside bbox %d"
          % (counts["skipped_mode"], counts["skipped_speed"],
             counts["skipped_dup"], counts["skipped_bbox"]))
    print("node coordinate mismatches: %d, lanes fixed to 1: %d"
          % (counts["coord_mismatch"], counts["lanes_fixed"]))
    if speeds_mph:
        print("speed mph: min %.1f, max %.1f"
              % (min(speeds_mph), max(speeds_mph)))
    if counts["skipped_dup"]:
        print("WARNING: duplicate (from,to) pairs were dropped; check "
              "whether parallel links need separate handling.")
    if counts["coord_mismatch"]:
        print("WARNING: some nodes disagree on coordinates across "
              "links; first-seen coordinates were kept.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
