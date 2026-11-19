# add_slope_to_links.py -- add a `slope` column to an existing
# input_link.csv (v13).
#
# The sim (v13) reads a per-link slope in FRACTION units (0.03 =
# 3%) from input_link.csv and looks the energy rate up in the
# paper's speed/slope maps. Old inputs have no slope column and
# still run -- they just fall back to the flat rate per link -- so
# this script is only needed when you want the lookup on existing
# inputs.
#
# The join is by (from_node_id, to_node_id) against
# link_attributes.csv, the sidecar the network converter
# (tools/build_dtalite_network_v2.py, and v3) writes next to
# input_link.csv. That file already carries the Zenodo shapefile's
# `slope` per link, in the same row order as input_link.csv and
# with matching from_node_id/to_node_id, so no shapefile is needed
# for the normal path. Row order is also checked as a cross-check:
# a mismatch warns but does not stop the join (the key join is
# authoritative).
#
# If link_attributes.csv is absent, --shp reads the slope straight
# from the shapefile (needs the `shapefile` / pyshp package, the
# same one the converter uses) plus --node-lookup node_lookup.csv
# to map the shapefile's original node strings to the integer ids
# in input_link.csv. Do NOT download the 52 MB shapefile just for
# this -- the link_attributes.csv path is the intended one.
#
# The slope is copied as-is (fraction, NOT percent, NOT negated):
# the sim negates it for the reverse direction of a two-way row
# itself, and clips it to +/-0.06 at lookup time. Rows with no
# slope match keep a blank slope (the sim falls back to the flat
# rate for those links) and are counted in the report.
#
# Usage:
#   python add_slope_to_links.py input_link.csv
#       [--attributes link_attributes.csv] [--out out.csv | --in-place]
#   python add_slope_to_links.py input_link.csv --shp road_network_with_slope.shp
#       --node-lookup node_lookup.csv [--out out.csv | --in-place]
# Defaults: --attributes is link_attributes.csv next to the input;
# --out defaults to <input>_with_slope.csv next to the input.
# --in-place replaces the input (a .bak copy is kept).
import argparse
import csv
import sys
from pathlib import Path


def read_attributes(path):
    # {(from_node_id, to_node_id): slope} plus the row order, for
    # the cross-check. A blank/NaN slope in the sidecar means "no
    # slope known" and joins to a blank slope, not to zero.
    by_key = {}
    order = []
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        missing = [c for c in ("from_node_id", "to_node_id", "slope")
                   if c not in (reader.fieldnames or [])]
        if missing:
            raise SystemExit(
                "%s is missing columns: %s" % (path, missing))
        for row in reader:
            try:
                key = (int(row["from_node_id"]), int(row["to_node_id"]))
            except (TypeError, ValueError):
                continue
            text = (row["slope"] or "").strip()
            slope = None
            if text:
                try:
                    slope = float(text)
                except ValueError:
                    slope = None
                if slope is not None and slope != slope:  # NaN
                    slope = None
            by_key[key] = slope
            order.append(key)
    return by_key, order


def read_shapefile_slopes(shp_path, node_lookup_path):
    # {(int from, int to): slope} from the shapefile. The
    # shapefile's from_node/to_node are the ORIGINAL node strings;
    # node_lookup.csv (written by the converter) maps them to the
    # integer ids input_link.csv uses.
    try:
        import shapefile
    except ImportError:
        raise SystemExit(
            "the shapefile fallback needs the `shapefile` (pyshp) "
            "package, as the converter uses it; install it or use "
            "--attributes link_attributes.csv instead")
    lookup = {}
    with open(node_lookup_path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            lookup[str(row["node_string"])] = int(row["node_id"])
    sf = shapefile.Reader(str(shp_path))
    fields = [f[0] for f in sf.fields[1:]]
    idx = {name: i for i, name in enumerate(fields)}
    missing = [c for c in ("from_node", "to_node", "slope")
               if c not in idx]
    if missing:
        raise SystemExit(
            "shapefile is missing columns: %s" % missing)
    by_key = {}
    for rec in sf.iterRecords():
        try:
            frm = lookup[str(rec[idx["from_node"]])]
            to = lookup[str(rec[idx["to_node"]])]
        except KeyError:
            continue
        try:
            slope = float(rec[idx["slope"]])
        except (TypeError, ValueError):
            continue
        if slope != slope:  # NaN
            continue
        by_key[(frm, to)] = slope
    return by_key


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input_link", help="input_link.csv to add slope to")
    ap.add_argument("--attributes", default=None,
                    help="link_attributes.csv (default: same folder as "
                    "the input)")
    ap.add_argument("--shp", default=None,
                    help="shapefile fallback when no attributes file "
                    "is available")
    ap.add_argument("--node-lookup", default=None,
                    help="node_lookup.csv for the --shp fallback "
                    "(default: same folder as the input)")
    ap.add_argument("--out", default=None,
                    help="output CSV (default: <input>_with_slope.csv)")
    ap.add_argument("--in-place", action="store_true",
                    help="replace the input file (keeps a .bak copy)")
    args = ap.parse_args(argv)
    in_path = Path(args.input_link)
    if not in_path.exists():
        raise SystemExit("no such file: %s" % in_path)
    if args.attributes is not None:
        attr_path = Path(args.attributes)
        if not attr_path.exists():
            raise SystemExit("no such file: %s" % attr_path)
        by_key, attr_order = read_attributes(attr_path)
        source = "attributes %s" % attr_path
    elif args.shp is not None:
        lookup_path = Path(args.node_lookup) if args.node_lookup \
            else in_path.parent / "node_lookup.csv"
        if not lookup_path.exists():
            raise SystemExit(
                "the --shp fallback needs node_lookup.csv "
                "(%s not found)" % lookup_path)
        by_key = read_shapefile_slopes(args.shp, lookup_path)
        attr_order = None
        source = "shapefile %s" % args.shp
    else:
        attr_path = in_path.parent / "link_attributes.csv"
        if not attr_path.exists():
            raise SystemExit(
                "no link_attributes.csv next to %s; pass "
                "--attributes PATH or --shp PATH (with "
                "--node-lookup)" % in_path)
        by_key, attr_order = read_attributes(attr_path)
        source = "attributes %s" % attr_path
    print("slope source:", source)
    print("slope entries:", len(by_key))

    with open(in_path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        fieldnames = list(reader.fieldnames or [])
        if not fieldnames:
            raise SystemExit("%s has no header" % in_path)
        rows = list(reader)
    has_slope = "slope" in fieldnames
    if has_slope:
        print("WARNING: %s already has a slope column; values will "
              "be overwritten" % in_path)
    else:
        fieldnames = fieldnames + ["slope"]
    if attr_order is not None and len(attr_order) == len(rows):
        mismatches = sum(
            1 for row, key in zip(rows, attr_order)
            if (int(row["from_node_id"]), int(row["to_node_id"])) != key)
        if mismatches:
            print("WARNING: row order differs from link_attributes.csv "
                  "on %d of %d rows; joining by (from_node_id, "
                  "to_node_id) instead" % (mismatches, len(rows)))
        else:
            print("row order matches link_attributes.csv "
                  "(%d rows)" % len(rows))
    elif attr_order is not None:
        print("NOTE: link_attributes.csv has %d rows vs %d input "
              "rows; joining by (from_node_id, to_node_id)"
              % (len(attr_order), len(rows)))
    counts = {"filled": 0, "blank": 0, "out_of_range": 0}
    for row in rows:
        key = (int(row["from_node_id"]), int(row["to_node_id"]))
        slope = by_key.get(key)
        if slope is None:
            row["slope"] = ""
            counts["blank"] += 1
        else:
            row["slope"] = repr(float(slope))
            counts["filled"] += 1
            if abs(slope) > 0.06:
                # The sim clips to +/-0.06 at lookup time; the raw
                # value is kept here so the file stays faithful
                # to the shapefile.
                counts["out_of_range"] += 1
    if args.in_place:
        out_path = in_path
        bak_path = in_path.with_suffix(in_path.suffix + ".bak")
        bak_path.write_bytes(in_path.read_bytes())
        print("backup:", bak_path)
    elif args.out:
        out_path = Path(args.out)
    else:
        out_path = in_path.with_name(
            in_path.stem + "_with_slope.csv")
    with open(out_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames,
                                lineterminator="\r\n")
        writer.writeheader()
        writer.writerows(rows)
    print("wrote %s (%d rows: %d with slope, %d blank; %d slopes "
          "outside +/-0.06, clipped at lookup)"
          % (out_path, len(rows), counts["filled"], counts["blank"],
             counts["out_of_range"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
