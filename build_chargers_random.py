# build_chargers_random.py -- starter input_charger.csv for the trimmed
# PySim charging layer: pick random nodes and put chargers on them.
#
# Placement is uniformly random over nodes (seeded, so re-running with
# the same --seed reproduces the same file), with a realistic tier mix:
# most sites are slow AC (11/22 kW) and a minority are fast DC
# (50/150 kW). Plugs per site default small (fast sites get 1 plug) so
# queues form and the charging-delay machinery actually gets exercised
# in a smoke test. This is a placeholder for real siting -- it says
# nothing about where chargers should go; the feasibility survey and
# the siting optimizer replace it later.
#
# Usage (from the repo root, after build_dtalite_network.py):
#   python build_chargers_random.py [--nodes dtalite_inputs/input_node.csv]
#                                   [--out input_charger.csv]
#                                   [--n-sites 200] [--seed 42]
#                                   [--mix 11:0.45,22:0.25,50:0.18,150:0.12]
#                                   [--plugs 11:2,22:2,50:2,150:1]
#
# Reads input_node.csv (node_id,x,y) from build_dtalite_network.py.
# Writes input_charger.csv (charger_id,node_id,power_kw,plugs,kind),
# one site per node, kind "fast" for >= 50 kW else "slow" (the same
# convention as the v3 tests). Stdlib only.

import argparse
import csv
import random
import sys
from pathlib import Path

DEFAULT_MIX = "11:0.45,22:0.25,50:0.18,150:0.12"
DEFAULT_PLUGS = "11:2,22:2,50:2,150:1"


def parse_mix(spec):
    tiers = []
    total = 0.0
    for part in spec.split(","):
        try:
            power, share = part.split(":")
            power = int(power)
            share = float(share)
        except ValueError:
            raise SystemExit("bad --mix part %r, want POWER:SHARE" % part)
        if power <= 0 or share <= 0:
            raise SystemExit("bad --mix part %r: power and share must "
                             "be positive" % part)
        tiers.append((power, share))
        total += share
    if not tiers or abs(total - 1.0) > 0.001:
        raise SystemExit("--mix shares must sum to 1, got %.4f" % total)
    return tiers


def parse_plugs(spec):
    plugs = {}
    for part in spec.split(","):
        try:
            power, count = part.split(":")
            plugs[int(power)] = int(count)
        except ValueError:
            raise SystemExit("bad --plugs part %r, want POWER:COUNT" % part)
    for power, count in plugs.items():
        if count < 1:
            raise SystemExit("bad --plugs part %d:%d: at least 1 plug"
                             % (power, count))
    return plugs


def find_nodes(here, given):
    if given:
        path = Path(given)
        if not path.is_file():
            raise SystemExit("--nodes file not found: %s" % path)
        return path
    for candidate in (here / "dtalite_inputs" / "input_node.csv",
                      here / "input_node.csv"):
        if candidate.is_file():
            return candidate
    raise SystemExit("input_node.csv not found (looked in dtalite_inputs/ "
                     "next to this script); pass --nodes PATH")


def read_node_ids(path):
    ids = set()
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        column = "node_id" if reader.fieldnames and \
            "node_id" in reader.fieldnames else reader.fieldnames[0]
        for row in reader:
            try:
                ids.add(int(row[column]))
            except (TypeError, ValueError):
                continue
    if not ids:
        raise SystemExit("no node ids read from %s" % path)
    return sorted(ids)


def pick_tier(tiers, rng):
    draw = rng.random()
    running = 0.0
    for power, share in tiers:
        running += share
        if draw < running:
            return power
    return tiers[-1][0]


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--nodes", default=None,
                    help="input_node.csv from build_dtalite_network.py "
                    "(default: dtalite_inputs/input_node.csv next to "
                    "this script)")
    ap.add_argument("--out", default=None,
                    help="output csv (default: input_charger.csv next to "
                    "this script)")
    ap.add_argument("--n-sites", type=int, default=200,
                    help="chargers to place (default 200)")
    ap.add_argument("--seed", type=int, default=42,
                    help="random seed; same seed gives the same file "
                    "(default 42)")
    ap.add_argument("--mix", default=DEFAULT_MIX,
                    help="tier shares as POWER:SHARE,... (default "
                    "%s)" % DEFAULT_MIX)
    ap.add_argument("--plugs", default=DEFAULT_PLUGS,
                    help="plugs per site as POWER:COUNT,... (default "
                    "%s)" % DEFAULT_PLUGS)
    args = ap.parse_args()
    if args.n_sites < 1:
        raise SystemExit("--n-sites must be >= 1")
    tiers = parse_mix(args.mix)
    plugs = parse_plugs(args.plugs)
    nodes_path = find_nodes(here, args.nodes)
    node_ids = read_node_ids(nodes_path)
    print("nodes available: %d (%s)" % (len(node_ids), nodes_path))
    if args.n_sites > len(node_ids):
        raise SystemExit("--n-sites %d exceeds nodes available %d"
                         % (args.n_sites, len(node_ids)))

    rng = random.Random(args.seed)
    chosen = rng.sample(node_ids, args.n_sites)
    out_path = Path(args.out) if args.out else here / "input_charger.csv"
    counts = {}
    total_plugs = 0
    total_kw = 0
    with open(out_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh, lineterminator="\r\n")
        writer.writerow(["charger_id", "node_id", "power_kw", "plugs",
                         "kind"])
        for i, node_id in enumerate(chosen, start=1):
            power = pick_tier(tiers, rng)
            plug_count = plugs.get(power, 2)
            kind = "fast" if power >= 50 else "slow"
            writer.writerow(["CHG%05d" % i, node_id, power, plug_count,
                             kind])
            counts[power] = counts.get(power, 0) + 1
            total_plugs += plug_count
            total_kw += power * plug_count
    print("wrote %s: %d sites, seed %d"
          % (out_path, args.n_sites, args.seed))
    for power in sorted(counts):
        print("  %d kW (%s): %d sites"
              % (power, "fast" if power >= 50 else "slow", counts[power]))
    print("  total plugs: %d, total nameplate: %d kW"
          % (total_plugs, total_kw))


if __name__ == "__main__":
    main()
