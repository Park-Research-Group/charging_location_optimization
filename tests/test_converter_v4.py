# Converter v4 checks (v5, design 12.4).
#
# Runs build_dtalite_agents_v4.py (and, for backward compatibility,
# v3) as a subprocess on synthetic mini files under /tmp -- never
# Eli's real data. Each person has the same three-activity day
# (home -> work -> home) snapped onto four nodes, so every agent
# gets exactly two legs per day with --days 1. The checks: same
# seed twice is byte-identical; the archetype mix matches the
# request; thresholds/tolerances stay in their ranges and are
# uniform across each agent's legs; omitted flags equal the
# documented defaults; v4 matches v3 on every shared column except
# archetype (v3 has no threshold/tolerance columns); bad flags
# fail loud.
import csv
import os
import shutil
import subprocess
import sys
from collections import Counter

HERE = os.path.dirname(__file__)
CONVERTER_V4 = os.path.normpath(os.path.join(
    HERE, "..", "..", "your_files", "build_dtalite_agents_v4.py"))
CONVERTER_V3 = os.path.normpath(os.path.join(
    HERE, "..", "..", "your_files", "build_dtalite_agents_v3.py"))

BASE = "/tmp/conv_case"
N_SMALL = 500
N_MIX = 40000


def write_mini_inputs(base, n_persons):
    if os.path.exists(base):
        shutil.rmtree(base)
    os.makedirs(base)
    with open(os.path.join(base, "input_node.csv"), "w",
              newline="") as fh:
        fh.write("node_id,x,y\n1,0,0\n2,1000,0\n3,0,1000\n4,1000,1000\n")
    with open(os.path.join(base, "plans.csv"), "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["person", "act_id", "POINT_X", "POINT_Y",
                         "act_purpose", "dep_time", "act_time", "mode"])
        for person in range(1, n_persons + 1):
            writer.writerow([person, 0, 0, 0, "home", 8.0, 480, "car"])
            writer.writerow([person, 1, 1000, 0, "work", 17.5, 540,
                             "car"])
            writer.writerow([person, 2, 0, 0, "home", 8.0, 0, "car"])
    with open(os.path.join(base, "home.csv"), "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["person", "home_charger"])
        for person in range(1, n_persons + 1):
            writer.writerow([person, person % 2])
    with open(os.path.join(base, "fleet.csv"), "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["person", "car"])
        for person in range(1, n_persons + 1):
            writer.writerow([person, "BCD"[person % 3]])


def run_converter(script, out_dir, args):
    assert os.path.exists(script), "converter not found: %s" % script
    cmd = [sys.executable, script,
           "--plans", os.path.join(BASE, "plans.csv"),
           "--home", os.path.join(BASE, "home.csv"),
           "--fleet", os.path.join(BASE, "fleet.csv"),
           "--nodes", os.path.join(BASE, "input_node.csv"),
           "--out", out_dir, "--days", "1"] + args
    return subprocess.run(cmd, capture_output=True, text=True,
                          timeout=600)


def run_converter_ok(script, out_dir, args):
    proc = run_converter(script, out_dir, args)
    assert proc.returncode == 0, (proc.returncode, proc.stdout[-500:],
                                  proc.stderr[-500:])
    return proc


def read_agent(path):
    with open(path, newline="") as fh:
        rows = [r for r in csv.reader(fh) if r]
    header = [h.strip() for h in rows[0]]
    return header, [dict(zip(header, r)) for r in rows[1:]]


def agent_archetypes(rows):
    seen = {}
    for row in rows:
        seen.setdefault(row["agent_id"], row["archetype"])
    return seen


def close(actual, expected, tol=0.05):
    assert abs(float(actual) - expected) <= tol, (actual, expected)


def test_reproducibility():
    # Same inputs + --seed 42 twice: byte-identical agent file and
    # vehicle-type file.
    write_mini_inputs(BASE, N_SMALL)
    run_converter_ok(CONVERTER_V4, "/tmp/conv_out_rep_a", ["--seed", "42"])
    run_converter_ok(CONVERTER_V4, "/tmp/conv_out_rep_b", ["--seed", "42"])
    for name in ("input_agent.csv", "optional_vehicle_type.csv"):
        with open(os.path.join("/tmp/conv_out_rep_a", name), "rb") as fh:
            first = fh.read()
        with open(os.path.join("/tmp/conv_out_rep_b", name), "rb") as fh:
            second = fh.read()
        assert first == second, "%s differs between seed-42 runs" % name
    print("reproducibility: seed 42 twice, agent + vehicle-type "
          "files byte-identical")


def test_mix():
    # 40,000 persons, default (even-thirds) mix: each archetype
    # share within 1 percentage point of 1/3.
    write_mini_inputs(BASE, N_MIX)
    run_converter_ok(CONVERTER_V4, "/tmp/conv_out_mix", [])
    _h, rows = read_agent("/tmp/conv_out_mix/input_agent.csv")
    counts = Counter(agent_archetypes(rows).values())
    total = sum(counts.values())
    assert total == N_MIX, total
    shares = {}
    for name in ("liquid", "plan_ahead", "opportunistic"):
        shares[name] = counts[name] / total
        assert abs(shares[name] - 1.0 / 3) <= 0.01, (name, shares[name])
    print("mix: %d agents, liquid %.3f / plan_ahead %.3f / "
          "opportunistic %.3f (each within 0.01 of 1/3)"
          % (total, shares["liquid"], shares["plan_ahead"],
             shares["opportunistic"]))


def test_mix_all_liquid():
    # --archetype-mix 1,0,0: every agent is liquid.
    write_mini_inputs(BASE, N_SMALL)
    run_converter_ok(CONVERTER_V4, "/tmp/conv_out_all_liquid",
                     ["--archetype-mix", "1,0,0"])
    _h, rows = read_agent("/tmp/conv_out_all_liquid/input_agent.csv")
    seen = agent_archetypes(rows)
    assert len(seen) == N_SMALL, len(seen)
    assert set(seen.values()) == {"liquid"}, set(seen.values())
    print("mix all-liquid: %d agents, every one liquid" % len(seen))


def test_ranges():
    # Narrow ranges: every threshold in [0.15, 0.25], every
    # tolerance in [200, 800]; the printed min/max match the file.
    write_mini_inputs(BASE, N_SMALL)
    run_converter_ok(CONVERTER_V4, "/tmp/conv_out_ranges",
                     ["--liquid-threshold-lo", "0.15",
                      "--liquid-threshold-hi", "0.25",
                      "--walk-tolerance-lo", "200",
                      "--walk-tolerance-hi", "800"])
    _h, rows = read_agent("/tmp/conv_out_ranges/input_agent.csv")
    thresholds = [float(r["soc_threshold_frac"]) for r in rows]
    tolerances = [float(r["walk_tolerance_m"]) for r in rows]
    assert all(0.15 - 1e-9 <= v <= 0.25 + 1e-9 for v in thresholds), \
        (min(thresholds), max(thresholds))
    assert all(200.0 - 1e-9 <= v <= 800.0 + 1e-9 for v in tolerances), \
        (min(tolerances), max(tolerances))
    print("ranges: threshold min %.4f max %.4f (in 0.15-0.25), "
          "tolerance min %.1f max %.1f (in 200-800)"
          % (min(thresholds), max(thresholds),
             min(tolerances), max(tolerances)))


def test_per_agent_uniformity():
    # One draw set per agent: archetype, threshold, and tolerance
    # are the same on every leg of an agent.
    write_mini_inputs(BASE, N_SMALL)
    run_converter_ok(CONVERTER_V4, "/tmp/conv_out_uniform",
                     ["--seed", "7"])
    _h, rows = read_agent("/tmp/conv_out_uniform/input_agent.csv")
    by_agent = {}
    for row in rows:
        key = (row["archetype"], row["soc_threshold_frac"],
               row["walk_tolerance_m"])
        by_agent.setdefault(row["agent_id"], set()).add(key)
    assert len(by_agent) == N_SMALL, len(by_agent)
    assert all(len(v) == 1 for v in by_agent.values()), \
        [k for k, v in by_agent.items() if len(v) != 1][:5]
    print("per-agent uniformity: %d agents, one archetype / "
          "threshold / tolerance each" % len(by_agent))


def test_defaults():
    # Omitted flags equal the documented defaults: no --seed is the
    # same file as --seed 42, and no mix/threshold/tolerance flags
    # are the same file as even thirds, 0.10/0.30, 100/1000.
    write_mini_inputs(BASE, N_SMALL)
    run_converter_ok(CONVERTER_V4, "/tmp/conv_out_default", [])
    run_converter_ok(CONVERTER_V4, "/tmp/conv_out_explicit",
                     ["--seed", "42", "--archetype-mix", "1,1,1",
                      "--liquid-threshold-lo", "0.10",
                      "--liquid-threshold-hi", "0.30",
                      "--walk-tolerance-lo", "100",
                      "--walk-tolerance-hi", "1000"])
    with open("/tmp/conv_out_default/input_agent.csv", "rb") as fh:
        default = fh.read()
    with open("/tmp/conv_out_explicit/input_agent.csv", "rb") as fh:
        explicit = fh.read()
    assert default == explicit, "default run != explicit-defaults run"
    _h, rows = read_agent("/tmp/conv_out_default/input_agent.csv")
    counts = Counter(agent_archetypes(rows).values())
    total = sum(counts.values())
    for name in ("liquid", "plan_ahead", "opportunistic"):
        assert abs(counts[name] / total - 1.0 / 3) <= 0.08, \
            (name, counts[name] / total)
    thresholds = [float(r["soc_threshold_frac"]) for r in rows]
    tolerances = [float(r["walk_tolerance_m"]) for r in rows]
    assert min(thresholds) >= 0.10 - 1e-9
    assert max(thresholds) <= 0.30 + 1e-9
    assert min(tolerances) >= 100.0 - 1e-9
    assert max(tolerances) <= 1000.0 + 1e-9
    print("defaults: omitted flags == explicit defaults "
          "(seed 42, even thirds, 0.10-0.30, 100-1000), files "
          "byte-identical")


def test_backward_compat_v3():
    # Same synthetic inputs through v3 and v4: every shared column
    # matches row for row except archetype (v3 writes it blank;
    # v4 draws it), and the vehicle-type file is identical. v3 has
    # no threshold/tolerance columns, so they are absent from the
    # comparison by construction.
    write_mini_inputs(BASE, N_SMALL)
    run_converter_ok(CONVERTER_V3, "/tmp/conv_out_v3", [])
    run_converter_ok(CONVERTER_V4, "/tmp/conv_out_v4", ["--seed", "42"])
    header3, rows3 = read_agent("/tmp/conv_out_v3/input_agent.csv")
    header4, rows4 = read_agent("/tmp/conv_out_v4/input_agent.csv")
    assert "soc_threshold_frac" not in header3, header3
    assert "walk_tolerance_m" not in header3, header3
    assert "soc_threshold_frac" in header4, header4
    assert "walk_tolerance_m" in header4, header4
    assert len(rows3) == len(rows4), (len(rows3), len(rows4))
    shared = [c for c in header3 if c != "archetype"]
    for row3, row4 in zip(rows3, rows4):
        for column in shared:
            assert row3[column] == row4[column], \
                (column, row3, row4)
    assert all(r["archetype"] == "" for r in rows3), "v3 archetype"
    assert set(r["archetype"] for r in rows4) <= \
        {"liquid", "plan_ahead", "opportunistic"}, rows4[0]
    with open("/tmp/conv_out_v3/optional_vehicle_type.csv",
              "rb") as fh:
        types3 = fh.read()
    with open("/tmp/conv_out_v4/optional_vehicle_type.csv",
              "rb") as fh:
        types4 = fh.read()
    assert types3 == types4, "vehicle-type file differs v3 vs v4"
    print("backward compat: v4 matches v3 on %d shared columns "
          "(%d rows), archetype only differs; vehicle-type file "
          "identical" % (len(shared), len(rows3)))


def test_invalid_inputs():
    # Bad flags fail loud (nonzero exit), before any output file
    # is written.
    write_mini_inputs(BASE, N_SMALL)
    bad = [
        ["--archetype-mix", "1,1"],
        ["--liquid-threshold-lo", "0.4",
         "--liquid-threshold-hi", "0.3"],
        ["--seed", "abc"],
    ]
    for args in bad:
        proc = run_converter(CONVERTER_V4, "/tmp/conv_out_bad", args)
        assert proc.returncode != 0, (args, proc.returncode,
                                      proc.stdout[-300:])
    print("invalid inputs: --archetype-mix 1,1, inverted threshold "
          "range, and --seed abc all exit nonzero")


def main():
    test_reproducibility()
    test_mix()
    test_mix_all_liquid()
    test_ranges()
    test_per_agent_uniformity()
    test_defaults()
    test_backward_compat_v3()
    test_invalid_inputs()
    print("CONVERTER PASS: reproducibility, mix (40k agents), "
          "all-liquid mix, threshold/tolerance ranges, per-agent "
          "uniformity, defaults, v3 backward compat, invalid "
          "inputs fail loud")
    return 0


if __name__ == "__main__":
    sys.exit(main())
