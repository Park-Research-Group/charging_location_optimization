# Parity check: trimmed charging-fork pysim vs standalone v2
# (emissions off) on the Phase-0 microtest fixtures. Compares
# output_agent.csv on every column the trimmed build still writes:
# the dropped columns (toll_cost + the seven emission columns) are
# excluded; everything else must match exactly, row for row.
import csv
import io
import os
import sys

sys.path.insert(0, "/tmp/pysim_v2")

from pysim import run_pysim as run_v2  # noqa: E402

# Re-import under the trimmed tree: the two packages share the module
# name, so load the trimmed one under a different package name.
import types  # noqa: E402
TRIM_DIR = os.path.join(os.path.dirname(__file__), "..", "pysim")
_pkg = types.ModuleType("pysim_trimmed")
_pkg.__path__ = [TRIM_DIR]
sys.modules["pysim_trimmed"] = _pkg
from pysim_trimmed.api import run_pysim as run_trimmed  # noqa: E402

FIXTURES = ["test1_advisory", "test1_abm", "test2_enroute", "test3_edge"]
SRC = os.path.expanduser("~/workspace/pysim_phase0/microtests")
DROPPED = {"toll_cost", "TotalEnergy_(KJ)", "CO2_(g)", "NOX_(g)",
           "CO_(g)", "HC_(g)", "PM_(g)", "PM_2.5(g)"}


def read_rows(path):
    with open(path, newline="") as fh:
        text = fh.read()
    return [r for r in csv.reader(io.StringIO(text)) if r]


def project(row, header):
    # Full rows: drop the removed columns by header name. Short rows
    # (agents with < 2 nodes) carry the old short layout; drop the
    # toll + emission positions of that layout instead.
    if len(row) == len(header):
        keep = [i for i, h in enumerate(header)
                if h.strip() not in DROPPED]
        return [row[i] for i in keep]
    short_drop = {17, 19, 20, 21, 22, 23, 24, 25}
    return [v for i, v in enumerate(row) if i not in short_drop]


def main():
    only = sys.argv[1:]
    fixtures = only or FIXTURES
    failures = 0
    for fx in fixtures:
        src = os.path.join(SRC, fx)
        v2_out = "/tmp/parity_v2/" + fx
        tr_out = "/tmp/parity_trim/" + fx
        r1 = run_v2(src, v2_out)
        r2 = run_trimmed(src, tr_out)
        m = read_rows(os.path.join(v2_out, "output_agent.csv"))
        t = read_rows(os.path.join(tr_out, "output_agent.csv"))
        hdr_m = [h.strip() for h in m[0]]
        hdr_t = [h.strip() for h in t[0]]
        shared = [h for h in hdr_m if h in hdr_t]
        assert not (set(hdr_m) - set(hdr_t) - DROPPED), \
            "trimmed build dropped an unexpected column"
        body_m = [project(r, hdr_m) for r in m[1:]]
        body_t = t[1:]
        same = sum(1 for a, b in zip(body_m, body_t) if a == b)
        n = min(len(body_m), len(body_t))
        status = "OK" if (same == n and len(body_m) == len(body_t)
                          and r1["agents"] == r2["agents"]
                          and r1["completed"] == r2["completed"]) else "FAIL"
        if status == "FAIL":
            failures += 1
            for a, b in zip(body_m, body_t):
                if a != b:
                    print("first mismatch in", fx)
                    print("  v2:    ", a)
                    print("  trimmed:", b)
                    break
        print("%s: %s rows=%d/%d match, agents %d/%d, completed %d/%d"
              % (fx, status, same, n, r1["agents"], r2["agents"],
                 r1["completed"], r2["completed"]))
    print("PARITY " + ("PASS" if failures == 0 else "FAIL"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
