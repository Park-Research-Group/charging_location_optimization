# Analyze one PySim charging run's output files.
#
# Usage:
#   python analyze_run_outputs.py [run_output_dir]
#
# run_output_dir defaults to the current directory. It reads
# output_agent.csv, output_decisions.csv, output_charging.csv and
# output_stranded.csv if present (a crashed run has no
# output_charger_summary.csv, and that is fine -- the script just
# skips it). Only the standard library is needed.
#
# The agent file is large (one row per leg, with long path
# sequences), so it is read by column index and the path columns
# are never loaded. All other files are small enough to read whole.
import csv
import sys
import os
from collections import Counter, defaultdict
from statistics import mean, median


def load_rows(path, wanted):
    # Yield dicts with only the wanted columns (by name), so the
    # big agent file never holds its path sequences in memory.
    # Missing columns are reported once and skipped.
    with open(path, newline='', encoding='utf-8') as fh:
        reader = csv.reader(fh)
        header = next(reader)
        idx = {}
        missing = []
        for col in wanted:
            if col in header:
                idx[col] = header.index(col)
            else:
                missing.append(col)
        if missing:
            print("  (%s: missing columns, skipped: %s)"
                  % (os.path.basename(path), ", ".join(missing)))
        for row in reader:
            if not row:
                continue
            yield {c: row[i] for c, i in idx.items()}


def f(row, col, default=0.0):
    try:
        return float(row.get(col, default) or default)
    except (TypeError, ValueError):
        return default


def pct(n, d):
    return 100.0 * n / d if d else 0.0


def fmt(x, nd=1):
    return "%.*f" % (nd, x)


def section(title):
    print()
    print("=== %s ===" % title)


def top(counter, n=10, label=""):
    for key, count in counter.most_common(n):
        print("  %-28s %d" % (str(key)[:28], count))


def analyze_agent(path):
    wanted = ["agent_id", "leg_index", "departure_mode", "trip_purpose",
              "day", "travel_time_in_min", "distance",
              "scheduled_departure_in_min", "scheduled_arrival_in_min",
              "actual_departure_in_min", "lateness_in_min",
              "soc_start_kwh", "soc_end_kwh", "stranded_flag",
              "tow_flag", "charged_en_route", "complete_flag"]
    n = 0
    agents = set()
    days = Counter()
    purposes = Counter()
    modes = Counter()
    travel, distance, delay, lateness = [], [], [], []
    soc_start, soc_end = [], []
    stranded = tow = enroute = 0
    stranded_by_day = Counter()
    enroute_by_day = Counter()
    low_soc_start = 0
    for row in load_rows(path, wanted):
        n += 1
        agents.add(row.get("agent_id", ""))
        day = int(f(row, "day"))
        days[day] += 1
        purposes[row.get("trip_purpose", "")] += 1
        modes[row.get("departure_mode", "")] += 1
        tt = f(row, "travel_time_in_min")
        fftt = f(row, "scheduled_arrival_in_min") - \
            f(row, "scheduled_departure_in_min")
        travel.append(tt)
        distance.append(f(row, "distance"))
        delay.append(tt - fftt)
        lateness.append(f(row, "lateness_in_min"))
        soc_start.append(f(row, "soc_start_kwh"))
        soc_end.append(f(row, "soc_end_kwh"))
        if row.get("stranded_flag") == "1":
            stranded += 1
            stranded_by_day[day] += 1
        if row.get("tow_flag") == "1":
            tow += 1
        if row.get("charged_en_route") == "1":
            enroute += 1
            enroute_by_day[day] += 1
        if f(row, "soc_start_kwh") < 0.2 * 60.0:
            # legs starting below 12 kWh (20% of the 60 kWh class)
            low_soc_start += 1
    if n == 0:
        section("LEGS (output_agent.csv, n=0)")
        print("  (no rows)")
        return
    section("LEGS (output_agent.csv, n=%d)" % n)
    print("agents with completed legs: %d" % len(agents))
    print("days covered: %s" % sorted(days))
    print("legs by day: %s" % dict(sorted(days.items())))
    print("trip purpose: %s" % dict(purposes))
    print("departure mode: %s" % dict(modes))
    print("travel time: mean %s min, median %s, max %s"
          % (fmt(mean(travel)), fmt(median(travel)), fmt(max(travel))))
    print("free-flow delay (actual - scheduled): mean %s min, "
          "median %s, p95 %s"
          % (fmt(mean(delay)), fmt(median(delay)),
             fmt(sorted(delay)[int(0.95 * len(delay))])))
    print("departure lateness: mean %s min, median %s"
          % (fmt(mean(lateness)), fmt(median(lateness))))
    print("distance: mean %s km, median %s"
          % (fmt(mean(distance), 2), fmt(median(distance), 2)))
    print("SoC at leg start: mean %s kWh, median %s; "
          "at leg end: mean %s, median %s"
          % (fmt(mean(soc_start)), fmt(median(soc_start)),
             fmt(mean(soc_end)), fmt(median(soc_end))))
    print("legs starting below 12 kWh: %d (%.1f%%)"
          % (low_soc_start, pct(low_soc_start, n)))
    print("stranded legs: %d (%.2f%%), tow legs: %d, "
          "en-route-charged legs: %d (%.1f%%)"
          % (stranded, pct(stranded, n), tow, enroute,
             pct(enroute, n)))
    print("stranded legs by day: %s" % dict(sorted(stranded_by_day.items())))
    print("en-route-charged legs by day: %s"
          % dict(sorted(enroute_by_day.items())))


def analyze_decisions(path):
    wanted = ["agent_id", "leg_index", "day", "soc_at_departure_kwh",
              "leg_energy_kwh", "remaining_energy_kwh",
              "destination_charger", "decision", "enroute_charger_id",
              "target_soc", "outcome", "trigger", "trigger_charger_id"]
    n = 0
    decisions = Counter()
    triggers = Counter()
    outcomes = Counter()
    by_day = defaultdict(Counter)
    soc_dep, leg_e, remain = [], [], []
    no_charge_soc = []
    for row in load_rows(path, wanted):
        n += 1
        day = int(f(row, "day"))
        dec = row.get("decision", "")
        decisions[dec] += 1
        by_day[day][dec] += 1
        if row.get("trigger"):
            triggers[row["trigger"]] += 1
        if row.get("outcome"):
            outcomes[row["outcome"]] += 1
        soc_dep.append(f(row, "soc_at_departure_kwh"))
        leg_e.append(f(row, "leg_energy_kwh"))
        remain.append(f(row, "remaining_energy_kwh"))
        if dec == "none":
            no_charge_soc.append(f(row, "soc_at_departure_kwh"))
    if n == 0:
        section("DECISIONS (output_decisions.csv, n=0)")
        print("  (no rows)")
        return
    section("DECISIONS (output_decisions.csv, n=%d)" % n)
    print("decision: %s" % dict(decisions))
    for day in sorted(by_day):
        print("  day %d: %s" % (day, dict(by_day[day])))
    print("trigger (among legs with a trigger): %s" % dict(triggers))
    print("outcome: %s" % dict(outcomes))
    print("SoC at departure: mean %s kWh, median %s"
          % (fmt(mean(soc_dep)), fmt(median(soc_dep))))
    print("leg energy: mean %s kWh, median %s, max %s"
          % (fmt(mean(leg_e)), fmt(median(leg_e)), fmt(max(leg_e))))
    print("remaining energy after leg: mean %s kWh, median %s, "
          "min %s" % (fmt(mean(remain)), fmt(median(remain)),
                      fmt(min(remain))))
    print("legs with NO remaining energy margin (remaining <= 0): %d "
          "(%.2f%%)" % (sum(1 for r in remain if r <= 0),
                        pct(sum(1 for r in remain if r <= 0), n)))
    if no_charge_soc:
        print("legs decided 'none': %d, their SoC at departure "
              "mean %s kWh, median %s"
              % (len(no_charge_soc), fmt(mean(no_charge_soc)),
                 fmt(median(no_charge_soc))))


def analyze_charging(path):
    wanted = ["agent_id", "charger_id", "arrival_time_in_min", "wait_in_min",
              "charge_start_in_min", "charge_end_in_min", "energy_kwh",
              "soc_before_kwh", "soc_after_kwh", "charger_power_kw",
              "leg_index", "charged_en_route", "tow_charge"]
    rows = list(load_rows(path, wanted))
    n = len(rows)
    if n == 0:
        section("CHARGING (output_charging.csv, n=0)")
        print("  (no rows)")
        return
    fast = [r for r in rows if f(r, "charger_power_kw") >= 50]
    slow = [r for r in rows if f(r, "charger_power_kw") < 50]
    energy = sum(f(r, "energy_kwh") for r in rows)
    waits = [f(r, "wait_in_min") for r in rows]
    durs = [f(r, "charge_end_in_min") - f(r, "charge_start_in_min")
            for r in rows]
    section("CHARGING (output_charging.csv, n=%d)" % n)
    print("fast (>=50 kW): %d (%.1f%%), slow: %d (%.1f%%)"
          % (len(fast), pct(len(fast), n), len(slow), pct(len(slow), n)))
    print("power breakdown: %s"
          % dict(Counter(r.get("charger_power_kw", "") for r in rows)
                 .most_common(8)))
    print("total energy: %s kWh (fast %s, %.1f%%)"
          % (fmt(energy), fmt(sum(f(r, "energy_kwh") for r in fast)),
             pct(sum(f(r, "energy_kwh") for r in fast), energy)))
    print("energy per charge: mean %s kWh, median %s, max %s"
          % (fmt(mean([f(r, "energy_kwh") for r in rows]), 2),
             fmt(median([f(r, "energy_kwh") for r in rows]), 2),
             fmt(max(f(r, "energy_kwh") for r in rows), 2)))
    print("wait: mean %s min, median %s, max %s; wait>0: %d (%.1f%%); "
          "wait>30 min: %d; wait>120 min: %d"
          % (fmt(mean(waits), 2), fmt(median(waits), 2),
             fmt(max(waits), 2), sum(1 for w in waits if w > 0),
             pct(sum(1 for w in waits if w > 0), n),
             sum(1 for w in waits if w > 30),
             sum(1 for w in waits if w > 120)))
    print("wait by power: fast mean %s min (%d with wait), "
          "slow mean %s min (%d with wait)"
          % (fmt(mean([f(r, "wait_in_min") for r in fast]), 2),
             sum(1 for r in fast if f(r, "wait_in_min") > 0),
             fmt(mean([f(r, "wait_in_min") for r in slow]), 2),
             sum(1 for r in slow if f(r, "wait_in_min") > 0)))
    print("charge duration: mean %s min, median %s, max %s"
          % (fmt(mean(durs)), fmt(median(durs)), fmt(max(durs))))
    print("en-route charges: %d (%.1f%%), tow charges: %d"
          % (sum(1 for r in rows if r.get("charged_en_route") == "1"),
             pct(sum(1 for r in rows
                     if r.get("charged_en_route") == "1"), n),
             sum(1 for r in rows if r.get("tow_charge") == "1")))
    print("unique chargers used: %d, unique agents charged: %d"
          % (len(set(r.get("charger_id", "") for r in rows)),
             len(set(r.get("agent_id", "") for r in rows))))
    print("top 10 chargers by events:")
    top(Counter(r.get("charger_id", "") for r in rows))
    energy_by = defaultdict(float)
    for r in rows:
        energy_by[r.get("charger_id", "")] += f(r, "energy_kwh")
    print("top 10 chargers by energy (kWh):")
    for key, val in sorted(energy_by.items(), key=lambda kv: -kv[1])[:10]:
        print("  %-28s %s" % (str(key)[:28], fmt(val)))
    by_day = Counter(int(f(r, "arrival_time_in_min") // 1440)
                     for r in rows)
    print("charging events by day: %s" % dict(sorted(by_day.items())))
    # Multi-power stations: a station used at more than one power
    # level shows the open-plug rule is reaching the slower plug.
    station = defaultdict(set)
    for r in rows:
        cid = r.get("charger_id", "")
        base = cid.rsplit("_L", 1)[0] if "_L" in cid else cid
        station[base].add(cid)
    multi = {s: v for s, v in station.items() if len(v) > 1}
    print("stations used at >1 power level: %d" % len(multi))
    if multi:
        l2 = sum(1 for r in rows
                 if r.get("charger_id", "").endswith("_L2")
                 and r.get("charger_id", "").rsplit("_L", 1)[0] in multi)
        l3 = sum(1 for r in rows
                 if r.get("charger_id", "").endswith("_L3")
                 and r.get("charger_id", "").rsplit("_L", 1)[0] in multi)
        print("  at those stations: L2 events %d, L3 events %d" % (l2, l3))


def analyze_stranded(path):
    wanted = ["agent_id", "leg_index", "soc_start_kwh",
              "trip_energy_kwh", "tow_charger_id", "tow_start_in_min",
              "charge_kwh", "tow_blockage_duration_min"]
    rows = list(load_rows(path, wanted))
    n = len(rows)
    if n == 0:
        section("STRANDED (output_stranded.csv, n=0)")
        print("  (no rows)")
        return
    section("STRANDED (output_stranded.csv, n=%d)" % n)
    print("unique agents stranded: %d"
          % len(set(r.get("agent_id", "") for r in rows)))
    per_agent = Counter(r.get("agent_id", "") for r in rows)
    print("strandings per agent: %s"
          % dict(sorted(Counter(per_agent.values()).items())))
    print("leg index: %s"
          % dict(sorted(Counter(r.get("leg_index", "")
                                for r in rows).items(),
                        key=lambda kv: int(kv[0]) if kv[0] else 0)))
    print("SoC at stranding: mean %s kWh, median %s"
          % (fmt(mean([f(r, "soc_start_kwh") for r in rows])),
             fmt(median([f(r, "soc_start_kwh") for r in rows]))))
    print("trip energy: mean %s kWh, median %s, max %s"
          % (fmt(mean([f(r, "trip_energy_kwh") for r in rows])),
             fmt(median([f(r, "trip_energy_kwh") for r in rows])),
             fmt(max(f(r, "trip_energy_kwh") for r in rows))))
    print("tow charge: mean %s kWh" % fmt(mean(
        [f(r, "charge_kwh") for r in rows])))
    print("tow blockage: mean %s min"
          % fmt(mean([f(r, "tow_blockage_duration_min") for r in rows]),
                2))
    print("unique tow chargers: %d"
          % len(set(r.get("tow_charger_id", "") for r in rows)))
    print("top 10 tow chargers:")
    top(Counter(r.get("tow_charger_id", "") for r in rows))
    by_day = Counter(int(f(r, "tow_start_in_min") // 1440) for r in rows)
    print("strandings by day: %s" % dict(sorted(by_day.items())))


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    print("Run output dir: %s" % os.path.abspath(out_dir))
    files = {
        "agent": os.path.join(out_dir, "output_agent.csv"),
        "decisions": os.path.join(out_dir, "output_decisions.csv"),
        "charging": os.path.join(out_dir, "output_charging.csv"),
        "stranded": os.path.join(out_dir, "output_stranded.csv"),
    }
    for key, path in files.items():
        if os.path.exists(path):
            print("  found %s (%d bytes)"
                  % (os.path.basename(path), os.path.getsize(path)))
        else:
            print("  missing %s (skipped)" % os.path.basename(path))
    if os.path.exists(files["agent"]):
        analyze_agent(files["agent"])
    if os.path.exists(files["decisions"]):
        analyze_decisions(files["decisions"])
    if os.path.exists(files["charging"]):
        analyze_charging(files["charging"])
    if os.path.exists(files["stranded"]):
        analyze_stranded(files["stranded"])
    print()
    print("Note: these are the rows written so far. On a crashed or "
          "partial run the day counts stop at the crash; the last day "
          "is a partial day.")


if __name__ == "__main__":
    main()
