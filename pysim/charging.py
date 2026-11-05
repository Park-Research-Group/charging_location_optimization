# Charging accounting post-pass (Swedish-run layer, v1).
#
# Runs AFTER network loading and never touches traffic state: it reads
# each completed vehicle's travelled links, tracks state of charge,
# and accounts charging at destination nodes that host charger sites,
# with per-station plug queues measured in accounting time only. A
# vehicle that runs out of charge mid-trip is recorded as stranded;
# its traffic is untouched. Stranding feedback into traffic, charging
# decisions (archetypes, detours), and node dwell are later layers:
# this pass deliberately answers only "how much charging demand, where,
# and how long are the queues" for a fixed set of trajectories.
#
# Swedish defaults used here: battery classes 40/60/100 kWh and charger
# power tiers (home 11 kW, intermediate 22 kW, fast 50/150 kW) come from
# Liao et al. 2023 and the Zenodo 7549847 dataset. The initial-SoC
# rule, the charge-to targets, and the default consumption rate are
# v1 assumptions, not the paper's: they are parameters, not findings.
import os

from .cpp import CsvParser, ProgramStop

KM_PER_MILE = 1.60934  # DTALite link lengths are in miles

DEFAULT_CONFIG = {
    "initial_soc_home": 1.0,     # fraction of battery, v1 assumption:
    "initial_soc_no_home": 0.8,  # home-charger owners start full
    "target_soc_fast": 0.8,      # fraction, fast chargers (at/above
    "target_soc_slow": 1.0,      #   the threshold); slower chargers
    "fast_threshold_kw": 50,     #   fill to full
    "consumption_default": 0.18,  # kWh/km, v1 assumption
}


class ChargerSite:
    __slots__ = ("charger_id", "node_id", "node_number", "power_kw",
                 "plugs", "kind", "plug_free", "events")

    def __init__(self, charger_id, node_id, node_number, power_kw, plugs,
                 kind):
        self.charger_id = charger_id
        self.node_id = node_id
        self.node_number = node_number
        self.power_kw = power_kw
        self.plugs = plugs
        self.kind = kind
        self.plug_free = [0.0] * plugs
        self.events = []


def read_charger_sites(world):
    path = os.path.join(world.dir, "input_charger.csv")
    if not os.path.exists(path):
        raise ProgramStop("input_charger.csv not found in " + world.dir)
    parser = CsvParser(path)
    sites = []
    by_node = {}
    while parser.read_record():
        ok, charger_id = parser.get("charger_id", "str")
        if not ok:
            break
        ok, node_number = parser.get("node_id", "int", required=True)
        if not ok:
            raise ProgramStop("node_id missing in input_charger.csv")
        if node_number not in world.node_index:
            raise ProgramStop(
                "charger %s node %d undefined" % (charger_id, node_number))
        ok, power = parser.get("power_kw", "float", required=True)
        if not ok or power <= 0:
            raise ProgramStop(
                "charger %s power_kw missing or not positive" % charger_id)
        ok, plugs = parser.get("plugs", "int")
        if not ok or plugs < 1:
            plugs = 1
        ok, kind = parser.get("kind", "str")
        site = ChargerSite(charger_id, world.node_index[node_number],
                           node_number, float(power), plugs,
                           kind if ok else "")
        sites.append(site)
        by_node.setdefault(site.node_id, []).append(site)
    return sites, by_node


def _ev_params(world, veh):
    # (battery_kwh, consumption_kwh_per_km) or None for non-EVs.
    idx = veh.vehicle_type - 1
    if idx < 0 or idx >= len(world.vehicle_type_is_electric):
        return None
    if not world.vehicle_type_is_electric[idx]:
        return None
    battery = world.vehicle_type_battery[idx]
    consumption = world.vehicle_type_consumption[idx]
    if battery <= 0:
        return None
    if consumption <= 0:
        consumption = DEFAULT_CONFIG["consumption_default"]
    return battery, consumption


def run_charging(world, output_dir, config):
    cfg = dict(DEFAULT_CONFIG)
    cfg.update(config or {})
    sites, by_node = read_charger_sites(world)
    events = []
    stranded = []
    pending = []
    for agent_id in sorted(world.vehicles):
        veh = world.vehicles[agent_id]
        if not veh.complete or veh.node_size < 2:
            continue
        params = _ev_params(world, veh)
        if params is None:
            continue
        battery, consumption = params
        soc_start = battery * (cfg["initial_soc_home"] if veh.home_charger
                               else cfg["initial_soc_no_home"])
        veh.soc_start_kwh = soc_start
        # Walk the travelled links, deducting energy. The first link
        # whose cumulative use exceeds the starting charge strands
        # the vehicle at that link's to-node.
        uses = []
        for j in range(veh.node_size - 1):
            link = world.links[veh.link_ary[j].link_no]
            uses.append(float(link.length) * KM_PER_MILE * consumption)
        total_energy = sum(uses)
        cumulative = 0.0
        hit_zero = None
        for j, use in enumerate(uses):
            cumulative += use
            if cumulative > soc_start:
                hit_zero = world.nodes[
                    world.links[veh.link_ary[j].link_no].to_id].number
                break
        if hit_zero is not None:
            stranded.append((veh.agent_id, veh.tour_id, hit_zero,
                             total_energy, soc_start))
            veh.soc_end_kwh = 0.0
            continue
        soc_end = soc_start - total_energy
        veh.soc_end_kwh = soc_end
        # Destination charging, v1 rule: if the destination node hosts
        # a charger site (first site in file order when several do)
        # and the vehicle is below that charger's target, it charges
        # up to the target at the site's power. Queueing is accounting
        # only: waiting time never delays the vehicle's trip.
        node_sites = by_node.get(veh.dest_node)
        if not node_sites:
            continue
        site = node_sites[0]
        target_frac = cfg["target_soc_fast"] \
            if site.power_kw >= cfg["fast_threshold_kw"] \
            else cfg["target_soc_slow"]
        target_kwh = target_frac * battery
        if soc_end >= target_kwh:
            continue
        energy = target_kwh - soc_end
        duration = energy / site.power_kw * 60.0
        pending.append((float(veh.arrival), veh.agent_id, veh.tour_id,
                        site, energy, duration, soc_end, target_kwh))
    # FIFO queueing needs arrivals in chronological order, not
    # agent-id order: sort first, then assign plugs per site.
    pending.sort(key=lambda p: (p[0], p[1]))
    for arrival, agent_id, tour_id, site, energy, duration, soc_end, \
            target_kwh in pending:
        plug = min(range(site.plugs), key=lambda p: site.plug_free[p])
        start = max(arrival, site.plug_free[plug])
        wait = start - arrival
        end = start + duration
        site.plug_free[plug] = end
        event = (agent_id, tour_id, site.charger_id,
                 site.node_number, arrival, wait, start, end, energy,
                 soc_end, target_kwh, site.power_kw)
        site.events.append(event)
        events.append(event)
    _write_events(output_dir, events)
    _write_summary(world, output_dir, sites)
    _write_stranded(output_dir, stranded)
    summary = {
        "charges": len(events),
        "stranded": len(stranded),
        "sites": len(sites),
        "total_energy_kwh": sum(e[8] for e in events),
    }
    return summary


def _write_events(output_dir, events):
    lines = ["agent_id,tour_id,charger_id,node_id,arrival_time_in_min,"
             "wait_in_min,charge_start_in_min,charge_end_in_min,"
             "energy_kwh,soc_before_kwh,soc_after_kwh,charger_power_kw\n"]
    for e in events:
        lines.append(
            "%d,%d,%s,%d,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f\n" % e)
    with open(os.path.join(output_dir, "output_charging.csv"),
              "w", newline="", encoding="utf-8") as fh:
        fh.writelines(lines)


def _write_summary(world, output_dir, sites):
    lines = ["charger_id,node_id,power_kw,plugs,kind,charges_served,"
             "total_energy_kwh,mean_wait_min,max_wait_min,"
             "max_queue_length,utilization\n"]
    for site in sites:
        evs = site.events
        total_energy = sum(e[8] for e in evs)
        busy = sum(e[7] - e[6] for e in evs)
        waits = [e[5] for e in evs]
        # Max queue: at each arrival, count vehicles that have
        # arrived but not yet started (including the arriver).
        max_queue = 0
        for e in evs:
            waiting = sum(1 for f in evs if f[4] <= e[4] < f[6])
            if waiting > max_queue:
                max_queue = waiting
        span = 0.0
        if evs:
            span = max(e[7] for e in evs) - world.start
        util = busy / (site.plugs * span) if span > 0 else 0.0
        lines.append(
            "%s,%d,%.1f,%d,%s,%d,%.3f,%.2f,%.2f,%d,%.4f\n"
            % (site.charger_id, site.node_number, site.power_kw,
               site.plugs, site.kind, len(evs), total_energy,
               sum(waits) / len(waits) if waits else 0.0,
               max(waits) if waits else 0.0, max_queue, util))
    with open(os.path.join(output_dir, "output_charger_summary.csv"),
              "w", newline="", encoding="utf-8") as fh:
        fh.writelines(lines)


def _write_stranded(output_dir, stranded):
    lines = ["agent_id,tour_id,stranded_node_id,trip_energy_kwh,"
             "soc_start_kwh\n"]
    for row in stranded:
        lines.append("%d,%d,%d,%.3f,%.3f\n" % row)
    with open(os.path.join(output_dir, "output_stranded.csv"),
              "w", newline="", encoding="utf-8") as fh:
        fh.writelines(lines)
