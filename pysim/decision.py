# Pre-trip charging decisions (slice 2).
#
# decide_charging is the single dispatch point for "should this leg
# charge, and where": it runs once per leg, at stage-0 generation,
# when the leg's path is known. The caller (ChargingEngine) supplies
# a ctx dict with the planning numbers; this module only applies the
# rules, it never touches traffic or plug queues.
#
# Archetypes: the dispatch on leg.archetype happens in exactly one
# place, at the top of decide_charging. No named archetypes exist
# yet, so a blank archetype and any unknown non-blank archetype both
# fall through to the default rules below. The archetype may later
# become a high-dimensional weight vector (mixed membership, e.g.
# 50% one archetype / 50% another) multiplied against situational
# dimensions (SoC, distance remaining to charger, charger at end
# location, home charging, ...); the string dispatch here is the
# seam that will take the vector.
from .sp import find_best_path

KM_PER_MILE = 1.60934  # DTALite link lengths are in miles


def leg_free_flow_energy(world, leg, consumption):
    # Free-flow energy of the leg's assigned path: sum over its
    # links (the trailing 65530 sentinel is not a link). 0.0 for a
    # leg with fewer than 2 nodes.
    if leg.node_size < 2 or not leg.link_ary:
        return 0.0
    total = 0.0
    for entry in leg.link_ary[:-1]:
        if entry.link_no == 65530:
            continue
        link = world.links[entry.link_no]
        total += float(link.length) * KM_PER_MILE * consumption
    return total


def _path_energy_time(world, ff_net, origin_zone, origin, departure,
                      dest_zone, dest, veh):
    # (node_size, path_links, energy_kwh, travel_time_min) for the
    # free-flow shortest path origin -> dest, or (0, [], 0.0, 0.0)
    # if none exists.
    node_size, path_links, total_cost = find_best_path(
        world, ff_net, origin_zone, origin, departure, dest_zone,
        dest, veh.demand_type, veh.vot)
    if node_size < 2:
        return 0, [], 0.0, 0.0
    params = world.ev_params(veh)
    consumption = params[1] if params is not None else 0.0
    energy = 0.0
    for link_no in path_links:
        energy += float(world.links[link_no].length) * KM_PER_MILE \
            * consumption
    return node_size, path_links, energy, float(total_cost)


def find_enroute_charger(world, ff_net, veh, leg, soc_kwh, consumption,
                         target_soc_frac):
    # Best viable en-route charger site for this leg, or None.
    # Viability for a site at node C: the free-flow shortest path
    # origin -> C exists with energy <= soc_kwh, and the free-flow
    # shortest path C -> dest exists with energy <= target_soc_frac
    # * battery. Choice: minimum total free-flow travel time
    # (origin -> C plus C -> dest), tie-break higher power_kw.
    # Sites at the leg's origin or destination node are excluded:
    # those are start/destination charging, not en-route. Returns
    # (site, links_to_charger, links_to_dest, via_time_min): the
    # two free-flow path link lists and their total travel time,
    # so the caller can build the via path without re-running the
    # shortest-path queries. Slice 3a's generation splice uses
    # them; the choice rules are unchanged from slice 2.
    params = world.ev_params(veh)
    if params is None:
        return None
    battery = params[0]
    engine = getattr(world, "charging_engine", None)
    if engine is not None:
        sites = engine.sites
    else:
        from .charging import read_charger_sites
        sites, _by_node = read_charger_sites(world)
    departure = leg.departure_actual \
        if leg.departure_actual is not None \
        else float(leg.scheduled_departure)
    best = None
    best_time = None
    best_power = -1.0
    best_links_to = None
    best_links_from = None
    for site in sites:
        if site.node_id in (leg.origin_node, leg.dest_node):
            continue
        site_zone = world.nodes[site.node_id].zone_id
        _n1, links_to, energy1, time1 = _path_energy_time(
            world, ff_net, leg.origin_zone, leg.origin_node,
            departure, site_zone, site.node_id, veh)
        if _n1 < 2 or energy1 > soc_kwh:
            continue
        _n2, links_from, energy2, time2 = _path_energy_time(
            world, ff_net, site_zone, site.node_id,
            int(departure + time1), leg.dest_zone, leg.dest_node,
            veh)
        if _n2 < 2 or energy2 > target_soc_frac * battery:
            continue
        total_time = time1 + time2
        if best_time is None or total_time < best_time - 1e-9 or \
                (abs(total_time - best_time) <= 1e-9 and
                 site.power_kw > best_power):
            best = site
            best_time = total_time
            best_power = site.power_kw
            best_links_to = links_to
            best_links_from = links_from
    if best is None:
        return None
    return (best, best_links_to, best_links_from, best_time)


def decide_charging(world, veh, leg, ctx):
    # (decision, charger_site_or_None, target_soc_frac) where
    # decision is "none" | "destination" | "enroute". ctx carries:
    # soc_kwh, leg_energy_kwh, remaining_energy_kwh, dest_site,
    # home_available, archetype, enroute_target_soc,
    # enroute_charger, battery_kwh, config. world/veh are accepted
    # for future archetype rules that may need them; the default
    # rules below use only ctx.
    soc = ctx["soc_kwh"]
    leg_energy = ctx["leg_energy_kwh"]
    remaining = ctx["remaining_energy_kwh"]
    dest_site = ctx["dest_site"]
    home_available = ctx["home_available"]
    archetype = ctx["archetype"]
    enroute_charger = ctx["enroute_charger"]
    battery = ctx["battery_kwh"]
    cfg = ctx["config"]
    # Single archetype dispatch point: no named archetypes exist
    # yet. A blank archetype takes the default rules; an unknown
    # non-blank archetype falls back to them as well (documented
    # above), so both cases run the same code below.
    if archetype and str(archetype).strip():
        pass  # unknown archetype: default rules
    # Default rules, in order. (1) takes precedence because if the
    # leg itself cannot be completed, reaching the destination to
    # charge there is impossible, so en-route comes first.
    margin = cfg.get("enroute_margin_frac", 0.1) * battery
    if soc < leg_energy + margin and enroute_charger is not None:
        return ("enroute", enroute_charger,
                ctx["enroute_target_soc"])
    # (2) destination, but only when the leg itself can be
    # completed: a destination the vehicle cannot reach cannot
    # charge it, so an uncompletable leg with no viable en-route
    # charger falls through to "none" and strands.
    if soc >= leg_energy and \
            (dest_site is not None or home_available) and \
            (soc - leg_energy < remaining - leg_energy or
             soc < 0.20 * battery):
        if dest_site is None:
            return ("destination", None, 1.0)
        target = cfg["target_soc_fast"] \
            if dest_site.power_kw >= cfg["fast_threshold_kw"] \
            else cfg["target_soc_slow"]
        return ("destination", dest_site, target)
    return ("none", None, 0.0)
