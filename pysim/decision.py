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
from .cpp import ProgramStop, f32
from .sp import find_best_path, find_best_paths_forward, \
    find_best_paths_reverse, multi_run_valid

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


def _links_energy(world, path_links, consumption):
    # Free-flow energy of a link list, summed in path order
    # exactly as _path_energy_time sums a query's path.
    energy = 0.0
    for link_no in path_links:
        energy += float(world.links[link_no].length) * KM_PER_MILE \
            * consumption
    return energy


def _path_time(world, ff_net, path_links, departure):
    # Travel time of path_links from a departure, mirroring
    # find_best_path's f32 accumulation in path order: the
    # departure is truncated to int and clamped to
    # [world.start, world.end], each link's TD time is looked
    # up at the running (horizon-capped) time, and the total is
    # the f32 cost minus the departure. The TD table is constant
    # across intervals wherever this is used (see
    # multi_run_valid), so the interval lookup cannot change
    # the result; it is kept so the arithmetic stays identical
    # to a forward find_best_path call on the same path.
    dep = int(departure)
    if dep < world.start:
        dep = world.start
    if dep > world.end:
        dep = world.end
    time_acc = f32(dep)
    cost_acc = f32(dep)
    for link_no in path_links:
        interval = world.interval_index(time_acc)
        td = ff_net.td_time[link_no][interval]
        time_acc = f32(time_acc + td)
        cost_acc = f32(cost_acc + td)
        if time_acc > world.horizon - 1:
            time_acc = f32(world.horizon - 1)
    return float(f32(cost_acc - f32(dep)))


def _find_enroute_charger_multi(world, ff_net, veh, leg, soc_kwh,
                                consumption, target_soc_frac, sites,
                                departure, battery):
    # v6 bypasses this too (see find_enroute_charger below).
    # Multi-destination version of the search below: two
    # label-correcting runs (forward from the origin, reverse
    # from the destination) replace the 2xS per-site
    # find_best_path calls. Viability and selection are the
    # same rules, applied to the same per-site paths and times:
    # a site is viable when the origin -> site path exists with
    # energy <= soc_kwh and the site -> dest path exists with
    # energy <= target_soc_frac * battery; the choice is the
    # minimum total free-flow travel time, tie-break higher
    # power_kw, sites in file order. Only called when
    # multi_run_valid holds, so the per-site zone filter and
    # departure-time effects cannot change any path. The
    # multi-runs only select the site: the returned paths
    # are recomputed below with the original per-site calls
    # for the selected site, because the multi-run path
    # extraction tie-breaks differently from find_best_path
    # on equal-cost paths.
    candidates = [site for site in sites
                  if site.node_id not in (leg.origin_node, leg.dest_node)]
    if not candidates:
        return None
    forward = find_best_paths_forward(
        world, ff_net, leg.origin_node, departure,
        [site.node_id for site in candidates])
    first = {}
    for site in candidates:
        got = forward.get(site.node_id)
        if got is None:
            continue
        _n1, links_to, total_cost = got
        time1 = float(total_cost)
        if _links_energy(world, links_to, consumption) > soc_kwh:
            continue
        first[site.node_id] = (links_to, time1)
    if not first:
        return None
    reverse = find_best_paths_reverse(world, ff_net, leg.dest_node,
                                      list(first))
    best = None
    best_time = None
    best_power = -1.0
    best_links_to = None
    best_links_from = None
    for site in candidates:
        got = first.get(site.node_id)
        if got is None:
            continue
        links_from = reverse.get(site.node_id)
        if links_from is None:
            continue
        links_to, time1 = got
        time2 = _path_time(world, ff_net, links_from,
                           int(departure + time1))
        if _links_energy(world, links_from, consumption) > \
                target_soc_frac * battery:
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
    # Hybrid: the multi-runs chose the site; the returned paths
    # come from the original per-site calls for that site alone,
    # with exactly the arguments the per-site loop below uses
    # (same zones, same departure handling). On a uniform grid
    # many equal-cost paths tie, and the multi-run extraction
    # tie-breaks differently from find_best_path, so the
    # multi-run geometry is not byte-identical to v3 even though
    # the site and the times agree. The per-site total_costs
    # are returned as the via time, exactly as the per-site
    # loop returns them.
    site_zone = world.nodes[best.node_id].zone_id
    _n1, links_to, _e1, time1 = _path_energy_time(
        world, ff_net, leg.origin_zone, leg.origin_node,
        departure, site_zone, best.node_id, veh)
    _n2, links_from, _e2, time2 = _path_energy_time(
        world, ff_net, site_zone, best.node_id,
        int(departure + time1), leg.dest_zone, leg.dest_node,
        veh)
    if _n1 >= 2 and _n2 >= 2:
        return (best, links_to, links_from, time1 + time2)
    # Unreachable in practice (the multi-run found both legs
    # viable under the same costs), but keep the multi-run
    # result rather than dropping a viable charger.
    return (best, best_links_to, best_links_from, best_time)


def find_enroute_charger(world, ff_net, veh, leg, soc_kwh, consumption,
                         target_soc_frac):
    # v6 BYPASSES this search: the en-route decision now uses the
    # nearest-charger map (choose_charger_for_trigger, "liquid"
    # branch, called from decide_for_leg). Kept, unused, so the
    # v4 min-detour choice can be restored by pointing
    # decide_for_leg back at this function.
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
    #
    # The paths are normally computed by two full-network
    # label-correcting runs (_find_enroute_charger_multi) instead
    # of 2xS per-site find_best_path calls; when the network or
    # TD table cannot support the multi-run (connector links, or
    # a time-varying TD table), the original per-site loop below
    # runs instead and returns the same result.
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
    if multi_run_valid(world, ff_net):
        return _find_enroute_charger_multi(
            world, ff_net, veh, leg, soc_kwh, consumption,
            target_soc_frac, sites, departure, battery)
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


def _links_dist_km(world, path_links):
    # Network distance (km) of a link list, summed in path
    # order, mirroring _links_energy without the consumption.
    total = 0.0
    for link_no in path_links:
        total += float(world.links[link_no].length) * KM_PER_MILE
    return total


def _trigger_sp_leg(world, ff_net, origin, dest, departure,
                    consumption, demand_type, vot):
    # (links, energy_kwh, time_min, reachable) for the free-flow
    # shortest path origin -> dest. origin == dest is reachable
    # with an empty path (the agent is already there); any other
    # missing path is unreachable.
    if origin == dest:
        return [], 0.0, 0.0, True
    origin_zone = world.nodes[origin].zone_id
    dest_zone = world.nodes[dest].zone_id
    node_size, links, total_cost = find_best_path(
        world, ff_net, origin_zone, origin, departure,
        dest_zone, dest, demand_type, vot)
    if node_size < 2:
        return [], 0.0, 0.0, False
    return links, _links_energy(world, links, consumption), \
        float(total_cost), True


def choose_charger_for_trigger(nearest_map, trigger, from_node, dest_node,
                               soc_kwh, consumption_kwh_per_km, battery_kwh,
                               target_soc_frac, walk_tolerance_m=None,
                               world=None, ff_net=None, veh=None,
                               departure=0.0):
    # v6: the departure-time en-route decision reuses the
    # "liquid" branch of this seam (see decide_for_leg); the
    # caller applies the origin/destination exclusion, which
    # the triggers do not need.
    # "Closest reachable charger" for an archetype trigger (v5
    # Phase 1): (site, links_to_charger, links_from_charger,
    # via_time_min) or None. No weights, no scoring features --
    # this is the seam a future scoring version replaces. The
    # choice is the time-nearest charger to the lookup node
    # ("liquid" / "plan_ahead": from_node; "opportunistic":
    # dest_node), with reachability on that charger only: if the
    # time-nearest charger is not reachable, there is no stop,
    # and no second-nearest attempt is made. links_to may be []
    # (lookup node IS the charger node) and links_from may be []
    # (charger node IS the destination); the caller handles empty
    # lists. The map path and the fallback path apply the same
    # contract; they differ only in how the nearest charger is
    # found. Extra context (world, ff_net, veh, departure) is
    # optional: world / ff_net default to the map's when a map is
    # given, and to the engine's free-flow net otherwise; veh
    # supplies demand_type / vot for the shortest-path calls
    # (defaults 1 / 10.0); departure is the leg departure used
    # by the per-site fallback, and is irrelevant on the map path
    # (the TD table is constant there, so travel times do not
    # depend on it). The fallback needs world and ff_net passed
    # explicitly when there is no map to carry them.
    if trigger not in ("liquid", "plan_ahead", "opportunistic"):
        raise ProgramStop(
            "choose_charger_for_trigger: unknown trigger %r"
            % (trigger,))
    if trigger == "opportunistic" and walk_tolerance_m is None:
        raise ProgramStop(
            "choose_charger_for_trigger: opportunistic needs "
            "walk_tolerance_m")
    if world is None and nearest_map is not None:
        world = nearest_map.world
        ff_net = nearest_map.ff_net
    if world is not None and ff_net is None:
        engine = getattr(world, "charging_engine", None)
        if engine is not None:
            ff_net = engine._ff_net()
    if world is None or ff_net is None:
        raise ProgramStop(
            "choose_charger_for_trigger needs world and ff_net")
    demand_type = veh.demand_type if veh is not None else 1
    vot = veh.vot if veh is not None else 10.0
    consumption = consumption_kwh_per_km
    if nearest_map is not None:
        lookup = from_node if trigger in ("liquid", "plan_ahead") \
            else dest_node
        idx = nearest_map.site_index[lookup]
        if idx < 0:
            return None
        site = nearest_map.sites[idx]
        if trigger == "opportunistic":
            # Walk test on the map distance dest -> charger.
            if nearest_map.dist_km[lookup] * 1000.0 \
                    > walk_tolerance_m:
                return None
            links_to, energy_to, time_to, ok = _trigger_sp_leg(
                world, ff_net, from_node, site.node_id, departure,
                consumption, demand_type, vot)
            if not ok or energy_to > soc_kwh + 1e-9:
                return None
            links_from, energy_from, time_from, ok = _trigger_sp_leg(
                world, ff_net, site.node_id, dest_node,
                int(departure + time_to), consumption,
                demand_type, vot)
            if not ok or energy_from > \
                    target_soc_frac * battery_kwh + 1e-9:
                return None
            return (site, links_to, links_from, time_to + time_from)
        links_to = nearest_map.path_to_charger(from_node)
        if links_to is None:
            return None
        if nearest_map.dist_km[from_node] * consumption \
                > soc_kwh + 1e-9:
            return None
        time_to = nearest_map.time_min[from_node]
        links_from, energy_from, time_from, ok = _trigger_sp_leg(
            world, ff_net, site.node_id, dest_node,
            int(departure + time_to), consumption,
            demand_type, vot)
        if not ok or energy_from > \
                target_soc_frac * battery_kwh + 1e-9:
            return None
        return (site, links_to, links_from, time_to + time_from)
    # Fallback (no map): per-site loop, mirroring the pre-multi
    # loop in find_enroute_charger, but with the trigger's lookup
    # node and reachability rules, and without that loop's
    # origin/destination exclusion -- a site at the lookup or
    # destination node is a valid (empty-leg) answer here. The
    # contract is the same as the map path (§6): first select the
    # time-nearest site to the lookup node (tie-break higher power,
    # then file order), then apply the reachability checks to THAT
    # site only. A nearest site that fails reachability means no
    # stop; there is no second-nearest attempt.
    engine = getattr(world, "charging_engine", None)
    if engine is not None:
        sites = engine.sites
    else:
        from .charging import read_charger_sites
        sites, _by_node = read_charger_sites(world)
    lookup = from_node if trigger in ("liquid", "plan_ahead") \
        else dest_node
    best_idx = None
    best_time = None
    best_power = -1.0
    lookup_links = None
    for idx, site in enumerate(sites):
        _l, _e, t, ok = _trigger_sp_leg(
            world, ff_net, lookup, site.node_id, departure,
            consumption, demand_type, vot)
        if not ok:
            continue
        if best_time is None or t < best_time - 1e-9 or \
                (abs(t - best_time) <= 1e-9 and
                 site.power_kw > best_power):
            best_time = t
            best_power = site.power_kw
            best_idx = idx
            lookup_links = _l
    if best_idx is None:
        return None
    site = sites[best_idx]
    if trigger == "opportunistic":
        if _links_dist_km(world, lookup_links) * 1000.0 \
                > walk_tolerance_m:
            return None
        links_to, energy_to, time_to, ok = _trigger_sp_leg(
            world, ff_net, from_node, site.node_id, departure,
            consumption, demand_type, vot)
        if not ok or energy_to > soc_kwh + 1e-9:
            return None
    else:
        links_to = lookup_links
        energy_to = _links_energy(world, links_to, consumption)
        time_to = best_time
        if energy_to > soc_kwh + 1e-9:
            return None
    links_from, energy_from, time_from, ok = _trigger_sp_leg(
        world, ff_net, site.node_id, dest_node,
        int(departure + time_to), consumption,
        demand_type, vot)
    if not ok or energy_from > \
            target_soc_frac * battery_kwh + 1e-9:
        return None
    return (site, links_to, links_from, time_to + time_from)


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
