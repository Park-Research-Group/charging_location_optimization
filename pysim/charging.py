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
import heapq
import os

from .cpp import CsvParser, ProgramStop
from .decision import choose_charger_for_trigger, decide_charging, \
    _links_energy, leg_free_flow_energy
from .sp import SPNetwork, find_best_path, multi_run_valid

KM_PER_MILE = 1.60934  # DTALite link lengths are in miles

DEFAULT_CONFIG = {
    "initial_soc_home": 1.0,     # fraction of battery, v1 assumption:
    "initial_soc_no_home": 0.8,  # home-charger owners start full
    "target_soc_fast": 0.8,      # fraction, fast chargers (at/above
    "target_soc_slow": 1.0,      #   the threshold); slower chargers
    "fast_threshold_kw": 50,     #   fill to full
    "consumption_default": 0.18,  # kWh/km, v1 assumption
    "enroute_margin_frac": 0.1,   # fraction of battery: an en-route
    "home_power_kw": 11.0,        #   stop is considered when SoC is
    # within this margin of the leg's energy need (v1 assumption);
    # home dwell charging power in kW (v1 assumption, 11 kW tier)
    "tow_dispatch_min": 30.0,     # minutes for the tow truck to reach
    #   a stranded vehicle (v1 assumption)
    "tow_target_soc": 0.8,        # fraction of battery the tow charge
    #   restores (v1 assumption)
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


def stranding_point(world, leg, soc_start, consumption):
    # Node number where a leg strands: walk its links accumulating
    # energy from soc_start; the stranded node is the to-node of
    # the first link whose cumulative use exceeds soc_start.
    # None if the leg's energy never exceeds soc_start.
    cumulative = 0.0
    for entry in leg.link_ary[:-1]:
        if entry.link_no == 65530:
            continue
        link = world.links[entry.link_no]
        cumulative += float(link.length) * KM_PER_MILE * consumption
        if cumulative > soc_start:
            return world.nodes[link.to_id].number
    return None


def _site_target_frac(site, cfg):
    return cfg["target_soc_fast"] \
        if site.power_kw >= cfg["fast_threshold_kw"] \
        else cfg["target_soc_slow"]


def _distance_tree(world, origin_id):
    # Single-source shortest path by link length in miles from
    # origin_id (Dijkstra): (dist, pred_link); unreachable nodes
    # keep dist inf and pred -1. No zone/connector filtering: the
    # tow path is never driven in traffic (the vehicle jumps from
    # the stranded node to the charger), so this only picks the
    # nearest site and its tow travel time.
    n = len(world.nodes)
    dist = [float("inf")] * n
    pred = [-1] * n
    dist[origin_id] = 0.0
    heap = [(0.0, origin_id)]
    while heap:
        d, node = heapq.heappop(heap)
        if d > dist[node] + 1e-12:
            continue
        for link_no in world.nodes[node].outgoing:
            link = world.links[link_no]
            nd = d + float(link.length)
            if nd < dist[link.to_id] - 1e-12:
                dist[link.to_id] = nd
                pred[link.to_id] = link_no
                heapq.heappush(heap, (nd, link.to_id))
    return dist, pred


def _reconstruct_path(world, pred, origin_id, dest_id):
    # Link list from origin to dest along pred links, or [] when
    # origin == dest. pred comes from _distance_tree with a finite
    # distance, so the walk always terminates at the origin.
    links = []
    node = dest_id
    while node != origin_id:
        link_no = pred[node]
        if link_no < 0:
            return []
        links.append(link_no)
        node = world.links[link_no].from_id
    links.reverse()
    return links


class ChargingEngine:
    # Inline charging for tour mode (slice 2): decisions are made
    # pre-trip at leg generation and destination/home charging is
    # applied during the dwell after a leg completes, bounded by
    # the next leg's departure. En-route decisions are executed by
    # slice 3a: generation splices the via path (loading.py) and
    # the vehicle parks at the charger node to charge
    # (charge_enroute, called from the loader). Lives in
    # charging.py alongside the legacy post-pass so the writers
    # and the site/queue state stay in one place.
    def __init__(self, world, config=None):
        cfg = dict(DEFAULT_CONFIG)
        cfg.update(config or {})
        self.config = cfg
        self.world = world
        self.sites, self.by_node = read_charger_sites(world)
        self.events = []       # tour-mode tuples (15 fields)
        self.home_events = []  # subset of events at HOME chargers
        self.stranded = []     # tour-mode tuples (13 fields)
        # Free-flow energy per leg, cached by _leg_energy (below).
        self._leg_energy_cache = {}
        # Free-flow SP network, set by the Simulator from its
        # iteration-0 net (fftt never changes during loading, so
        # reuse is valid); created lazily on first use otherwise.
        self.ff_net = None
        # Nearest-charger map (v5 triggers, v6 en-route decision):
        # built once by build_nearest_map and owned by the engine.
        # v6 builds it on every charging run: the en-route
        # decision needs it for every leg, not only archetype
        # legs, so blank-archetype runs no longer behave as v4
        # (the en-route charger choice changed). When the network
        # cannot support the multi-source runs the map stays None
        # and the callers fall back to per-site searches.
        self.nearest_map = None

    def _ff_net(self):
        if self.ff_net is None:
            self.ff_net = SPNetwork(self.world, use_prevailing=False)
        return self.ff_net

    def build_nearest_map(self):
        # v5/v6: precompute the nearest-charger map (nearest.py).
        # v5 built it only when some leg carried a non-blank
        # archetype, so blank-archetype runs paid nothing and
        # behaved exactly as v4. v6 drops that gate: the en-route
        # decision now looks the charger up in the map for every
        # leg, so the map is built on every charging run. When
        # the network cannot support the multi-source runs, the
        # map stays None and the callers fall back to per-site
        # searches (one stdout line, once).
        self.nearest_map = None
        ff_net = self._ff_net()
        if not multi_run_valid(self.world, ff_net):
            print("nearest-charger map unavailable "
                  "(multi_run_valid false); per-trigger search fallback")
            return
        from .nearest import NearestChargerMap
        self.nearest_map = NearestChargerMap(
            self.world, ff_net, self.sites)

    def _dest_site(self, leg):
        sites = self.by_node.get(leg.dest_node)
        return sites[0] if sites else None

    @staticmethod
    def _home_available(veh, leg):
        return leg.trip_purpose == "home" and veh.home_charger == 1

    def _leg_energy(self, world, leg, consumption):
        # Free-flow energy of a leg's current path, cached per
        # leg: decide_for_leg sums the energies of all later
        # legs for every leg, so without the cache each leg's
        # energy is recomputed once per earlier leg. The cache
        # is keyed by the leg object and validated against the
        # exact link list object, its length, and the
        # consumption: link lists are only ever replaced (via
        # splice, reroute, tow), never edited in place after
        # demand reading, so a changed path always misses and
        # is recomputed. A cached value is the same sum
        # leg_free_flow_energy computes, and the remaining sums
        # re-add cached values in the same order, so the
        # results are exactly identical.
        ary = leg.link_ary
        if ary is None:
            return 0.0
        hit = self._leg_energy_cache.get(id(leg))
        if hit is not None and hit[0] is ary \
                and hit[1] == len(ary) \
                and hit[2] == consumption:
            return hit[3]
        energy = leg_free_flow_energy(world, leg, consumption)
        self._leg_energy_cache[id(leg)] = \
            (ary, len(ary), consumption, energy)
        return energy

    def decide_for_leg(self, world, veh, leg):
        # Pre-trip decision for a leg that is beginning travel.
        # Stores the result on the leg (decision, charger id,
        # target, and the planning energies) and returns it as
        # (decision, site_or_None, target_soc_frac). Non-EVs get
        # "none" with no computation.
        params = world.ev_params(veh)
        if params is None:
            leg.decision = "none"
            leg.decision_charger_id = ""
            leg.decision_target_soc = 0.0
            leg.decision_leg_energy_kwh = 0.0
            leg.decision_remaining_kwh = 0.0
            return ("none", None, 0.0)
        battery, consumption = params
        soc = leg.soc_start_kwh
        leg_energy = self._leg_energy(world, leg, consumption)
        # Planning estimate: later legs' paths were assigned at
        # iteration 0, so their free-flow energies are known now.
        remaining = 0.0
        for later in veh.legs[veh.current_leg:]:
            remaining += self._leg_energy(world, later, consumption)
        dest_site = self._dest_site(leg)
        home_available = self._home_available(veh, leg)
        # Plan-ahead pending via (v5): a stop planned when the
        # previous leg completed, consumed at this leg's
        # generation. Reachability is re-validated with the
        # current starting charge; a via that no longer fits is
        # dropped and the leg falls through to the v4 rules.
        if leg.pending_via is not None:
            site, links_to, links_from, via_time = leg.pending_via
            leg.pending_via = None
            if _links_energy(world, links_to, consumption) \
                    <= soc + 1e-9:
                leg.enroute_via = (links_to, links_from, via_time)
                leg.decision = "enroute"
                leg.decision_charger_id = site.charger_id
                leg.decision_target_soc = leg.enroute_target_soc
                leg.decision_leg_energy_kwh = leg_energy
                leg.decision_remaining_kwh = remaining
                leg.trigger = "plan_ahead"
                leg.trigger_charger_id = site.charger_id
                return ("enroute", site, leg.enroute_target_soc)
            leg.trigger = ""
            leg.trigger_charger_id = ""
        # Opportunistic trigger (v5): a charger within walking
        # tolerance of the DESTINATION, looked up at trip start.
        # The stop executes as an en-route park during the leg
        # (existing charge_enroute machinery); a charger at the
        # destination or at the origin is left to the v4 rules,
        # and no reachable charger means no stop (normal leg).
        if leg.archetype == "opportunistic" and leg.node_size >= 2 \
                and not leg.stranded and not leg.depleted \
                and leg.charger_stop is None:
            found = choose_charger_for_trigger(
                self.nearest_map, "opportunistic", leg.origin_node,
                leg.dest_node, soc, consumption, battery,
                leg.enroute_target_soc,
                walk_tolerance_m=leg.walk_tolerance_m, world=world,
                ff_net=self._ff_net(), veh=veh,
                departure=float(veh.departure))
            if found is not None:
                site, links_to, links_from, via_time = found
                if site.node_id != leg.dest_node and \
                        site.node_id != leg.origin_node:
                    leg.enroute_via = (links_to, links_from, via_time)
                    leg.decision = "enroute"
                    leg.decision_charger_id = site.charger_id
                    leg.decision_target_soc = leg.enroute_target_soc
                    leg.decision_leg_energy_kwh = leg_energy
                    leg.decision_remaining_kwh = remaining
                    leg.trigger = "opportunistic"
                    leg.trigger_charger_id = site.charger_id
                    return ("enroute", site, leg.enroute_target_soc)
        # The en-route choice (v6) is the nearest charger to the
        # leg's origin by free-flow time, looked up in the
        # precomputed nearest-charger map (nearest.py): the same
        # "closest reachable" seam the v5 triggers use
        # (choose_charger_for_trigger, "liquid" branch). The
        # choice is destination-blind and there is no
        # second-nearest attempt: if the nearest charger is not
        # reachable on current SoC there is no stop. A site at
        # the leg's origin or destination node is not an en-route
        # answer either (the via splice needs both legs
        # non-empty), so it also means no stop; the leg then
        # falls through to the destination rule or "none". The
        # lookup runs only when the leg may actually need it:
        # SoC within the margin of the leg's energy need. The
        # site plus its via path is stashed on the leg for
        # generation to splice in (slice 3a) and cleared
        # whenever no en-route stop applies. The v4
        # multi-destination/per-site search (find_enroute_charger
        # in decision.py) is bypassed in v6 and kept only so the
        # choice can be reverted.
        enroute = None
        enroute_via = None
        margin = self.config.get("enroute_margin_frac", 0.1) * battery
        if soc < leg_energy + margin:
            found = choose_charger_for_trigger(
                self.nearest_map, "liquid", leg.origin_node,
                leg.dest_node, soc, consumption, battery,
                leg.enroute_target_soc, world=world,
                ff_net=self._ff_net(), veh=veh,
                departure=float(veh.departure))
            if found is not None:
                site, links_to, links_from, via_time = found
                if site.node_id not in (leg.origin_node,
                                        leg.dest_node):
                    enroute = site
                    enroute_via = (links_to, links_from, via_time)
        leg.enroute_via = enroute_via
        ctx = {
            "soc_kwh": soc,
            "leg_energy_kwh": leg_energy,
            "remaining_energy_kwh": remaining,
            "dest_site": dest_site,
            "home_available": home_available,
            "archetype": leg.archetype,
            "enroute_target_soc": leg.enroute_target_soc,
            "enroute_charger": enroute,
            "battery_kwh": battery,
            "config": self.config,
        }
        decision, site, target = decide_charging(world, veh, leg, ctx)
        leg.decision = decision
        if decision == "enroute":
            leg.decision_charger_id = site.charger_id if site else ""
        elif decision == "destination" and dest_site is None:
            leg.decision_charger_id = "HOME"
        elif decision == "destination" and site is not None:
            leg.decision_charger_id = site.charger_id
        else:
            leg.decision_charger_id = ""
        leg.decision_target_soc = target
        leg.decision_leg_energy_kwh = leg_energy
        leg.decision_remaining_kwh = remaining
        return (decision, site, target)

    def apply_destination_charge(self, world, veh, leg, arrival_time,
                                 departure_next_or_None):
        # Dwell charging for a leg whose decision is "destination",
        # called after the next leg's departure is known. Window is
        # [arrival_time, D_next) for non-terminal legs and
        # [arrival_time, inf) for terminal legs. Stranded legs never
        # reach their destination, so they are not charged here
        # (the caller skips them); a leg already at/above its target
        # records no event and counts as at-target. The at-target
        # check comes before the plug-busy check: a vehicle that
        # needs no charge needs no plug, so a busy plug cannot
        # queue it out.
        # Returns the outcome string, or None if nothing applies.
        if leg.decision != "destination":
            return None
        params = world.ev_params(veh)
        if params is None:
            return None
        battery, _consumption = params
        soc_at_arrival = leg.soc_end_kwh
        site = self._dest_site(leg)
        home_available = self._home_available(veh, leg)
        if site is not None:
            charger_id = site.charger_id
            power = site.power_kw
            node_number = site.node_number
            target_kwh = _site_target_frac(site, self.config) * battery
            if target_kwh - soc_at_arrival <= 0:
                leg.charge_outcome = "at-target"
                return "at-target"
            plug = min(range(site.plugs),
                       key=lambda p: site.plug_free[p])
            start = max(arrival_time, site.plug_free[plug])
            if departure_next_or_None is not None and \
                    start >= departure_next_or_None:
                leg.charge_outcome = "queued-out"
                return "queued-out"
        elif home_available:
            charger_id = "HOME"
            power = self.config["home_power_kw"]
            node_number = world.nodes[leg.dest_node].number
            target_kwh = battery
            if target_kwh - soc_at_arrival <= 0:
                leg.charge_outcome = "at-target"
                return "at-target"
            plug = None
            start = arrival_time
            if departure_next_or_None is not None and \
                    start >= departure_next_or_None:
                leg.charge_outcome = "queued-out"
                return "queued-out"
        else:
            return None
        energy_needed = target_kwh - soc_at_arrival
        time_needed = energy_needed / power * 60.0
        if departure_next_or_None is None:
            charge_time = time_needed
        else:
            charge_time = min(time_needed,
                              departure_next_or_None - start)
        if charge_time <= 0:
            leg.charge_outcome = "queued-out"
            return "queued-out"
        energy = charge_time * power / 60.0
        soc_after = soc_at_arrival + energy
        end = start + charge_time
        if plug is not None:
            site.plug_free[plug] = end
            leg.dwell_plug_index = plug
        veh.soc_kwh = soc_after
        leg.soc_end_kwh = soc_after
        outcome = "charged-partial" if charge_time < time_needed \
            else "charged-destination"
        leg.charge_outcome = outcome
        event = (veh.agent_id, veh.tour_id, charger_id, node_number,
                 arrival_time, start - arrival_time, start, end,
                 energy, soc_at_arrival, soc_after, power,
                 leg.leg_index, 0, 0)
        self.events.append(event)
        if plug is not None:
            # Keep the per-site event list in sync so the summary
            # writer (which reads site.events) sees tour charges.
            site.events.append(event)
        if charger_id == "HOME":
            self.home_events.append(event)
        veh.last_dwell_charge = (charger_id, leg.dest_node, power,
                                 soc_after, end)
        return outcome

    def charge_at_site_during_dwell(self, world, veh, leg, site,
                                    arrival_time,
                                    departure_next_or_None,
                                    target_soc_frac):
        # Forced dwell charge at a named site (v5 plan-ahead):
        # apply_destination_charge mechanics, but the site and
        # target are given explicitly instead of coming from the
        # leg's "destination" decision. The caller only reaches
        # here when no dwell charge actually ran: a completable
        # trigger leg with a charger at its arrival node always
        # decided "destination" (v4 rule 2 fires whenever soc <
        # remaining, which is exactly the plan-ahead shortfall),
        # so the reachable path is decision "destination" with
        # outcome "queued-out" (plug busy past the next
        # departure), not a "none" decision. Window is [arrival,
        # D_next)
        # (inf for a terminal leg, which cannot happen here: a
        # trigger needs a next leg). Target is target_soc_frac *
        # battery (the leg's enroute target, not the site tier
        # target). Same event shape as a dwell charge (leg_index
        # = the just-completed leg, charged_en_route = 0), same
        # charge_outcome values, same plug recording. Returns the
        # outcome string.
        params = world.ev_params(veh)
        if params is None:
            return None
        battery, _consumption = params
        soc_at_arrival = leg.soc_end_kwh
        target_kwh = target_soc_frac * battery
        if target_kwh - soc_at_arrival <= 0:
            leg.charge_outcome = "at-target"
            return "at-target"
        plug = min(range(site.plugs),
                   key=lambda p: site.plug_free[p])
        start = max(arrival_time, site.plug_free[plug])
        if departure_next_or_None is not None and \
                start >= departure_next_or_None:
            leg.charge_outcome = "queued-out"
            return "queued-out"
        energy_needed = target_kwh - soc_at_arrival
        time_needed = energy_needed / site.power_kw * 60.0
        if departure_next_or_None is None:
            charge_time = time_needed
        else:
            charge_time = min(time_needed,
                              departure_next_or_None - start)
        if charge_time <= 0:
            leg.charge_outcome = "queued-out"
            return "queued-out"
        energy = charge_time * site.power_kw / 60.0
        soc_after = soc_at_arrival + energy
        end = start + charge_time
        site.plug_free[plug] = end
        leg.dwell_plug_index = plug
        veh.soc_kwh = soc_after
        leg.soc_end_kwh = soc_after
        outcome = "charged-partial" if charge_time < time_needed \
            else "charged-destination"
        leg.charge_outcome = outcome
        event = (veh.agent_id, veh.tour_id, site.charger_id,
                 site.node_number, arrival_time,
                 start - arrival_time, start, end, energy,
                 soc_at_arrival, soc_after, site.power_kw,
                 leg.leg_index, 0, 0)
        self.events.append(event)
        site.events.append(event)
        if site.charger_id == "HOME":
            self.home_events.append(event)
        veh.last_dwell_charge = (site.charger_id, leg.dest_node,
                                 site.power_kw, soc_after, end)
        return outcome

    def try_stay_and_finish(self, world, veh, leg, d_next):
        # Stay-and-finish check (slice 3b): prefer charging where
        # the vehicle already is over detouring to an en-route
        # charger. Runs at stage-0 generation, after decide_for_leg
        # and before the via-path splice, only when the vehicle
        # dwell-charged at this leg's origin node, the leg decided
        # "enroute", and the leg is dwell-based (fixed legs never
        # stay: they leave on schedule). Returns the stay end time
        # when the vehicle stays, else None. The stay itself is
        # applied here (plug, event, SoC); the caller re-schedules
        # the leg for the stay end and does not generate it now.
        # Queue estimate is 0 throughout: only the plug's own
        # free time can delay the stay, never another vehicle.
        # Trigger legs (v5) never stay: the committed via is kept,
        # and for plan-ahead legs the re-decision gate below would
        # consume the pending via and then discard it.
        if getattr(leg, "trigger", ""):
            return None
        if veh.last_dwell_charge is None or leg.decision != "enroute" \
                or leg.departure_mode != "dwell":
            return None
        if veh.current_leg == 0 or leg.enroute_via is None:
            return None
        charger_id, node_id, power_here, _soc_after, _end = \
            veh.last_dwell_charge
        if node_id != leg.origin_node:
            return None
        params = world.ev_params(veh)
        if params is None:
            return None
        battery, consumption = params
        target_kwh = leg.decision_target_soc * battery
        e_needed = max(0.0, target_kwh - veh.soc_kwh)
        if e_needed <= 0:
            return None
        enroute_site = next(
            (s for s in self.sites
             if s.charger_id == leg.decision_charger_id), None)
        if enroute_site is None:
            return None
        # The leg still holds its direct path here (the splice has
        # not run), so the direct free-flow time and energy are
        # read off it; the detour is via time minus direct time.
        direct_time = 0.0
        for entry in leg.link_ary[:-1]:
            if entry.link_no == 65530:
                continue
            direct_time += float(world.links[entry.link_no].fftt)
        _links_to, _links_from, via_time = leg.enroute_via
        detour_time = via_time - direct_time
        time_here = e_needed / power_here * 60.0
        time_enroute = detour_time + \
            e_needed / enroute_site.power_kw * 60.0
        if time_here > time_enroute:
            return None
        # The stay must suffice to complete the leg: charging here
        # to the target and then driving direct must be possible.
        if leg_free_flow_energy(world, leg, consumption) > target_kwh:
            return None
        # Same plug the dwell charge used (the previous leg's); a
        # HOME charge has no plug and starts at D_next.
        prev_leg = veh.legs[veh.current_leg - 1]
        if charger_id == "HOME":
            site_here = None
            plug = None
            start = d_next
        else:
            site_here = next(
                (s for s in self.sites if s.charger_id == charger_id),
                None)
            if site_here is None or prev_leg.dwell_plug_index is None:
                return None
            plug = prev_leg.dwell_plug_index
            start = max(d_next, site_here.plug_free[plug])
        stay_end = start + e_needed / power_here * 60.0
        if stay_end - d_next > time_enroute:
            return None
        # Gate: tentative re-decision at the target charge, before
        # applying anything. decide_for_leg reads the leg's
        # starting charge, so both it and the vehicle's charge are
        # set to the target for the call and the vehicle's charge
        # is restored right after. A re-decision that is still
        # "enroute" means the target sits within the en-route
        # margin of the leg's need, so staying would not change
        # the decision: restore the saved fields and do not stay.
        saved = (leg.decision, leg.decision_charger_id,
                 leg.decision_target_soc, leg.decision_leg_energy_kwh,
                 leg.decision_remaining_kwh, leg.enroute_via)
        soc_before = veh.soc_kwh
        soc_start_before = leg.soc_start_kwh
        veh.soc_kwh = target_kwh
        leg.soc_start_kwh = target_kwh
        redecision, _site, _target = self.decide_for_leg(
            world, veh, leg)
        veh.soc_kwh = soc_before
        if redecision == "enroute":
            (leg.decision, leg.decision_charger_id,
             leg.decision_target_soc, leg.decision_leg_energy_kwh,
             leg.decision_remaining_kwh, leg.enroute_via) = saved
            leg.soc_start_kwh = soc_start_before
            return None
        # Apply the stay: take the plug, record a separate charging
        # event against the previous leg (the dwell that is being
        # extended), and move the charge to the target. The leg is
        # not generated now; the caller re-schedules it.
        if plug is not None:
            site_here.plug_free[plug] = stay_end
        node_number = world.nodes[node_id].number
        event = (veh.agent_id, veh.tour_id, charger_id, node_number,
                 d_next, start - d_next, start, stay_end, e_needed,
                 soc_before, target_kwh, power_here,
                 prev_leg.leg_index, 0, 0)
        self.events.append(event)
        if charger_id == "HOME":
            self.home_events.append(event)
        elif site_here is not None:
            site_here.events.append(event)
        veh.soc_kwh = target_kwh
        leg.soc_start_kwh = target_kwh
        veh.last_dwell_charge = (charger_id, node_id, power_here,
                                 target_kwh, stay_end)
        return stay_end

    def charge_enroute(self, world, veh, leg, arrival_time):
        # En-route charge at a stop the vehicle just reached
        # (slice 3a, called from the loader when the vehicle
        # completes the link into the stop node, after that link's
        # energy was deducted). Returns the charge end time when
        # the vehicle must park until then, or None when it drives
        # through: the stop is marked done either way. A vehicle
        # already at/above its target needs no charge and no plug,
        # so a busy plug cannot hold it up. Otherwise the
        # earliest-free plug is taken, the charge runs to the
        # target (no dwell window bounds an en-route stop), and
        # the event is recorded with charged_en_route = 1. The
        # caller parks the vehicle until the returned end time and
        # sets veh.soc_kwh to the target here: nothing reads SoC
        # while it is parked.
        stop = leg.charger_stop
        site = stop["site"]
        params = world.ev_params(veh)
        battery, _consumption = params
        soc_at_stop = veh.soc_kwh
        target_kwh = stop["target_soc"] * battery
        stop["done"] = True
        if target_kwh - soc_at_stop <= 0:
            return None
        plug = min(range(site.plugs),
                   key=lambda p: site.plug_free[p])
        start = max(arrival_time, site.plug_free[plug])
        energy_needed = target_kwh - soc_at_stop
        charge_time = energy_needed / site.power_kw * 60.0
        end = start + charge_time
        site.plug_free[plug] = end
        veh.soc_kwh = target_kwh
        leg.charged_en_route = True
        leg.enroute_plug_index = plug
        event = (veh.agent_id, veh.tour_id, site.charger_id,
                 site.node_number, arrival_time, start - arrival_time,
                 start, end, energy_needed, soc_at_stop, target_kwh,
                 site.power_kw, leg.leg_index, 1, 0)
        self.events.append(event)
        site.events.append(event)
        return end

    def tow_vehicle(self, world, veh, leg, stranded_node_id,
                    stranded_time, stranded_link):
        # Tow for a leg that stranded mid-leg (slice 4). Returns a
        # plan dict, or None when there is no viable tow: no sites
        # at all, no distance path from the stranded node to any
        # site, or no free-flow path from the chosen site to the
        # leg's destination. The nearest site is by free-flow
        # DISTANCE (Dijkstra over link.length), not time; ties
        # break toward higher power, then file order. A site at
        # the destination node IS a tow destination for a stranded
        # vehicle (it cannot drive there): the site->dest leg is
        # trivially satisfied with no drive after charging, and
        # the leg completes when the charge ends. The plan carries
        # the charger->dest free-flow links (empty for a
        # destination site), the charge end time, and the target
        # charge; the caller (loading.py) splices the path, parks
        # the vehicle until the charge end, and sets the tow
        # flags. The charging records are made here: the tow
        # charge event (tow_charge = 1, charged_en_route = 0) and
        # the stranded row with its tow fields. Timeline (v1
        # assumptions): the truck is dispatched at the stranding
        # time T and reaches the vehicle at T + tow_dispatch_min
        # (the blockage window on the stranded link); it tows
        # along the distance-shortest path (travel_time_min of
        # free-flow time) to the charger; the vehicle queues for
        # the earliest-free plug and charges from 0 to
        # tow_target_soc * battery at the site's power.
        params = world.ev_params(veh)
        if params is None:
            return None
        battery, consumption = params
        dist, pred = _distance_tree(world, stranded_node_id)
        ff_net = self._ff_net()
        best = None
        best_key = None
        best_from = None
        for idx, site in enumerate(self.sites):
            d = dist[site.node_id]
            if d == float("inf"):
                continue
            if site.node_id == leg.dest_node:
                # Charger at the destination: the site->dest leg
                # is trivially satisfied (no drive after charging),
                # so the site is viable without a path search --
                # find_best_path returns no path for origin == dest.
                path_from = []
            else:
                site_zone = world.nodes[site.node_id].zone_id
                node_size, path_from, _cost = find_best_path(
                    world, ff_net, site_zone, site.node_id,
                    int(stranded_time), leg.dest_zone, leg.dest_node,
                    veh.demand_type, veh.vot)
                if node_size < 2:
                    continue
            key = (d, -site.power_kw, idx)
            if best_key is None or key < best_key:
                best = site
                best_key = key
                best_from = path_from
        if best is None:
            return None
        site = best
        path_to = _reconstruct_path(world, pred, stranded_node_id,
                                    site.node_id)
        travel_time = sum(float(world.links[ln].fftt)
                          for ln in path_to)
        target_kwh = self.config["tow_target_soc"] * battery
        tow_start = stranded_time + self.config["tow_dispatch_min"]
        arrival_charger = tow_start + travel_time
        plug = min(range(site.plugs),
                   key=lambda p: site.plug_free[p])
        start = max(arrival_charger, site.plug_free[plug])
        charge_time = target_kwh / site.power_kw * 60.0
        end = start + charge_time
        site.plug_free[plug] = end
        event = (veh.agent_id, veh.tour_id, site.charger_id,
                 site.node_number, arrival_charger,
                 start - arrival_charger, start, end, target_kwh,
                 0.0, target_kwh, site.power_kw, leg.leg_index, 0, 1)
        self.events.append(event)
        site.events.append(event)
        # Trip energy is the free-flow energy of the path as it
        # stood at this stranding (the live path, which a reroute
        # or an earlier tow may already have rewritten).
        # soc_start_kwh is the LEG-START SoC (the charge at
        # departure), not the charge at this stranding: a second
        # stranding on the same leg repeats the leg-start value,
        # not the tow-restored charge.
        trip_energy = 0.0
        for entry in veh.link_ary[:-1]:
            if entry.link_no == 65530:
                continue
            link = world.links[entry.link_no]
            trip_energy += float(link.length) * KM_PER_MILE \
                * consumption
        blockage = "%d->%d" % (
            world.nodes[stranded_link.from_id].number,
            world.nodes[stranded_link.to_id].number)
        self.stranded.append(
            (veh.agent_id, veh.tour_id,
             world.nodes[stranded_node_id].number, trip_energy,
             leg.soc_start_kwh, leg.leg_index, site.charger_id,
             tow_start, end, target_kwh, blockage, stranded_time,
             self.config["tow_dispatch_min"]))
        leg.stranded_rows_recorded += 1
        return {"site": site, "path_from_links": best_from,
                "charge_end": end, "target_kwh": target_kwh,
                "tow_start": tow_start,
                "arrival_charger": arrival_charger}

    def record_stranded(self, world, veh, leg):
        params = world.ev_params(veh)
        if params is None:
            return
        _battery, consumption = params
        # A stranding already recorded by a tow needs no second
        # row: tow_vehicle appends its row at tow time (with the
        # tow fields), and this only covers strandings that have
        # no row yet -- an untowed stranding, or a later stranding
        # after the tow cap. The counters live on the leg:
        # stranding_count rises at every crossing, and
        # stranded_rows_recorded rises at every row written.
        if leg.stranding_count <= leg.stranded_rows_recorded:
            return
        # Per-link deduction (slice 3a) already recorded the exact
        # crossing node; the walk below is only a fallback for a
        # stranded flag set before that deduction existed.
        node = leg.stranded_node
        if node is None:
            node = stranding_point(world, leg, leg.soc_start_kwh,
                                   consumption)
        if node is None:
            node = world.nodes[leg.dest_node].number
        trip_energy = leg_free_flow_energy(world, leg, consumption)
        # soc_start_kwh is the LEG-START SoC (see tow_vehicle):
        # an untowed stranding after the tow cap repeats it too.
        self.stranded.append((veh.agent_id, veh.tour_id, node,
                              trip_energy, leg.soc_start_kwh,
                              leg.leg_index, "", 0.0, 0.0, 0.0, "",
                              0.0, 0.0))
        leg.stranded_rows_recorded += 1

    def finalize(self, world, output_dir):
        _write_events(output_dir, self.events, tour=True)
        _write_summary(world, output_dir, self.sites,
                       home_events=self.home_events,
                       home_power_kw=self.config["home_power_kw"])
        _write_stranded(output_dir, self.stranded, tour=True)
        _write_decisions(output_dir, world, self)
        return {
            "charges": len(self.events),
            "stranded": len(self.stranded),
            "sites": len(self.sites),
            "total_energy_kwh": sum(e[8] for e in self.events),
        }


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


def _write_events(output_dir, events, tour=False):
    header = "agent_id,tour_id,charger_id,node_id,arrival_time_in_min," \
        "wait_in_min,charge_start_in_min,charge_end_in_min," \
        "energy_kwh,soc_before_kwh,soc_after_kwh,charger_power_kw"
    if tour:
        header += ",leg_index,charged_en_route,tow_charge"
    lines = [header + "\n"]
    for e in events:
        if tour:
            lines.append(
                "%d,%d,%s,%d,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,"
                "%d,%d,%d\n" % e)
        else:
            lines.append(
                "%d,%d,%s,%d,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f\n"
                % e)
    with open(os.path.join(output_dir, "output_charging.csv"),
              "w", newline="", encoding="utf-8") as fh:
        fh.writelines(lines)


def _write_summary(world, output_dir, sites, home_events=None,
                   home_power_kw=11.0):
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
    if home_events:
        # HOME row: home charging has no site node (0), no shared
        # plugs and no queue. plugs is the number of distinct agents
        # that home-charged; waits, max queue and utilization are
        # rendered 0.0/0 by construction (see README).
        agents = {e[0] for e in home_events}
        total_energy = sum(e[8] for e in home_events)
        lines.append(
            "HOME,0,%.1f,%d,home,%d,%.3f,%.2f,%.2f,0,%.4f\n"
            % (home_power_kw, len(agents), len(home_events),
               total_energy, 0.0, 0.0, 0.0))
    with open(os.path.join(output_dir, "output_charger_summary.csv"),
              "w", newline="", encoding="utf-8") as fh:
        fh.writelines(lines)


def _write_stranded(output_dir, stranded, tour=False):
    header = "agent_id,tour_id,stranded_node_id,trip_energy_kwh," \
        "soc_start_kwh"
    if tour:
        # Tow fields are empty / 0.0 when the leg was not towed.
        header += ",leg_index,tow_charger_id,tow_start_in_min," \
            "tow_end_in_min,charge_kwh,tow_blockage_link," \
            "tow_blockage_start_in_min,tow_blockage_duration_min"
    lines = [header + "\n"]
    for row in stranded:
        if tour:
            lines.append("%d,%d,%d,%.3f,%.3f,%d,%s,%.2f,%.2f,%.3f,"
                         "%s,%.2f,%.2f\n" % row)
        else:
            lines.append("%d,%d,%d,%.3f,%.3f\n" % row)
    with open(os.path.join(output_dir, "output_stranded.csv"),
              "w", newline="", encoding="utf-8") as fh:
        fh.writelines(lines)


def _write_decisions(output_dir, world, engine):
    # One row per leg (vehicles by agent_id, legs by leg_index).
    # Outcome precedence: stranded > queued-out > charged-partial >
    # charged-destination > at-target > completed. day follows outputs.py: the
    # actual departure when the leg departed, else the scheduled one.
    # v5: trigger columns appear only when at least one leg in
    # the run recorded a trigger, so blank-archetype runs stay
    # byte-identical to v4.
    has_trigger = False
    for veh in world.vehicles.values():
        for leg in veh.legs:
            if leg.trigger:
                has_trigger = True
                break
        if has_trigger:
            break
    lines = ["agent_id,leg_index,day,soc_at_departure_kwh,"
             "leg_energy_kwh,remaining_energy_kwh,"
             "destination_charger,decision,enroute_charger_id,"
             "target_soc,outcome"
             + (",trigger,trigger_charger_id" if has_trigger else "")
             + "\n"]
    for agent_id in sorted(world.vehicles):
        veh = world.vehicles[agent_id]
        for leg in sorted(veh.legs, key=lambda l: l.leg_index):
            departed = leg.complete or leg.departure_actual is not None
            actual_dep = leg.departure_actual if departed \
                else float(leg.scheduled_departure)
            day = int(actual_dep // 1440)
            dest_charger = 1 if (engine.by_node.get(leg.dest_node)
                                 or engine._home_available(veh, leg)) \
                else 0
            enroute_id = leg.decision_charger_id \
                if leg.decision == "enroute" else ""
            if leg.stranded:
                outcome = "stranded"
            elif leg.charge_outcome == "queued-out":
                outcome = "queued-out"
            elif leg.charge_outcome == "charged-partial":
                outcome = "charged-partial"
            elif leg.charge_outcome == "charged-destination":
                outcome = "charged-destination"
            elif leg.charge_outcome == "at-target":
                outcome = "at-target"
            else:
                outcome = "completed"
            if has_trigger:
                lines.append(
                    "%d,%d,%d,%.3f,%.3f,%.3f,%d,%s,%s,%.3f,%s,%s,%s\n"
                    % (veh.agent_id, leg.leg_index, day,
                       float(leg.soc_start_kwh),
                       float(leg.decision_leg_energy_kwh),
                       float(leg.decision_remaining_kwh), dest_charger,
                       leg.decision, enroute_id,
                       float(leg.decision_target_soc), outcome,
                       leg.trigger, leg.trigger_charger_id))
            else:
                lines.append(
                    "%d,%d,%d,%.3f,%.3f,%.3f,%d,%s,%s,%.3f,%s\n"
                    % (veh.agent_id, leg.leg_index, day,
                       float(leg.soc_start_kwh),
                       float(leg.decision_leg_energy_kwh),
                       float(leg.decision_remaining_kwh), dest_charger,
                       leg.decision, enroute_id,
                       float(leg.decision_target_soc), outcome))
    with open(os.path.join(output_dir, "output_decisions.csv"),
              "w", newline="", encoding="utf-8") as fh:
        fh.writelines(lines)
