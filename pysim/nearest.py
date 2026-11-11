# Nearest-charger map (v5, Phase 1 archetype triggers).
#
# One object, built once at startup and owned by the ChargingEngine:
# for every node it stores the time-nearest charger site, the
# free-flow travel time and network distance to it, and the first
# link of the time-shortest path node -> charger. Triggers then
# answer "which charger, and how do I get there" with an O(1)
# lookup plus one shortest-path call for the charger -> destination
# leg (see decision.choose_charger_for_trigger); the map alone
# cannot supply that leg, because it depends on the destination.
#
# The map is built from the free-flow network (ff_net, the
# iteration-0 network): congestion is ignored, by design. It is
# built only when multi_run_valid holds (no connector links, TD
# table constant across intervals); otherwise the engine leaves
# nearest_map None and choose_charger_for_trigger falls back to a
# per-site search.
from collections import deque

from .cpp import f32, floating_less
from .decision import KM_PER_MILE
from .sp import MAX_SPLABEL


def _best_site_per_node(sites):
    # One source entry per charger node: where several sites share
    # a node, the site with the highest power_kw wins, tie-break
    # lowest file index. Returns {node_id: site_index}.
    best = {}
    for idx, site in enumerate(sites):
        cur = best.get(site.node_id)
        if cur is None or site.power_kw > sites[cur].power_kw:
            best[site.node_id] = idx
    return best


def _source_key(sites, site_index):
    # Ordering key for the relaxation tie-break: a smaller key is
    # a better source (higher power first, then lower file index).
    site = sites[site_index]
    return (-site.power_kw, site_index)


class NearestChargerMap:
    # Parallel lists per node (internal id 0..N-1):
    #   site_index: index into engine.sites, or -1 when no charger
    #     is reachable from the node.
    #   time_min: free-flow travel time (min) node -> nearest
    #     charger, inf when none.
    #   dist_km: network distance (km) along the time-shortest
    #     path node -> charger, inf when none.
    #   pred_to: link_no of the first link on the time-shortest
    #     path node -> charger, -1 when none (or at a charger).
    #   pred_from: link_no of the predecessor link on the
    #     time-shortest path charger -> node, -1 when none. Kept
    #     for symmetry with pred_to; the trigger splice does not
    #     use it (the charger -> destination leg is computed per
    #     trigger with one find_best_path call, because the
    #     forward tree's per-node source is in general a different
    #     charger from the reverse tree's source for that node).
    def __init__(self, world, ff_net, sites):
        self.world = world
        self.ff_net = ff_net
        self.sites = list(sites)
        n = len(world.nodes)
        self.site_index = [-1] * n
        self.time_min = [float("inf")] * n
        self.dist_km = [float("inf")] * n
        self.pred_to = [-1] * n
        self.pred_from = [-1] * n
        # v7: degraded lookups. path_to_charger never raises for a
        # single bad node: a broken chain degrades to "no reachable
        # charger" (None) and is counted here, so the run continues
        # and the count is reported at the end of the run. The
        # _build_reverse consistency rule below should keep this at
        # zero; a nonzero count means the map still has a hole.
        self.broken_lookups = 0
        if not self.sites:
            return
        best = _best_site_per_node(self.sites)
        self._build_reverse(world, ff_net, best)
        self._build_forward(world, ff_net, best)

    def nearest_site(self, node_id):
        # ChargerSite nearest (by free-flow time) to node_id, or
        # None when no charger is reachable from it.
        idx = self.site_index[node_id]
        return self.sites[idx] if idx >= 0 else None

    def path_to_charger(self, node_id):
        # [link_no, ...] along the time-shortest path node_id ->
        # nearest charger, in travel order; [] when the node IS a
        # charger node; None when no charger is reachable from the
        # node. v7: a broken predecessor chain ALSO returns None
        # (and is counted in broken_lookups) instead of raising
        # ProgramStop -- one bad node must never kill a run. The
        # caller treats None as "no reachable charger" and falls
        # through to existing behavior.
        idx = self.site_index[node_id]
        if idx < 0:
            return None
        site = self.sites[idx]
        if node_id == site.node_id:
            return []
        links = []
        node = node_id
        while node != site.node_id:
            link_no = self.pred_to[node]
            if link_no < 0 or len(links) > len(self.world.nodes):
                self.broken_lookups += 1
                return None
            links.append(link_no)
            node = self.world.links[link_no].to_id
        return links

    def _build_reverse(self, world, net, best):
        # Multi-source label-correcting run on the reversed
        # network: every charger node is a source with label 0,
        # and incoming links are traversed backwards, exactly as
        # find_best_paths_reverse traverses them (same deque
        # discipline, same floating_less, same f32 accumulation,
        # same horizon cap). Result per node: nearest charger,
        # time_min, dist_km, and pred_to (the link from the node
        # toward the charger).
        nodes = world.nodes
        links = world.links
        n = len(nodes)
        label_time = [MAX_SPLABEL] * n
        label_cost = [MAX_SPLABEL] * n
        label_dist = [float("inf")] * n
        source = [-1] * n
        status = [0] * n
        se = deque()
        for idx, site in enumerate(self.sites):
            node_id = site.node_id
            if best.get(node_id) != idx:
                continue
            label_time[node_id] = f32(0)
            label_cost[node_id] = f32(0)
            label_dist[node_id] = 0.0
            source[node_id] = idx
            self.site_index[node_id] = idx
            self.time_min[node_id] = 0.0
            self.dist_km[node_id] = 0.0
            se.append(node_id)
            status[node_id] = 1
        while se:
            cur = se.popleft()
            status[cur] = 2
            for link_no in nodes[cur].incoming:
                link = links[link_no]
                from_id = link.from_id
                interval = world.interval_index(label_time[cur])
                td = net.td_time[link_no][interval]
                new_time = f32(label_time[cur] + td)
                new_cost = f32(label_cost[cur] + td)
                new_dist = label_dist[cur] + float(link.length) \
                    * KM_PER_MILE
                new_key = _source_key(self.sites, source[cur])
                old_cost = label_cost[from_id]
                better = floating_less(new_cost, old_cost)
                if not better and source[from_id] >= 0 \
                        and not floating_less(new_cost, old_cost) \
                        and not floating_less(old_cost, new_cost):
                    # Equal cost (within the 0.01 quantisation):
                    # keep the better source (higher power, then
                    # lower file index). The key strictly
                    # decreases, so re-queueing terminates.
                    better = new_key < _source_key(
                        self.sites, source[from_id])
                if not better and self.pred_to[from_id] == link_no:
                    # v7 consistency (root cause of the v6 crash):
                    # from_id's stored path runs through cur on
                    # exactly this link. If cur's source changed
                    # since from_id was derived from it -- via the
                    # equal-cost tie-break, or a strict improvement
                    # smaller than the 0.01 quantisation -- then
                    # from_id's stored site no longer matches the
                    # charger its pred_to chain actually leads to,
                    # and the chain can end at a different charger
                    # node (whose pred_to is -1) and break. The
                    # same staleness hits dist_km when cur's path
                    # changed at equal time (a longer-distance
                    # route), which would corrupt the reachability
                    # check that reads dist_km. Re-derive from_id
                    # from cur's current label even though the
                    # recomputed cost is not "better": the cost
                    # can only move within the 0.01 quantisation,
                    # so this cannot undo a strictly better label,
                    # and the re-queue below propagates the fix to
                    # from_id's own dependents.
                    if source[cur] != source[from_id] or \
                            abs(new_dist - label_dist[from_id]) > 1e-9:
                        better = True
                if not better:
                    continue
                if new_time > world.horizon - 1:
                    new_time = f32(world.horizon - 1)
                label_time[from_id] = new_time
                label_cost[from_id] = new_cost
                label_dist[from_id] = new_dist
                source[from_id] = source[cur]
                self.pred_to[from_id] = link_no
                self.site_index[from_id] = source[cur]
                self.time_min[from_id] = float(new_cost)
                self.dist_km[from_id] = new_dist
                if status[from_id] == 2:
                    se.appendleft(from_id)
                    status[from_id] = 1
                elif status[from_id] == 0:
                    se.append(from_id)
                    status[from_id] = 1

    def _build_forward(self, world, net, best):
        # Multi-source label-correcting run on the forward
        # network: every charger node is a source with label 0,
        # and outgoing links are traversed, exactly as
        # find_best_paths_forward traverses them. Result per node:
        # pred_from, the predecessor link on the time-shortest
        # path from a charger to the node. The per-node source
        # here is in general a different charger from the reverse
        # run's source for that node, so this tree is not used
        # for path reconstruction (see the class note); it is
        # kept so the structure carries both directions.
        nodes = world.nodes
        links = world.links
        n = len(nodes)
        label_time = [MAX_SPLABEL] * n
        label_cost = [MAX_SPLABEL] * n
        source = [-1] * n
        status = [0] * n
        se = deque()
        for idx, site in enumerate(self.sites):
            node_id = site.node_id
            if best.get(node_id) != idx:
                continue
            label_time[node_id] = f32(0)
            label_cost[node_id] = f32(0)
            source[node_id] = idx
            se.append(node_id)
            status[node_id] = 1
        while se:
            from_id = se.popleft()
            status[from_id] = 2
            for link_no in nodes[from_id].outgoing:
                link = links[link_no]
                to_id = link.to_id
                interval = world.interval_index(label_time[from_id])
                td = net.td_time[link_no][interval]
                new_time = f32(label_time[from_id] + td)
                new_cost = f32(label_cost[from_id] + td)
                new_key = _source_key(self.sites, source[from_id])
                old_cost = label_cost[to_id]
                better = floating_less(new_cost, old_cost)
                if not better and source[to_id] >= 0 \
                        and not floating_less(new_cost, old_cost) \
                        and not floating_less(old_cost, new_cost):
                    better = new_key < _source_key(
                        self.sites, source[to_id])
                if not better:
                    continue
                if new_time > world.horizon - 1:
                    new_time = f32(world.horizon - 1)
                label_time[to_id] = new_time
                label_cost[to_id] = new_cost
                source[to_id] = source[from_id]
                self.pred_from[to_id] = link_no
                if status[to_id] == 2:
                    se.appendleft(to_id)
                    status[to_id] = 1
                elif status[to_id] == 0:
                    se.append(to_id)
                    status[to_id] = 1
