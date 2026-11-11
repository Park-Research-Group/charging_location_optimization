# Time-dependent shortest path on the physical network:
# DTANetworkForSP::BuildPhysicalNetwork + FindBestPathWithVOTForSingleAgent
# (ShortestPath.cpp). TD link times are float32 per assignment interval.
from collections import deque

import numpy as np

from .cpp import f32, floating_less

MAX_SPLABEL = f32(9999.0)


class SPNetwork:
    def __init__(self, world, use_prevailing=False, current_time=0.0):
        self.world = world
        # 2D float32 table, one row per link (link_no is the row index:
        # links are numbered 0..n-1 in read order) and one column per
        # assignment interval. Same 0.01 default in every cell as the
        # old list of lists; rebuild fills the visited columns below.
        self.td_time = np.full(
            (len(world.links), world.num_periods), f32(0.01),
            dtype=np.float32)
        self.rebuild(use_prevailing, current_time)

    def rebuild(self, use_prevailing, current_time):
        # BuildPhysicalNetwork TD loop: t walks the assignment intervals of
        # [start, horizon); each visited interval gets the link's current
        # travel time (simulated MOE average, or prevailing time when the
        # network carries current information), times the link-type bias.
        # The per-link value does not depend on t (travel_time_by_min
        # returns the link's free-flow time; prevailing_tt is a link
        # attribute), so compute it once per link and broadcast it
        # across the visited intervals in a single assignment. The
        # visited set itself is computed once per rebuild by replaying
        # the t-walk exactly as before; intervals the walk never lands
        # on keep the 0.01 default, exactly as they did when the table
        # was a list of lists. The per-link arithmetic is the same
        # scalar float32 multiply + 0.01 clamp as the old loop, in
        # link order, so every written cell is bit-identical to it.
        world = self.world
        visited = []
        t = world.start
        while t < world.horizon:
            visited.append(world.interval_index(t))
            t += world.interval_length(t)
        if not visited:
            return
        cols = np.array(visited, dtype=np.intp)
        values = []
        for link in world.links:
            bias = world.link_type_of(link.link_type).bias
            if use_prevailing:
                avg = link.prevailing_tt
            else:
                avg = world.travel_time_by_min(
                    link, world.start,
                    world.interval_length(world.start))
            avg = f32(avg * bias)
            if avg < f32(0.01):
                avg = f32(0.01)
            values.append(avg)
        self.td_time[:, cols] = \
            np.array(values, dtype=np.float32)[:, None]

    def update_current_travel_time(self, current_time):
        # DTANetworkForSP::UpdateCurrentTravelTime: refresh TD times from
        # CurrentTime forward using each link's prevailing travel time.
        # Same shape as rebuild: the t-walk runs in the original f32
        # clock (t advances by whole interval lengths from
        # f32(current_time)), so replay it once to collect the visited
        # intervals, then broadcast each link's prevailing-time value
        # across them in a single assignment. The per-link arithmetic
        # is the same scalar float32 multiply + 0.01 clamp as before.
        world = self.world
        visited = []
        t = f32(current_time)
        while t < world.horizon:
            visited.append(world.interval_index(t))
            t = f32(t + f32(world.interval_length(t)))
        if not visited:
            return
        cols = np.array(visited, dtype=np.intp)
        values = []
        for link in world.links:
            bias = world.link_type_of(link.link_type).bias
            avg = f32(link.prevailing_tt * bias)
            if avg < f32(0.01):
                avg = f32(0.01)
            values.append(avg)
        self.td_time[:, cols] = \
            np.array(values, dtype=np.float32)[:, None]


def multi_run_valid(world, net):
    # Whether the two multi-destination runs below reproduce
    # per-site find_best_path calls. Two preconditions, both
    # checked once and cached on the net: (1) no connector ("c")
    # links -- find_best_path's zone filter depends on the
    # query's origin/destination zones, so a network with
    # connectors needs the per-site queries; (2) the TD table is
    # constant across intervals -- the reverse run and the
    # per-site departure times then cannot change link times.
    # The check compares the first, middle and last columns for
    # every link, the same tripwire loading.py uses for its SP
    # cache. A net that fails either check falls back to the
    # per-site loop in decision.py.
    cached = getattr(net, "_multi_valid", None)
    if cached is not None:
        return cached
    valid = True
    for link in world.links:
        if "c" in link.type_code:
            valid = False
            break
    if valid:
        rows = net.td_time
        n_periods = world.num_periods
        mid = n_periods // 2
        last = n_periods - 1
        for row in rows:
            if row[0] != row[mid] or row[0] != row[last]:
                valid = False
                break
    try:
        net._multi_valid = valid
    except AttributeError:
        pass
    return valid


def _extract_path(pred_node, pred_link, origin, destination):
    # Link list origin -> destination along the predecessor
    # links, mirroring find_best_path's extraction (including
    # its 3000-link cap). [] when the chain does not reach the
    # origin.
    if pred_link[destination] < 0:
        return []
    reversed_links = [pred_link[destination]]
    pred = pred_node[destination]
    while pred != origin and pred != -1 and len(reversed_links) < 3000:
        reversed_links.append(pred_link[pred])
        pred = pred_node[pred]
    if pred != origin:
        return []
    return list(reversed(reversed_links))


def find_best_paths_forward(world, net, origin, departure_time, targets):
    # One label-correcting run from origin for every target node
    # at once, mirroring find_best_path's loop exactly (same
    # deque discipline, same floating_less, same f32 accumulation
    # and horizon cap) but with no destination upper bound: the
    # bound is per-destination in the single-target version and
    # cannot prune a run that serves several destinations.
    # Returns {target: (node_size, path_links, total_cost)} for
    # reachable targets only; total_cost is the f32 cost of the
    # extracted path minus the (clamped) departure, exactly as
    # find_best_path reports it. Only valid when
    # multi_run_valid(world, net) holds.
    if origin == -1:
        return {}
    departure_time = int(departure_time)
    if departure_time < world.start:
        departure_time = world.start
    if departure_time > world.end:
        departure_time = world.end
    nodes = world.nodes
    if len(nodes[origin].outgoing) == 0:
        return {}
    n = len(nodes)
    pred_node = [-1] * n
    pred_link = [-1] * n
    status = [0] * n
    label_time = [MAX_SPLABEL] * n
    label_cost = [MAX_SPLABEL] * n
    label_time[origin] = f32(departure_time)
    label_cost[origin] = f32(departure_time)
    se = deque([origin])
    while se:
        from_id = se.popleft()
        status[from_id] = 2
        for link_no in nodes[from_id].outgoing:
            link = world.links[link_no]
            to_id = link.to_id
            interval = world.interval_index(label_time[from_id])
            td = net.td_time[link_no][interval]
            new_time = f32(label_time[from_id] + td)
            new_cost = f32(label_cost[from_id] + td)
            if floating_less(new_cost, label_cost[to_id]):
                if new_time > world.horizon - 1:
                    new_time = f32(world.horizon - 1)
                label_time[to_id] = new_time
                label_cost[to_id] = new_cost
                pred_node[to_id] = from_id
                pred_link[to_id] = link_no
                if status[to_id] == 2:
                    se.appendleft(to_id)
                    status[to_id] = 1
                elif status[to_id] == 0:
                    se.append(to_id)
                    status[to_id] = 1
    out = {}
    for target in targets:
        if target == origin:
            continue
        total_cost = f32(label_cost[target] - f32(departure_time))
        if total_cost > f32(MAX_SPLABEL - f32(10)):
            continue
        path_links = _extract_path(pred_node, pred_link, origin, target)
        if not path_links:
            continue
        out[target] = (len(path_links) + 1, path_links, total_cost)
    return out


def find_best_paths_reverse(world, net, destination, targets):
    # One label-correcting run on the reversed network from
    # destination: for every target node it returns the shortest
    # path target -> destination as a forward-ordered link list.
    # The loop mirrors find_best_path (same deque discipline,
    # same floating_less, same f32 accumulation); traversing a
    # link backwards uses that link's own TD time, which is
    # valid only when the TD table is constant across intervals
    # (see multi_run_valid) -- otherwise the reverse
    # accumulation would depend on the arrival time at the
    # destination, which varies per target. The caller recomputes
    # each path's travel time in forward order (see
    # decision._path_time), since the reverse accumulation adds
    # the link times in the opposite order to a forward run.
    # Returns {target: path_links} for reachable targets only.
    nodes = world.nodes
    n = len(nodes)
    pred_node = [-1] * n
    pred_link = [-1] * n
    status = [0] * n
    label_time = [MAX_SPLABEL] * n
    label_cost = [MAX_SPLABEL] * n
    label_time[destination] = f32(0)
    label_cost[destination] = f32(0)
    se = deque([destination])
    while se:
        cur = se.popleft()
        status[cur] = 2
        for link_no in nodes[cur].incoming:
            link = world.links[link_no]
            from_id = link.from_id
            interval = world.interval_index(label_time[cur])
            td = net.td_time[link_no][interval]
            new_time = f32(label_time[cur] + td)
            new_cost = f32(label_cost[cur] + td)
            if floating_less(new_cost, label_cost[from_id]):
                if new_time > world.horizon - 1:
                    new_time = f32(world.horizon - 1)
                label_time[from_id] = new_time
                label_cost[from_id] = new_cost
                pred_node[from_id] = cur
                pred_link[from_id] = link_no
                if status[from_id] == 2:
                    se.appendleft(from_id)
                    status[from_id] = 1
                elif status[from_id] == 0:
                    se.append(from_id)
                    status[from_id] = 1
    out = {}
    for target in targets:
        if target == destination:
            continue
        if label_cost[target] > f32(MAX_SPLABEL - f32(10)):
            continue
        # Walk from the target toward the destination: the
        # reverse run's predecessor of a node is the next node
        # on its path to the destination, and the predecessor
        # link is the original link joining them.
        links = []
        node = target
        while node != destination and len(links) < 3000:
            link_no = pred_link[node]
            if link_no < 0:
                links = []
                break
            links.append(link_no)
            node = world.links[link_no].to_id
        if links and node == destination:
            out[target] = links
    return out


def find_best_path(world, net, origin_zone, origin, departure_time,
                   destination_zone, destination, demand_type, vot):
    # FindBestPathWithVOTForSingleAgent (ShortestPath.cpp:797), generalized
    # cost flag off. Returns (node_size, path_link_list, total_cost);
    # node_size 0 means no feasible path.
    if vot <= 0.01:
        vot = f32(1)
    if origin == destination:
        return 0, [], f32(0)
    if demand_type == 0:
        demand_type = 1
    departure_time = int(departure_time)
    if departure_time < world.start:
        departure_time = world.start
    if departure_time > world.end:
        departure_time = world.end
    nodes = world.nodes
    if len(nodes[origin].outgoing) == 0:
        return 0, [], f32(0)
    n = len(nodes)
    pred_node = [-1] * n
    pred_link = [-1] * n
    status = [0] * n
    label_time = [MAX_SPLABEL] * n
    label_cost = [MAX_SPLABEL] * n
    label_time[origin] = f32(departure_time)
    label_cost[origin] = f32(departure_time)
    se = deque([origin])
    cost_upper_bound = MAX_SPLABEL
    td_time = net.td_time
    links = world.links
    interval_index = world.interval_index
    fl = floating_less
    while se:
        from_id = se.popleft()
        status[from_id] = 2
        # The interval depends only on the popped node's label, which
        # cannot improve while its outgoing links are scanned: every
        # td is >= 0.01, so a relaxation back into from_id (only a
        # self-loop could do that) can never lower its label. The
        # label and cost of from_id are likewise fixed for the scan,
        # so compute the interval and alias them once per node.
        interval = interval_index(label_time[from_id])
        label_from = label_time[from_id]
        cost_from = label_cost[from_id]
        for link_no in nodes[from_id].outgoing:
            link = links[link_no]
            to_id = link.to_id
            if "c" in link.type_code:
                origin_taz = nodes[link.from_id].zone_id
                dest_taz = nodes[link.to_id].zone_id
                if origin_taz >= 1 and dest_taz <= 0 and \
                        origin_taz != origin_zone:
                    continue
                if dest_taz >= 1 and origin_taz <= 0 and \
                        dest_taz != destination_zone:
                    continue
                if origin_taz >= 1 and origin_taz != origin_zone and \
                        dest_taz >= 1 and dest_taz != destination_zone:
                    continue
                if to_id == origin:
                    continue
            td = td_time[link_no, interval]
            new_time = f32(label_from + td)
            new_cost = f32(cost_from + td)
            if fl(new_cost, label_cost[to_id]) and \
                    new_cost < cost_upper_bound:
                if new_time > world.horizon - 1:
                    new_time = f32(world.horizon - 1)
                label_time[to_id] = new_time
                label_cost[to_id] = new_cost
                pred_node[to_id] = from_id
                pred_link[to_id] = link_no
                if to_id == destination:
                    cost_upper_bound = label_cost[to_id]
                if status[to_id] == 2:
                    se.appendleft(to_id)
                    status[to_id] = 1
                elif status[to_id] == 0:
                    se.append(to_id)
                    status[to_id] = 1
    total_cost = f32(label_cost[destination] - f32(departure_time))
    if total_cost > f32(MAX_SPLABEL - f32(10)):
        return 0, [], total_cost
    # extract the link list by walking predecessors back from the destination
    reversed_links = [pred_link[destination]]
    pred = pred_node[destination]
    while pred != origin and pred != -1 and len(reversed_links) < 3000:
        reversed_links.append(pred_link[pred])
        pred = pred_node[pred]
    path_links = list(reversed(reversed_links))
    return len(path_links) + 1, path_links, total_cost
