# The network loading: g_NetworkLoading + g_VehicularSimulation
# (simulation.cpp) and the two real-time path adjustments
# (RealTimeRouting.cpp). One step = 0.1 min (6 s), five stages per step:
# generation, loading buffer, entrance->exit transfer, capacities with
# merge distribution, node transfer; then per-step statistics.
import numpy as np

from .cpp import ProgramStop, f32, floating_less, less_eq
from .network import VehicleLink
from .sp import SPNetwork, find_best_path

SIM_INTERVAL = 0.1  # g_DTASimulationInterval (double)


class Simulator:
    def __init__(self, world):
        self.world = world
        self.generated = 0
        self.completed = 0
        # g_PrevailingTimeNetwork_MP[0], built in g_ReadInputFiles with
        # current (prevailing) information at the loading start time
        self.prevailing_net = SPNetwork(world, use_prevailing=True,
                                        current_time=world.start)
        self.td_map = {}

    # ----- iteration 0 of the agent-based driver -----

    def assign_paths_iteration_zero(self):
        # g_AgentBasedDynamicTrafficAssignmentSimulation, iteration 0:
        # pathless agents (by zone bucket, then read order) get a free-flow
        # shortest path on the physical network; preassigned paths are kept.
        world = self.world
        net = SPNetwork(world, use_prevailing=False)
        buckets = {}
        for agent_id in world.vehicle_order:
            veh = world.vehicles[agent_id]
            zone_seq = world.zones[veh.origin_zone].seq_no
            interval = world.interval_index(veh.departure)
            buckets.setdefault((zone_seq, interval), []).append(veh)
        for key in sorted(buckets):
            for veh in buckets[key]:
                if veh.info_type == 0 and veh.node_size >= 2:
                    continue
                if veh.origin_zone == veh.dest_zone:
                    continue  # intra-zone traffic is not simulated
                world.gen_rng.ratio()  # pVeh->GetRandomRatio() draw, unused
                if veh.node_size != 0:
                    continue
                node_size, path_links, _ = find_best_path(
                    world, net, veh.origin_zone, veh.origin_node,
                    veh.departure, veh.dest_zone, veh.dest_node,
                    veh.demand_type, veh.vot)
                if node_size >= 2:
                    veh.node_size = node_size
                    veh.link_ary = [VehicleLink(link_no)
                                    for link_no in path_links]
                    veh.link_ary.append(VehicleLink(65530))
                    distance = f32(0)
                    for link_no in path_links:
                        distance = f32(distance +
                                       world.links[link_no].length)
                    veh.distance = distance

    # ----- real-time path adjustments -----

    def _store_original_path(self, veh):
        veh.original_link_ary = [
            VehicleLink(entry.link_no) for entry in veh.link_ary]
        for new, old in zip(veh.original_link_ary, veh.link_ary):
            new.abs_arrival = old.abs_arrival
        veh.original_node_size = veh.node_size

    def generation_adjustment(self, veh, current_time):
        # g_AgentBasedPathAdjustmentWithRealTimeInfo (RealTimeRouting.cpp:399)
        world = self.world
        if veh.node_size >= 1 and veh.link_ary:
            current_link = veh.link_ary[veh.sim_link_seq].link_no
            starting_node = world.links[current_link].to_id
            if starting_node == veh.dest_node:
                return
            current_path = [entry.link_no for entry in veh.link_ary[:-1]]
            stamps = [entry.abs_arrival for entry in veh.link_ary[:-1]]
            link_count = veh.sim_link_seq
            updated = list(current_path[:veh.sim_link_seq + 1])
        else:
            starting_node = veh.origin_node
            current_path = []
            stamps = []
            link_count = 0
            updated = []
        node_size, path_links, _ = find_best_path(
            world, self.prevailing_net, veh.origin_zone, starting_node,
            veh.departure, veh.dest_zone, veh.dest_node, veh.demand_type,
            veh.vot)
        switch = 0
        for i in range(node_size - 1):
            link_count += 1
            old = current_path[link_count] if link_count < len(current_path) \
                else -1
            if old != path_links[i]:
                switch = 1
            updated.append(path_links[i])
        if node_size <= 1 or switch == 0:
            return
        if veh.node_size >= 2:
            self._store_original_path(veh)
            veh.node_size = len(updated) + 1
            veh.link_ary = []
            distance = f32(0)
            for i, link_no in enumerate(updated):
                entry = VehicleLink(link_no)
                entry.abs_arrival = stamps[i] if i < len(stamps) else f32(0)
                veh.link_ary.append(entry)
                distance = f32(distance + world.links[link_no].length)
            veh.link_ary.append(VehicleLink(65530))
            veh.distance = distance

    def enroute_adjustment(self, veh, current_time):
        # g_OpenMPAgentBasedPathAdjustmentWithRealTimeInfo
        # (RealTimeRouting.cpp:649), VMS detour branch inert (no signs)
        world = self.world
        current_link = veh.link_ary[veh.sim_link_seq].link_no
        starting_node = world.links[current_link].to_id
        current_path = [entry.link_no for entry in veh.link_ary[:-1]]
        stamps = [entry.abs_arrival for entry in veh.link_ary[:-1]]
        node_size, sub_links, _ = find_best_path(
            world, self.prevailing_net, veh.origin_zone, starting_node,
            veh.departure, veh.dest_zone, veh.dest_node, veh.demand_type,
            veh.vot)
        if veh.sim_link_seq == veh.node_size - 2:
            return
        now_count = veh.sim_link_seq + 1
        first_diff_old = -1
        first_diff_new = -1
        switch = 0
        for i in range(node_size - 1):
            old = current_path[now_count] if now_count < len(current_path) \
                else -1
            if old != sub_links[i]:
                first_diff_old = now_count
                first_diff_new = i
                switch = 1
                break
            now_count += 1
        if node_size - 1 <= 0:
            return
        diff_size = (node_size - 1) - first_diff_new
        new_node_size = first_diff_old + diff_size + 1
        if new_node_size >= 3000:
            return
        if switch == 0:
            return
        self._store_original_path(veh)
        combined = current_path[:first_diff_old] + \
            sub_links[first_diff_new:first_diff_new + diff_size]
        veh.node_size = new_node_size
        veh.link_ary = []
        distance = f32(0)
        for i, link_no in enumerate(combined):
            entry = VehicleLink(link_no)
            stamp = stamps[i] if i < len(stamps) else f32(0)
            if stamp == f32(0):
                stamp = f32(99999.0)
            entry.abs_arrival = stamp
            veh.link_ary.append(entry)
            distance = f32(distance + world.links[link_no].length)
        veh.link_ary.append(VehicleLink(65530))
        veh.distance = distance

    def _splice_reroute(self, veh, next_node, new_links):
        # Replace the rest of a vehicle's path with a caller-supplied
        # link list, keeping everything up to and including the link
        # the vehicle just completed (sim_link_seq still points at
        # it: the splice runs before the move to the next link).
        # Contract for new_links: a connected chain of link numbers
        # whose first link starts at next_node (the to-node of the
        # just-completed link) and whose last link ends at the
        # vehicle's destination node.
        # Anything else raises ProgramStop rather than silently
        # corrupting the path. The pre-splice path is stored as the
        # original path, exactly as the built-in adjustments do, so
        # output_agent.csv keeps both the travelled and original
        # node sequences.
        world = self.world
        links = world.links
        if not new_links:
            return
        if links[new_links[0]].from_id != next_node:
            raise ProgramStop(
                "reroute path for agent %d does not start at its "
                "current node" % veh.agent_id)
        if links[new_links[-1]].to_id != veh.dest_node:
            raise ProgramStop(
                "reroute path for agent %d does not end at its "
                "destination" % veh.agent_id)
        for a, b in zip(new_links, new_links[1:]):
            if links[a].to_id != links[b].from_id:
                raise ProgramStop(
                    "reroute path for agent %d is not a connected "
                    "chain" % veh.agent_id)
        self._store_original_path(veh)
        kept = [entry.link_no for entry in veh.link_ary[:veh.sim_link_seq + 1]]
        stamps = [entry.abs_arrival
                  for entry in veh.link_ary[:veh.sim_link_seq + 1]]
        combined = kept + list(new_links)
        veh.node_size = len(combined) + 1
        veh.link_ary = []
        distance = f32(0)
        for i, link_no in enumerate(combined):
            entry = VehicleLink(link_no)
            entry.abs_arrival = stamps[i] if i < len(stamps) else f32(0)
            veh.link_ary.append(entry)
            distance = f32(distance + world.links[link_no].length)
        veh.link_ary.append(VehicleLink(65530))
        veh.distance = distance

    # ----- network loading -----

    def network_loading(self):
        world = self.world
        for link in world.links:
            link.setup_loading(world.horizon)
        world.info_updating_flag = False
        for agent_id in world.vehicle_order:
            veh = world.vehicles[agent_id]
            if veh.demand_type != 4:
                veh.arrival = f32(0)
                veh.trip_time = f32(0)
                veh.trip_fftt = f32(0)
            veh.loaded = False
            veh.complete = False
            veh.delay = f32(0)
            if veh.info_type >= 2:
                world.info_updating_flag = True
        self.generated = 0
        self.completed = 0
        # rebuild the time-dependent departure lists from the original map,
        # walking the same floating-point time loop as the C++ code
        self.td_map = {}
        step = 0
        time = float(world.start)
        limit = min(world.horizon, world.horizon)
        while time < limit:
            time = world.start + step * SIM_INTERVAL
            time_int = int(time * 10)
            if time_int in world.original_td_map:
                self.td_map[time_int] = list(world.original_td_map[time_int])
            step += 1
        # Early-exit bookkeeping. n_generatable counts, once, the vehicles
        # that can ever be generated and completed: stage 0 loads only
        # vehicles with node_size >= 2, and node_size is final at this
        # point (iteration-0 assignment already ran; the real-time
        # adjustments rewrite only loaded or in-transit vehicles and keep
        # node_size >= 2, so nothing can join the generatable set later).
        # The count is therefore exact for the generated set and a superset
        # of the completed set: a counted vehicle that never completes
        # (permanent gridlock) simply keeps the break from firing.
        self.n_generatable = 0
        for agent_id in world.vehicle_order:
            if world.vehicles[agent_id].node_size >= 2:
                self.n_generatable += 1
        self._early_exit_vetoed = False
        # Main loop: stops early once the network is drained (see the
        # break below). horizon = loading end + 300.
        step = 0
        time = float(world.start)
        while time < limit:
            time = world.start + step * SIM_INTERVAL
            self.step(time, step)
            step_no = step
            step += 1
            # Early exit. Every clause is an O(1) integer/float compare.
            # time > world.end: every registered departure lies in
            # [start, end] (read time rejects anything outside), and each
            # departure bucket is visited exactly once as the clock
            # advances, so past end no vehicle can still be generated.
            # step_no % 10 == 0: the break step is a minute boundary, so
            # stage 5 has just written this minute's MOE from final flows
            # and the post-pass propagates those finals over the tail.
            # generated == completed == n_generatable: every generatable
            # vehicle entered the network and reached its destination.
            if not self._early_exit_vetoed and time > world.end and \
                    step_no % 10 == 0 and \
                    self.generated == self.n_generatable and \
                    self.completed == self.n_generatable:
                # Belt-and-braces veto scan: ground truth for "nothing in
                # transit" is the queue system itself (a generated vehicle
                # sits in exactly one of loading_buffer, entrance_queue,
                # exit_queue until its completion pop). It runs at most
                # once: a veto disables all later checks and the run
                # continues to the horizon, outputs identical to unpatched.
                if self._queues_empty():
                    break
                self._early_exit_vetoed = True
        self._post_loading()

    def _queues_empty(self):
        for link in self.world.links:
            if link.loading_buffer or link.entrance_queue or \
                    link.exit_queue:
                return False
        return True

    def _post_loading(self):
        world = self.world
        # post-simulation tally (completed vehicles only): trip FFTT
        for agent_id in world.vehicle_order:
            veh = world.vehicles[agent_id]
            if not veh.complete:
                continue
            veh.trip_fftt = f32(0)
            for j in range(veh.node_size - 1):
                link = world.links[veh.link_ary[j].link_no]
                veh.trip_fftt = f32(veh.trip_fftt + link.fftt)

    # ----- one simulation step -----

    def step(self, current_time, step_no):
        world = self.world
        links = world.links
        time_stamp = int(current_time + 0.0001)

        # information updating block (before generation)
        if world.info_updating_flag and \
                step_no % (10 * world.info_updating_interval) == 0:
            for link in links:
                total_delay = 0.0
                for item in link.exit_queue:
                    delay = current_time - float(item[2])
                    if delay > 0:
                        total_delay += delay
                prevailing = f32(float(link.fftt) +
                                 total_delay / max(1, len(link.exit_queue)))
                based = f32(link.departure_based_tt /
                            f32(max(1, link.departure_count)))
                if based < link.fftt:
                    based = link.fftt
                link.prevailing_tt = f32(
                    f32(f32(1 - 0.5) * prevailing) + f32(f32(0.5) * based))
                link.departure_count = 0
                link.departure_based_tt = f32(0)
            self.prevailing_net.update_current_travel_time(current_time)

        # stage 0: generation from the departure bucket of this step
        simulation_time_no = time_stamp * 10 + step_no % 10
        computation = []
        for agent_id in self.td_map.get(simulation_time_no, []):
            veh = world.vehicles.get(agent_id)
            if veh is None or veh.loaded:
                continue
            if veh.info_type in (2, 3):
                computation.append(veh)
            if veh.node_size >= 2:
                first = links[veh.link_ary[0].link_no]
                stamp = f32(veh.departure + first.fftt)
                next_node_number = -1
                if veh.node_size >= 3:
                    next_node_number = world.nodes[
                        links[veh.link_ary[1].link_no].to_id].number
                first.loading_buffer.append(
                    [agent_id, next_node_number, stamp])
                veh.loaded = True
                veh.sim_link_seq = 0
                self.generated += 1
        for veh in computation:
            self.generation_adjustment(veh, current_time)

        # stage 1: loading buffer -> entrance queue (storage gate)
        for link in links:
            while link.loading_buffer and link.outflow_lanes > 0.01:
                item = link.loading_buffer[0]
                stamp = item[2]
                if less_eq(stamp, current_time):
                    stamp = f32(current_time)
                on_link = link.cflow_arrival - link.cflow_departure
                available = f32(link.space_capacity - f32(on_link))
                threshold = f32(link.space_capacity *
                                f32(f32(1) - f32(0.8)))
                if threshold < 2:
                    threshold = f32(2)
                if available >= threshold:
                    link.loading_buffer.popleft()
                    item[2] = stamp
                    link.entrance_queue.append(item)
                    link.cflow_arrival += 1
                    veh = world.vehicles[item[0]]
                    link.counts_demand[veh.demand_type] += 1
                    link.counts_vtype[veh.vehicle_type] += 1
                    veh.leaving_buffer_time = f32(current_time)
                    link.buffer_waiting_time = f32(
                        float(link.buffer_waiting_time) +
                        (current_time - float(veh.departure)))
                else:
                    break

        # stage 2: entrance queue -> exit queue at the free-flow stamp
        for link in links:
            while link.entrance_queue:
                item = link.entrance_queue[0]
                if less_eq(item[2], current_time):
                    link.entrance_queue.popleft()
                    link.exit_queue.append(item)
                else:
                    break

        # stage 3: link in/out capacities (Newell only)
        relax = current_time >= world.end + 60
        for link in links:
            per_hour = link.lane_capacity
            per_interval = float(per_hour) * SIM_INTERVAL / 60.0 * \
                float(link.outflow_lanes)
            capacity = f32(per_interval)
            if relax:
                capacity = f32(capacity * f32(3))
            link.cum_out_counter = f32(link.cum_out_counter + capacity)
            link.link_out_capacity = int(link.cum_out_counter) - \
                link.cum_out_prev
            link.cum_out_prev += link.link_out_capacity
            on_link = link.cflow_arrival - link.cflow_departure
            f_in = f32(99999.0)
            # Space-capacity inflow limiting applies to freeway
            # links only. The old signal-model gate is gone: with no
            # signal model, this build keeps the default behaviour
            # (old signal_representation_model = 0). Networks that
            # previously set signal_model = 2 will see different
            # arterial inflow behaviour.
            if "f" in link.type_code:
                available = f32(link.space_capacity - f32(on_link))
                inflow_cap = 1800 * SIM_INTERVAL / 60.0 * \
                    float(link.inflow_lanes)
                f_in = f32(min(float(available), inflow_cap))
                if f_in < 0:
                    f_in = f32(0)
                if step_no >= link.backward_wave_steps:
                    slot = max(0, step_no - link.backward_wave_steps) % 300
                    jam = f32(f32(link.length *
                                  f32(link.outflow_lanes)) * link.k_jam)
                    n_allow = int(f32(link.ring_departure[slot] + jam))
                    prev_slot = max(0, step_no - 1) % 300
                    flow_allowed = n_allow - int(link.ring_arrival[
                        prev_slot])
                    if flow_allowed < 0:
                        flow_allowed = 0
                    if f_in > flow_allowed:
                        f_in = f32(flow_allowed)
                inflow_rate = f32(per_interval) * world.min_inflow_ratio
                if relax:
                    inflow_rate = f32(per_interval)
                if f_in < inflow_rate:
                    f_in = inflow_rate
            link.cum_in_counter = f32(link.cum_in_counter + f_in)
            link.link_in_capacity = int(link.cum_in_counter) - \
                link.cum_in_prev
            link.cum_in_prev += link.link_in_capacity
            if "r" in link.type_code:
                if link.on_ramp and world.use_pq_on_ramps == 1:
                    link.link_in_capacity = 99999
                if not link.on_ramp and world.use_pq_off_ramps == 1:
                    link.link_in_capacity = 99999
        # merge distribution of the downstream in-capacity
        for link in links:
            if link.merge_flag < 1:
                continue
            total_demand = sum(
                len(links[entry[0]].exit_queue)
                for entry in link.merge_incoming)
            if total_demand <= link.link_in_capacity:
                continue
            if len(link.merge_incoming) == 2:
                for il, entry in enumerate(link.merge_incoming):
                    other = link.merge_incoming[1 - il][0]
                    entry[2] = f32(link.link_in_capacity -
                                   len(links[other].exit_queue))
            for entry in link.merge_incoming:
                incoming = links[entry[0]]
                cap1 = f32(f32(link.link_in_capacity) * entry[1])
                cap2 = entry[2]
                cap3 = f32(incoming.link_out_capacity)
                merged = min(cap3, max(cap1, cap2))
                prev = incoming.cum_merge_counter
                incoming.cum_merge_counter = f32(
                    incoming.cum_merge_counter + merged)
                incoming.link_out_capacity = \
                    int(incoming.cum_merge_counter) - int(prev)

        # stage 4: node transfer
        self._node_transfer(current_time, step_no)

        # stage 5: ring buffers for Newell's backward wave
        slot = step_no % 300
        for link in links:
            link.ring_arrival[slot] = f32(link.cflow_arrival)
            link.ring_departure[slot] = f32(link.cflow_departure)

    def _node_transfer(self, current_time, step_no):
        world = self.world
        links = world.links
        for node in world.nodes:
            incoming = node.incoming
            if not incoming:
                continue
            size = len(incoming)
            sequence = step_no % size
            count = 0
            while count < size:
                link = links[incoming[sequence]]
                self._discharge_link(link, node, current_time)
                count += 1
                sequence = (sequence + 1) % size

    def _discharge_link(self, link, node, current_time):
        world = self.world
        links = world.links
        vehicle_out_count = link.link_out_capacity
        if len(link.exit_queue) <= link.link_out_capacity:
            vehicle_out_count = len(link.exit_queue)
        # PCE pre-walk over the whole exit queue
        demand_counter = f32(0)
        agent_counter = 0
        for item in link.exit_queue:
            veh = world.vehicles[item[0]]
            demand_counter = f32(demand_counter + veh.pce)
            if demand_counter > vehicle_out_count:
                difference = f32(demand_counter - f32(vehicle_out_count))
                chance = float(f32(veh.pce - difference)) / \
                    max(0.0001, float(veh.pce))
                if link.rng.ratio() < chance:
                    agent_counter += 1
                break
            agent_counter += 1
        vehicle_out_count = agent_counter
        through_blocking = 0
        while link.exit_queue and vehicle_out_count > 0:
            if through_blocking > 0:
                break
            item = link.exit_queue[0]
            veh = world.vehicles[item[0]]
            seq = veh.sim_link_seq
            if seq >= 1:
                t_link_arrival = int(veh.link_ary[seq - 1].abs_arrival)
            else:
                t_link_arrival = int(veh.departure)
            number_of_links = veh.node_size - 1
            if less_eq(current_time - SIM_INTERVAL, item[2]):
                arrival = item[2]
            else:
                arrival = f32(current_time)
            veh.link_ary[seq].abs_arrival = arrival
            if seq >= 1:
                travel_time = f32(arrival - veh.link_ary[seq - 1].abs_arrival)
            else:
                travel_time = f32(arrival - veh.departure)
            if seq < number_of_links - 1:
                # Mid-run reroute hook (charging fork): the vehicle has
                # just completed this link and is at its to-node. A
                # user callback may replace the rest of its path from
                # that node onward, e.g. to detour via a charger. It
                # runs before the vehicle commits to its next link, so
                # a returned chain starts at the arrival node. Without
                # a callback, or when it returns None, the preassigned
                # path stands.
                rerouted = False
                arrival_node = link.to_id
                if arrival_node != veh.dest_node and \
                        world.reroute_callback is not None:
                    new_links = world.reroute_callback(
                        veh, arrival_node, world)
                    if new_links:
                        self._splice_reroute(veh, arrival_node, new_links)
                        number_of_links = veh.node_size - 1
                        rerouted = True
                next_link = links[veh.link_ary[seq + 1].link_no]
                if next_link.link_in_capacity > 0:
                    minimum = link.fftt
                    if travel_time < minimum:
                        arrival = f32(arrival + f32(minimum - travel_time))
                        travel_time = minimum
                    veh.delay = f32(veh.delay +
                                    f32(travel_time - link.fftt))
                    veh.sim_link_seq = seq + 1
                    new_item = [item[0], -1,
                                f32(arrival + next_link.fftt)]
                    if veh.sim_link_seq < number_of_links - 2:
                        next_next = links[
                            veh.link_ary[veh.sim_link_seq + 2].link_no]
                        new_item[1] = world.nodes[next_next.to_id].number
                    next_link.entrance_queue.append(new_item)
                    link.exit_queue.popleft()
                    next_link.cflow_arrival += 1
                    next_link.counts_demand[veh.demand_type] += 1
                    next_link.counts_vtype[veh.vehicle_type] += 1
                    link.cflow_departure += 1
                    link.departure_count += 1
                    link.departure_based_tt = f32(
                        link.departure_based_tt + travel_time)
                    next_link.link_in_capacity -= 1
                    # Built-in en-route adjustment (info 3), gated on
                    # not reaching the destination node with this move
                    # and not already rerouted by the callback above.
                    current_link_no = veh.link_ary[veh.sim_link_seq].link_no
                    next_node = links[current_link_no].to_id
                    if next_node != veh.dest_node and not rerouted and \
                            veh.info_type == 3:
                        self.enroute_adjustment(veh, current_time)
                else:
                    through_blocking += 1
            else:
                # destination reached on this link
                arrival = f32(current_time)
                veh.link_ary[seq].abs_arrival = arrival
                if seq >= 1:
                    travel_time = f32(
                        arrival - veh.link_ary[seq - 1].abs_arrival)
                else:
                    travel_time = f32(arrival - veh.departure)
                veh.delay = f32(veh.delay + f32(travel_time - link.fftt))
                veh.sim_link_seq = seq + 1
                veh.arrival = arrival
                veh.trip_time = f32(arrival - veh.departure)
                veh.travel_time = f32(arrival - veh.leaving_buffer_time)
                veh.complete = True
                veh.trip_fftt = f32(0)
                for j in range(veh.node_size - 1):
                    veh.trip_fftt = f32(
                        veh.trip_fftt + links[veh.link_ary[j].link_no].fftt)
                link.cflow_departure += 1
                link.departure_count += 1
                link.departure_based_tt = f32(
                    link.departure_based_tt + travel_time)
                link.exit_queue.popleft()
                self.completed += 1
            vehicle_out_count -= 1
