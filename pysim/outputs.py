# The agent output file, ported from OutputVehicleTrajectoryData
# (StatisticsOutput.cpp:2056). Writers use UTF-8, not latin-1.
# This build writes only output_agent.csv: the trip summary and both
# link MOE files are cut, and so are the toll and emission columns.
# Row formatting for the kept columns is unchanged from standalone v2,
# including the short row written for agents with fewer than 2 nodes.
from .cpp import f32

AGENT_HEADER = (
    "agent_id,tour_id,dependency_agent_id,duration_in_min,following_agent_id,"
    "from_zone_id, to_zone_id, from_origin_node_id, to_destination_node_id, "
    "departure_time_in_min, arrival_time_in_min, complete_flag, "
    "travel_time_in_min, demand_type, vehicle_type, PCE, information_type, "
    "value_of_time, distance, vehicle_age,number_of_nodes,"
    "path_node_sequence,org_path_node_sequence,path_time_sequence,"
    "origin_node_x,origin_node_y,destination_node_x,destination_node_y,\n")

# Columns appended after the v2 columns on every tour-mode leg row.
LEG_COLUMNS = (
    "leg_index,departure_mode,trip_purpose,day,"
    "scheduled_departure_in_min,actual_departure_in_min,"
    "scheduled_arrival_in_min,lateness_in_min,"
    "soc_start_kwh,soc_end_kwh,stranded_flag,tow_flag,charged_en_route")


def _leg_tail(leg):
    # The appended leg columns, shared by full and short rows.
    # departed is True once the leg was generated (departure_actual
    # set, including a real 0.0 for a leg generated at time 0) or
    # completed; departure_actual is None only for a leg that never
    # departed. A leg that never departed reports its scheduled
    # departure as the actual one, with lateness 0.00, and its
    # carried starting charge as both soc_start and soc_end (it
    # used no energy).
    departed = leg.complete or leg.departure_actual is not None
    actual_dep = leg.departure_actual if departed \
        else float(leg.scheduled_departure)
    day = int(actual_dep // 1440)
    sched_arr = float(leg.scheduled_departure) + float(leg.trip_fftt)
    lateness = actual_dep - float(leg.scheduled_departure) \
        if departed else 0.0
    soc_end = leg.soc_end_kwh if departed else leg.soc_start_kwh
    return (
        f"{leg.leg_index},{leg.departure_mode},{leg.trip_purpose},{day},"
        f"{float(leg.scheduled_departure):4.2f},{actual_dep:4.2f},"
        f"{sched_arr:4.2f},{lateness:4.2f},"
        f"{float(leg.soc_start_kwh):.3f},{float(soc_end):.3f},"
        f"{1 if leg.stranded else 0},{1 if leg.tow else 0},"
        f"{1 if leg.charged_en_route else 0}")


def _write_agent_file_tour(world, agent_path):
    # Tour mode: one row per leg, rendered from the snapshotted Leg
    # records only (vehicle working fields are overwritten by later
    # legs). The v2 column prefix is byte-identical to the legacy
    # layout, with the leg columns appended after its trailing comma.
    nodes = world.nodes
    links = world.links
    agent_lines = [AGENT_HEADER[:-1] + LEG_COLUMNS + "\n"]
    for agent_id in sorted(world.vehicles):
        veh = world.vehicles[agent_id]
        for leg in sorted(veh.legs, key=lambda l: l.leg_index):
            origin_number = nodes[leg.origin_node].number
            dest_number = nodes[leg.dest_node].number
            flag = "c" if leg.complete else "i"
            if leg.node_size >= 2:
                travel = f32(leg.arrival_actual - leg.departure_actual) \
                    if leg.complete else f32(0)
                # Node and time sequences, built in parallel: one
                # node per link (from-node, then each to-node) and
                # one stamp per node (buffer exit, then each link
                # arrival). A tow hop belongs right after the last
                # link travelled before the tow: the charger node
                # and the tow's arrival time at the charger are
                # inserted there, in tow order, so the jump is
                # visible and the sequences stay the same length.
                # The boundary is usually a discontinuity (the
                # next link starts at the charger node), but when
                # the charger node coincides with the stranding
                # link's to-node the splice is connected and the
                # hop doubles that node instead. A hop whose
                # boundary is past the last link (a tow to the
                # destination charger, with no links driven after
                # it) is appended at the end. Connected paths take
                # no insertions, so this is a no-op for them.
                # number_of_nodes is the length of the (possibly
                # extended) sequence.
                node_numbers = [nodes[links[leg.link_ary[0].link_no]
                                      .from_id].number]
                time_stamps = [leg.leaving_buffer_time]
                hops = getattr(leg, "tow_hops", [])
                hop_i = 0
                for i, entry in enumerate(leg.link_ary[:-1]):
                    node_numbers.append(
                        nodes[links[entry.link_no].to_id].number)
                    time_stamps.append(entry.abs_arrival)
                    while hop_i < len(hops) and hops[hop_i][0] == i + 1:
                        node_numbers.append(hops[hop_i][1])
                        time_stamps.append(hops[hop_i][2])
                        hop_i += 1
                for hop in hops[hop_i:]:
                    node_numbers.append(hop[1])
                    time_stamps.append(hop[2])
                row = (
                    f"{veh.agent_id},{veh.tour_id},"
                    f"{veh.dependency_agent_id},"
                    f"{float(veh.duration):.0f},{veh.following_agent_id},"
                    f"{leg.origin_zone},{leg.dest_zone},{origin_number},"
                    f"{dest_number},{float(leg.scheduled_departure):4.2f},"
                    f"{float(leg.arrival_actual):4.2f},{flag},"
                    f"{float(travel):4.2f},"
                    f"{veh.demand_type},{veh.vehicle_type},"
                    f"{float(veh.pce):4.2f},"
                    f"{veh.info_type},{float(veh.vot):4.2f},"
                    f"{float(leg.distance):4.3f},"
                    f"{veh.age},{len(node_numbers)},")
                row += "".join("%d;" % n for n in node_numbers) + ","
                if leg.original_link_ary is not None:
                    row += "%d;" % node_numbers[-1]
                    for entry in leg.original_link_ary[:-1]:
                        row += "%d;" % nodes[
                            links[entry.link_no].to_id].number
                row += ","
                row += "".join("%4.1f;" % float(t)
                               for t in time_stamps) + ","
                origin_node = nodes[links[leg.link_ary[0].link_no].from_id]
                # The last driven link ends at the destination on
                # every connected path; a tow to the destination
                # charger ends at the stranded node instead, so
                # the destination coordinates come from the leg's
                # destination node in that case.
                if links[leg.link_ary[leg.node_size - 2].link_no].to_id \
                        == leg.dest_node:
                    dest_node = nodes[
                        links[leg.link_ary[leg.node_size - 2].link_no]
                        .to_id]
                else:
                    dest_node = nodes[leg.dest_node]
                row += f"{origin_node.x:f},{origin_node.y:f},"
                row += f"{dest_node.x:f},{dest_node.y:f},"
                row += _leg_tail(leg) + "\n"
                agent_lines.append(row)
            else:
                # Short row: the v2 short layout per leg, plus the
                # same appended leg columns. Instant legs take their
                # endpoints from the leg's own nodes.
                agent_lines.append(
                    f"{veh.agent_id},{veh.tour_id},"
                    f"{veh.dependency_agent_id},"
                    f"{float(veh.duration):.0f},{veh.following_agent_id},"
                    f"{leg.origin_zone},{leg.dest_zone},{origin_number},"
                    f"{dest_number},{float(leg.scheduled_departure):4.2f},"
                    f"{float(leg.arrival_actual):4.2f},{flag},{0.0:4.2f},"
                    f"{veh.demand_type},{veh.vehicle_type},{veh.info_type},"
                    f"{float(veh.vot):4.2f},"
                    f"{float(leg.distance):4.3f},"
                    f"{leg.node_size},{_leg_tail(leg)}\n")
    with open(agent_path, "w", newline="", encoding="utf-8") as fh:
        fh.writelines(agent_lines)


def write_agent_file(world, agent_path):
    if world.tour_mode:
        return _write_agent_file_tour(world, agent_path)
    nodes = world.nodes
    links = world.links
    agent_lines = [AGENT_HEADER]
    for agent_id in sorted(world.vehicles):
        veh = world.vehicles[agent_id]
        origin_number = nodes[veh.origin_node].number
        dest_number = nodes[veh.dest_node].number
        flag = "c" if veh.complete else "i"
        if veh.node_size >= 2:
            trip_time = f32(veh.arrival - veh.departure) if veh.complete \
                else f32(0)
            row = (
                f"{veh.agent_id},{veh.tour_id},{veh.dependency_agent_id},"
                f"{float(veh.duration):.0f},{veh.following_agent_id},"
                f"{veh.origin_zone},{veh.dest_zone},{origin_number},"
                f"{dest_number},{float(veh.departure):4.2f},"
                f"{float(veh.arrival):4.2f},{flag},{float(trip_time):4.2f},"
                f"{veh.demand_type},{veh.vehicle_type},{float(veh.pce):4.2f},"
                f"{veh.info_type},{float(veh.vot):4.2f},"
                f"{float(veh.distance):4.3f},"
                f"{veh.age},{veh.node_size},")
            # path node sequence, derived from the current link list
            node_numbers = [nodes[links[veh.link_ary[0].link_no].from_id]
                            .number]
            for entry in veh.link_ary[:-1]:
                node_numbers.append(
                    nodes[links[entry.link_no].to_id].number)
            row += "".join("%d;" % n for n in node_numbers) + ","
            # original path (after a real-time switch or a reroute
            # callback splice); the C++ prints the current path's last
            # node first, then the stored path's nodes
            if veh.original_link_ary is not None:
                row += "%d;" % node_numbers[-1]
                for entry in veh.original_link_ary[:-1]:
                    row += "%d;" % nodes[links[entry.link_no].to_id].number
            row += ","
            row += f"{float(veh.leaving_buffer_time):4.1f};"
            for entry in veh.link_ary[:-1]:
                row += f"{float(entry.abs_arrival):4.1f};"
            row += ","
            origin_node = nodes[links[veh.link_ary[0].link_no].from_id]
            dest_node = nodes[
                links[veh.link_ary[veh.node_size - 2].link_no].to_id]
            row += f"{origin_node.x:f},{origin_node.y:f},"
            row += f"{dest_node.x:f},{dest_node.y:f},"
            row += "\n"
            agent_lines.append(row)
        else:
            agent_lines.append(
                f"{veh.agent_id},{veh.tour_id},{veh.dependency_agent_id},"
                f"{float(veh.duration):.0f},{veh.following_agent_id},"
                f"{veh.origin_zone},{veh.dest_zone},{origin_number},"
                f"{dest_number},{float(veh.departure):4.2f},"
                f"{float(veh.arrival):4.2f},{flag},{0.0:4.2f},"
                f"{veh.demand_type},{veh.vehicle_type},{veh.info_type},"
                f"{float(veh.vot):4.2f},"
                f"{float(veh.distance):4.3f},"
                f"{veh.node_size}\n")
    with open(agent_path, "w", newline="", encoding="utf-8") as fh:
        fh.writelines(agent_lines)
