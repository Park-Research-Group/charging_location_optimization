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


def write_agent_file(world, agent_path):
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
