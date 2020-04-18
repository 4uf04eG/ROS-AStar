import heapq
import path_planner


def find_path(map: path_planner.Map, start: float, goal: float) -> list:
    """ Function to find path from start to goal using A* algorithm """
    open_list = []
    closed_list = []
    heapq.heappush(open_list, (0.0, start))
    tolerance = (path_planner.pixel_tolerance - 1) * map.resolution

    while len(open_list) > 0:
        current_node = heapq.heappop(open_list)[1]

        for neighbor in current_node.generate_neighbors(map.resolution):
            if map.is_node_free(neighbor):
                if neighbor.calculate_distance(goal) < tolerance:  # Found path
                    neighbor.parent = current_node
                    return neighbor.backtrack_path()

                neighbor.g = current_node.g + neighbor.calculate_distance(current_node)
                neighbor.h = neighbor.calculate_distance(goal)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current_node

                if not any(neighbor == node and f <= neighbor.f for f, node in open_list):
                    if not any(neighbor == node and node.f <= neighbor.f for node in closed_list):
                        heapq.heappush(open_list, (neighbor.f, neighbor))

        closed_list.append(current_node)

    return []
