import path_planner
from nav_msgs.msg import OccupancyGrid


class Map:
    def __init__(self, grid_map: OccupancyGrid):
        self.data       = grid_map.data
        self.width      = grid_map.info.width
        self.height     = grid_map.info.height
        self.resolution = grid_map.info.resolution
        self.origin     = grid_map.info.origin.position

    def get_by_indices(self, i: int, j: int) -> int:
        return self.data[i * self.width + j]

    def get_by_coordinates(self, x: float, y: float) -> int:
        indices = self.coordinates_to_indices(x, y)
        return self.get_by_indices(indices[0], indices[1])

    def coordinates_to_indices(self, x: float, y: float) -> tuple:
        i = int((y - self.origin.y) / self.resolution)
        j = int((x - self.origin.x) / self.resolution)
        return i, j

    def is_node_free(self, node: path_planner.Node) -> bool:
        i, j = self.coordinates_to_indices(node.x, node.y)
        tolerance = path_planner.pixel_tolerance

        if not 0 <= i < self.height or not 0 <= j < self.width:
            return False

        for offset_x in range(-tolerance, tolerance):
            for offset_y in range(-tolerance, tolerance):
                if not -1 < self.get_by_indices(i - offset_y, j - offset_x) < 100:
                    return False

        return True

    def print_map(self, index):
        """ Debug function to print position on map """

        for i in range(0, self.height):
            for j in range(0, self.width):
                if index[0] == i and index[1] == j:
                    print("Z", end='')
                elif self.get_by_indices(i, j) == -1:
                    print(0, end='')
                elif self.get_by_indices(i, j) == 100:
                    print('a', end='')
                else:
                    print('1', end='')

            print()
