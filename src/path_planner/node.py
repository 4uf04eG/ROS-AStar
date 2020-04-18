import rospy
import path_planner
from tf import transformations
from geometry_msgs.msg import Pose, PoseStamped


class Node:
    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        parent=None
    ):
        self.parent = parent

        self.x = x
        self.y = y
        self.theta = theta

        self.g = 0  # From current node to start node
        self.h = 0  # From current node to end node
        self.f = 0  # Total cost
    
    def __eq__(self, other) -> bool:
        return self.calculate_distance(other) < 0.1

    def __add__(self, other):
        return Node(self.x + other.x, self.y + other.y)

    def __radd__(self, other):
        if other == 0:
            return self
        else:
            return self.__add__(other)

    def __truediv__(self, other):
        return Node(self.x / other, self.y / other)

    def calculate_distance(self, end) -> float:
        """
        Euclidean distance between two nodes

        d = sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2))
        """
        return ((self.x - end.x) ** 2 + (self.y - end.y) ** 2) ** 0.5

    def generate_neighbors(self, map_resolution: float) -> list:
        neighbors = []
        step = (path_planner.pixel_tolerance + 1) * map_resolution
        moves = [(0, -step), (0, step), (-step, 0), (step, 0),
                 (-step, -step), (-step, step), (step, -step), (step, step)]

        for move in moves:
            neighbors.append(Node(x=self.x + move[0], y=self.y + move[1]))
        
        return neighbors

    def backtrack_path(self) -> list:
        path = []
        current_node = self

        while current_node.parent:
            path.append(current_node)
            current_node = current_node.parent

        return (path + [current_node])[::-1]

    @staticmethod
    def from_pose(pose: Pose):
        new_state = Node()
        new_state.x = pose.position.x
        new_state.y = pose.position.y

        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)
        new_state.theta = yaw

        return new_state

    @staticmethod
    def from_tf(position: list, quaternion: list):
        new_state = Node()
        new_state.x = position[0]
        new_state.y = position[1]

        (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)
        new_state.theta = yaw

        return new_state
    
    def to_pose_stamped(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = 1.0

        return pose
