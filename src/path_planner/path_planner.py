#!/usr/bin/env python
import rospy
import path_planner
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf import TransformListener, ExtrapolationException, LookupException
from tf2_msgs.msg import TFMessage


class PathPlanner:
    def __init__(self):
        self.map = None
        self.start = None
        self.goal = None

        self.mover = path_planner.GotoMover(self)

        self.is_goal_cancelled = False
        self.is_goal_reached = False
        self.is_map_loaded = False

        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.start_subscriber = rospy.Subscriber("/tf", TFMessage, self.start_callback)
        self.goal_subscriber = rospy.Subscriber("/goal", PoseStamped, self.goal_callback)
        self.transform_listener = TransformListener()

        self.path_publisher = rospy.Publisher("/path", Path, queue_size=10)
        self.mover_publisher = rospy.Publisher("/move", Path, queue_size=1)
      
    def map_callback(self, data: OccupancyGrid):
        self.map = path_planner.Map(data)
        self.is_map_loaded = True
    
    def start_callback(self, data: TFMessage):
        try:
            position, quaternion = self.transform_listener.\
                lookupTransform("/map", "/base_link", rospy.Time())
        except (ExtrapolationException, LookupException):
            return

        self.start = path_planner.Node.from_tf(position, quaternion)
        self.mover.robot_position = self.start

    def goal_callback(self, data: PoseStamped) -> bool:
        if self.mover.is_moving:
            self.cancel_goal()

        rospy.loginfo("Received new goal")
        self.goal = path_planner.Node.from_pose(data.pose)
        self.is_goal_cancelled = False
        self.is_goal_reached = False

        if not self.map.is_node_free(self.goal) or not self.map or not self.start:
            rospy.loginfo("Goal can't be reached")
            self.display_path([])  # Clearing path
            return False

        return self.calculate_path()

    def calculate_path(self):
        rospy.loginfo("Calculating path...")
        path_list = path_planner.find_path(self.map, self.start, self.goal)

        if len(path_list) > 0:
            rospy.loginfo("Path calculated")
            path_list = self.moving_average(path_list)
            path_msg = self.display_path(path_list)
            self.mover_publisher.publish(path_msg)
            return True

        rospy.loginfo("No path found")
        self.display_path([])

        return False

    def wait_for_map(self):
        while not self.is_map_loaded and not rospy.is_shutdown(): 
            rospy.sleep(0.2)
    
    def wait_for_result(self, duration: float) -> bool:
        sending_time = rospy.Time.now().to_sec()
        rospy.sleep(1)

        while self.mover.is_moving and not self.is_goal_cancelled and not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - sending_time > duration:
                self.cancel_goal()
                return False
        
        return self.is_goal_reached
    
    def send_goal(self, pose: Pose) -> bool:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose = pose

        return self.goal_callback(goal)

    def cancel_goal(self):
        self.mover.initialize_stop()
        self.is_goal_cancelled = True

    def moving_average(self, path: list, window: int = 4) -> list:
        window_queue = []
        smoothed_path = [path[0]]

        for node in path:
            if len(window_queue) == window:
                smoothed_path.append(sum(window_queue) / window)  # Mean
                window_queue.pop(0)

            window_queue.append(node)

        return smoothed_path + [self.goal]

    def display_path(self, path_nodes: list) -> Path:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for node in path_nodes:
            path.poses.append(node.to_pose_stamped())

        self.path_publisher.publish(path)

        return path

