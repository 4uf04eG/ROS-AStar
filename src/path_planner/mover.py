import rospy
import path_planner
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from math import atan2, pi
from numpy import nanmin


move_tolerance = 0.1
scan_tolerance_front = 0.4
scan_tolerance_side = 0.3
rotate_tolerance = 0.004
linear_velocity = 0.25
angular_velocity = 1.5


class GotoMover:
    def __init__(self, planner):
        self.robot_position = None  # Set from planner
        self.path_deviation = 0.0   # Used to avoid obstacles

        self.planner = planner

        self.is_shutdown_initiated = False
        self.is_moving = False
        self.is_obstacle_ahead = False

        self.mover_subscriber = rospy.Subscriber("/move", Path, self.move_callback)
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def move_callback(self, path: Path):
        node_path = []

        for pose in path.poses:
            node_path.append(path_planner.Node.from_pose(pose.pose))

        self.follow_path(node_path)

    def scan_callback(self, scan_data: LaserScan):
        if nanmin(scan_data.ranges[0:10] + scan_data.ranges[350:360]) < scan_tolerance_front:
            self.is_obstacle_ahead = True
        elif nanmin(scan_data.ranges[11:165]) < scan_tolerance_side:
            self.path_deviation = -0.6
        elif nanmin(scan_data.ranges[195:349]) < scan_tolerance_side:
            self.path_deviation = 0.6
        else:
            self.path_deviation = 0

    def initialize_stop(self):
        self.is_shutdown_initiated = True

    def stop_moving(self):
        self.is_shutdown_initiated = False
        self.is_moving = False
        self.velocity_publisher.publish(Twist())

    def follow_path(self, path: list):
        self.is_moving = True

        for node in path:
            if self.is_shutdown_initiated:
                self.stop_moving()
                return
            if self.is_obstacle_ahead:
                self.velocity_publisher.publish(Twist())
                self.go_back()
                self.velocity_publisher.publish(Twist())
                self.planner.calculate_path()  # Recalculating path
                return

            self.move_to_point(node)

        self.velocity_publisher.publish(Twist())  # Stopping robot
        self.rotate_to_goal(self.planner.goal)
        self.velocity_publisher.publish(Twist())

        self.planner.is_goal_reached = True
        self.is_moving = False

    def go_back(self):
        current_distance = 0.0
        vel_msg = Twist()
        vel_msg.linear.x = -linear_velocity
        t0 = rospy.Time().now().to_sec()
        loop_rate = rospy.Rate(1000)

        while current_distance < 0.4:
            if self.is_shutdown_initiated:
                self.is_obstacle_ahead = False
                return

            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time().now().to_sec()
            current_distance = linear_velocity * (t1 - t0)
            loop_rate.sleep()

        self.is_obstacle_ahead = False

    def move_to_point(self, point: path_planner.Node):
        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        loop_rate = rospy.Rate(1000)

        while self.robot_position.calculate_distance(point) > move_tolerance:
            if self.is_shutdown_initiated or self.is_obstacle_ahead:
                return

            speed = angular_velocity * self.angular_difference(point)
            vel_msg.angular.z = min(angular_velocity, speed) + self.path_deviation

            self.velocity_publisher.publish(vel_msg)
            loop_rate.sleep()

    def rotate_to_goal(self, goal: path_planner.Node):
        vel_msg = Twist()
        loop_rate = rospy.Rate(1000)

        while abs(goal.theta - self.robot_position.theta) > rotate_tolerance:
            if self.is_shutdown_initiated:
                self.stop_moving()
                return

            speed = angular_velocity * (goal.theta - self.robot_position.theta)
            vel_msg.angular.z = min(angular_velocity, speed)
            self.velocity_publisher.publish(vel_msg)
            loop_rate.sleep()

        self.velocity_publisher.publish(Twist())

    def angular_difference(self, point: path_planner.Node) -> float:
        angle = atan2(point.y - self.robot_position.y, point.x - self.robot_position.x) \
                - self.robot_position.theta

        if angle <= -pi:  # Normalizing angle
            angle += 2 * pi
        elif angle > pi:
            angle -= 2 * pi

        return angle

