#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def goto_goal(pos_x: float, pos_y: float):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pos_x
    goal.target_pose.pose.position.y = pos_y
    goal.target_pose.pose.orientation.w = 1.0

    move_base.send_goal(goal)

    move_status = move_base.wait_for_result(rospy.Duration(120)) 
    result_state = move_base.get_state()

    if move_status and result_state == GoalStatus.SUCCEEDED:
        return True

    return False


def on_shutdown():
    move_base.cancel_goal()


if __name__ == '__main__':
    try:
        rospy.init_node('goto_mover', anonymous=True)
        rospy.on_shutdown(on_shutdown)

        move_base = SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Connecting to action server")
        move_base.wait_for_server()

        pos_x = float(input('Input X coordinate:'))
        pos_y = float(input('Input Y coordinate:'))
        
        if goto_goal(pos_x, pos_y):
            rospy.loginfo("Goal reached successfully") 
        else:
            rospy.loginfo("Goal couldn't be reached")
    except rospy.ROSInterruptException:
        pass
    except ValueError:
        print("Error: not a number")
