#!/usr/bin/env python
import rospy
from path_planner import PathPlanner

if __name__ == '__main__':
    try:
        rospy.init_node('path_planner', anonymous=True)
        move_base = PathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
