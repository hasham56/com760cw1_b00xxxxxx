#!/usr/bin/env python3
import rospy
import random
from std_srvs.srv import Empty

# Set random background color for turtlesim
def set_random_background():
    rospy.init_node('set_background_node', anonymous=True)
    
    rospy.set_param('/turtlesim/background_r', random.randint(0, 255))
    rospy.set_param('/turtlesim/background_g', random.randint(0, 255))
    rospy.set_param('/turtlesim/background_b', random.randint(0, 255))

# Entry point for set_background node
if __name__ == '__main__':
    try:
        set_random_background()
    except rospy.ROSInterruptException:
        pass