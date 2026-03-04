#!/usr/bin/env python3
import rospy
import random
from std_srvs.srv import Empty

def set_random_background():
    # Initialize the node
    rospy.init_node('set_background_node', anonymous=True)
    
    # 1. Use the correct namespace: /turtlesim/background_x
    # Most turtlesim nodes expect the 'turtlesim' prefix.
    rospy.set_param('/turtlesim/background_r', random.randint(0, 255))
    rospy.set_param('/turtlesim/background_g', random.randint(0, 255))
    rospy.set_param('/turtlesim/background_b', random.randint(0, 255))

    rospy.loginfo("Waiting for /clear service...")
    rospy.wait_for_service('/clear')
    
    try:
        # 2. Give the Parameter Server a split second to update
        rospy.sleep(0.1)
        
        # 3. Call the clear service to refresh the background
        clear_bg = rospy.ServiceProxy('/clear', Empty)
        clear_bg()
        
        rospy.loginfo("Background color updated successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        set_random_background()
    except rospy.ROSInterruptException:
        pass