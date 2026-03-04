#!/usr/bin/env python3

import rospy
import math
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from geometry_msgs.msg import Twist
from com760cw1_b01041751.msg import B01041751LeaderMessage
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

class FollowerATurtle:
    # Initialize follower A node with pose tracking and TF listening
    def __init__(self):
        rospy.init_node('follower_a_node', anonymous=True)

        self.follower_name = 'B01041751FollowerA'
        self.target_frame = 'target_FollowerA'

        self.my_pose = Pose()
        self.initial_pose = None
        self.following = False

        self.rate = rospy.Rate(10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.vel_pub = rospy.Publisher(
            f'/{self.follower_name}/cmd_vel',
            Twist,
            queue_size=10
        )

        rospy.Subscriber(f'/{self.follower_name}/pose', Pose, self.pose_callback)
        rospy.Subscriber('/leader_instructions', B01041751LeaderMessage, self.instruction_callback)

        rospy.loginfo("FollowerA ready")
        rospy.sleep(1.0)

    # Update pose and save initial position on first callback
    def pose_callback(self, pose):
        self.my_pose = pose
        if self.initial_pose is None:
            self.initial_pose = Pose()
            self.initial_pose.x = pose.x
            self.initial_pose.y = pose.y
            self.initial_pose.theta = pose.theta
            rospy.loginfo(f"FollowerA initial pose saved: ({pose.x:.2f}, {pose.y:.2f})")
        self.broadcast_tf()

    # Broadcast follower position as transformation frame
    def broadcast_tf(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = self.follower_name
        t.transform.translation.x = self.my_pose.x
        t.transform.translation.y = self.my_pose.y
        t.transform.translation.z = 0.0
        yaw = self.my_pose.theta
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.tf_broadcaster.sendTransform(t)

    # Handle formation and return instructions from leader
    def instruction_callback(self, msg):
        rospy.loginfo(f"FollowerA received instruction {msg.instructionID}: {msg.message}")
        self.following = False
        if msg.instructionID == 0:
            # Formation instruction
            self.move_to_formation()
        elif msg.instructionID == 1:
            # Return instruction
            self.return_to_initial()

    # Set follower pen color, width, and up/down state
    def set_pen(self, r, g, b, width, off):
        try:
            rospy.wait_for_service(f'/{self.follower_name}/set_pen', timeout=2.0)
            rospy.ServiceProxy(f'/{self.follower_name}/set_pen', SetPen)(r, g, b, width, off)
        except:
            pass

    # Query TF to get current target position in world coordinates
    def get_target_position_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'world',
                self.target_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            return tx, ty
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"FollowerA TF lookup failed: {e}")
            return None, None

    # Move to formation position relative to leader
    def move_to_formation(self):
        self.set_pen(255, 255, 255, 2, 0)
        rospy.loginfo("FollowerA moving to formation position")

        rate = rospy.Rate(10)
        for _ in range(300):
            tx, ty = self.get_target_position_from_tf()
            if tx is None:
                rate.sleep()
                continue

            dist = math.sqrt((tx - self.my_pose.x)**2 + (ty - self.my_pose.y)**2)
            if dist < 0.3:
                self.vel_pub.publish(Twist())
                rospy.loginfo("FollowerA reached formation position")
                self.following = True
                return

            angle = math.atan2(ty - self.my_pose.y, tx - self.my_pose.x)
            diff = angle - self.my_pose.theta
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi

            twist = Twist()
            twist.linear.x = min(1.5 * dist, 2.0)
            twist.angular.z = 4.0 * diff
            self.vel_pub.publish(twist)
            rate.sleep()

        self.vel_pub.publish(Twist())
        self.following = True

    # Continuously track and maintain formation with leader
    def follow_leader(self):
        tx, ty = self.get_target_position_from_tf()
        if tx is None:
            return

        dist = math.sqrt((tx - self.my_pose.x)**2 + (ty - self.my_pose.y)**2)
        if dist < 0.15:
            self.vel_pub.publish(Twist())
            return

        angle = math.atan2(ty - self.my_pose.y, tx - self.my_pose.x)
        diff = angle - self.my_pose.theta
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi

        twist = Twist()
        twist.linear.x = min(1.5 * dist, 2.0)
        twist.angular.z = 4.0 * diff
        self.vel_pub.publish(twist)

    # Teleport back to initial spawn position
    def return_to_initial(self):
        if self.initial_pose is None:
            rospy.logwarn("FollowerA has no initial pose")
            return
        self.set_pen(255, 255, 255, 2, 1)
        try:
            rospy.wait_for_service(f'/{self.follower_name}/teleport_absolute', timeout=2.0)
            teleport = rospy.ServiceProxy(f'/{self.follower_name}/teleport_absolute', TeleportAbsolute)
            teleport(self.initial_pose.x, self.initial_pose.y, self.initial_pose.theta)
            rospy.loginfo(f"FollowerA teleported to initial: ({self.initial_pose.x:.2f}, {self.initial_pose.y:.2f})")
            self.vel_pub.publish(Twist())
        except Exception as e:
            rospy.logerr(f"FollowerA teleport failed: {e}")
        self.set_pen(255, 255, 255, 2, 0)

    # Main control loop for follower behavior
    def run(self):
        rospy.loginfo("FollowerA running")
        while not rospy.is_shutdown():
            if self.following:
                self.follow_leader()
            self.rate.sleep()

# Entry point for follower A node
if __name__ == '__main__':
    try:
        follower = FollowerATurtle()
        follower.run()
    except rospy.ROSInterruptException:
        pass
