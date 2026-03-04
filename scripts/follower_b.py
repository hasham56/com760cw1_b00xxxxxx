#!/usr/bin/env python3

import rospy
import math
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from geometry_msgs.msg import Twist
from com760cw1_b00xxxxxx.msg import B00xxxxxxLeaderMessage
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

class FollowerBTurtle:
    def __init__(self):
        rospy.init_node('follower_b_node', anonymous=True)

        self.follower_name = 'b00xxxxxxFollowerB'
        self.target_frame = 'target_FollowerB'

        self.my_pose = Pose()
        self.initial_pose = None
        self.following = False

        self.rate = rospy.Rate(10)

        # TF broadcaster for this follower's frame
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # TF listener to look up target frame from leader
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.vel_pub = rospy.Publisher(
            f'/{self.follower_name}/cmd_vel',
            Twist,
            queue_size=10
        )

        rospy.Subscriber(f'/{self.follower_name}/pose', Pose, self.pose_callback)
        rospy.Subscriber('/leader_instructions', B00xxxxxxLeaderMessage, self.instruction_callback)

        rospy.loginfo("FollowerB ready")
        rospy.sleep(1.0)

    def pose_callback(self, pose):
        self.my_pose = pose
        # Save very first pose as initial position
        if self.initial_pose is None:
            self.initial_pose = Pose()
            self.initial_pose.x = pose.x
            self.initial_pose.y = pose.y
            self.initial_pose.theta = pose.theta
            rospy.loginfo(f"FollowerB initial pose saved: ({pose.x:.2f}, {pose.y:.2f})")
        self.broadcast_tf()

    def broadcast_tf(self):
        """Broadcast this follower's position as a TF frame"""
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

    def instruction_callback(self, msg):
        rospy.loginfo(f"FollowerB received instruction {msg.instructionID}: {msg.message}")
        self.following = False
        if msg.instructionID == 0:
            # Formation instruction
            self.move_to_formation()
        elif msg.instructionID == 1:
            # Return instruction
            self.return_to_initial()

    def set_pen(self, r, g, b, width, off):
        try:
            rospy.wait_for_service(f'/{self.follower_name}/set_pen', timeout=2.0)
            rospy.ServiceProxy(f'/{self.follower_name}/set_pen', SetPen)(r, g, b, width, off)
        except:
            pass

    def get_target_position_from_tf(self):
        """Use TF to find where target_FollowerB is in world coordinates"""
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
            rospy.logwarn(f"FollowerB TF lookup failed: {e}")
            return None, None

    def move_to_formation(self):
        """Move to formation position using TF to find target"""
        self.set_pen(0, 0, 255, 2, 0)  # Blue pen
        rospy.loginfo("FollowerB moving to formation position")

        rate = rospy.Rate(10)
        for _ in range(300):
            tx, ty = self.get_target_position_from_tf()
            if tx is None:
                rate.sleep()
                continue

            dist = math.sqrt((tx - self.my_pose.x)**2 + (ty - self.my_pose.y)**2)
            if dist < 0.3:
                self.vel_pub.publish(Twist())
                rospy.loginfo("FollowerB reached formation position")
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

    def follow_leader(self):
        """Continuously track target_FollowerB frame using TF"""
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

    def return_to_initial(self):
        """Teleport back to original random spawn position with white pen"""
        if self.initial_pose is None:
            rospy.logwarn("FollowerB has no initial pose")
            return
        self.set_pen(255, 255, 255, 2, 1)
        try:
            rospy.wait_for_service(f'/{self.follower_name}/teleport_absolute', timeout=2.0)
            teleport = rospy.ServiceProxy(f'/{self.follower_name}/teleport_absolute', TeleportAbsolute)
            teleport(self.initial_pose.x, self.initial_pose.y, self.initial_pose.theta)
            rospy.loginfo(f"FollowerB teleported to initial: ({self.initial_pose.x:.2f}, {self.initial_pose.y:.2f})")
        except Exception as e:
            rospy.logerr(f"FollowerB teleport failed: {e}")
        self.set_pen(255, 255, 255, 2, 0)

    def run(self):
        rospy.loginfo("FollowerB running")
        while not rospy.is_shutdown():
            if self.following:
                self.follow_leader()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = FollowerBTurtle()
        follower.run()
    except rospy.ROSInterruptException:
        pass
