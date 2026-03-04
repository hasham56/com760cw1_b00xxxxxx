#!/usr/bin/env python3

import rospy
import random
import math
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from com760cw1_b00xxxxxx.msg import B00xxxxxxLeaderMessage
from com760cw1_b00xxxxxx.srv import B00xxxxxxGoToTarget, B00xxxxxxGoToTargetRequest, B00xxxxxxGoToTargetResponse
import tf2_ros
import geometry_msgs.msg

WINDOW_MIN = 0.5
WINDOW_MAX = 10.5
BORDER_THRESHOLD = 1.2
LEADER_SPEED = 1.0
CENTER_X = 5.5
CENTER_Y = 5.5

class LeaderTurtle:
    def __init__(self):
        rospy.init_node('leader_node', anonymous=True)

        self.leader_name = 'b00xxxxxxLeader'
        self.follower_a_name = 'b00xxxxxxFollowerA'
        self.follower_b_name = 'b00xxxxxxFollowerB'

        self.leader_pose = Pose()
        self.leader_pose.x = CENTER_X
        self.leader_pose.y = CENTER_Y
        self.rate = rospy.Rate(10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo("Waiting for turtlesim...")
        rospy.wait_for_service('/spawn')
        rospy.wait_for_service('/kill')

        self.setup_background()
        self.spawn_turtles()

        self.instruction_pub = rospy.Publisher(
            '/leader_instructions',
            B00xxxxxxLeaderMessage,
            queue_size=10
        )

        self.vel_pub = rospy.Publisher(
            f'/{self.leader_name}/cmd_vel',
            Twist,
            queue_size=10
        )

        rospy.Subscriber(
            f'/{self.leader_name}/pose',
            Pose,
            self.pose_callback
        )

        rospy.Service(
            'b00xxxxxxgototarget',
            B00xxxxxxGoToTarget,
            self.go_to_target_handler
        )

        rospy.loginfo("Leader ready")
        rospy.sleep(2.0)

    def setup_background(self):
        """Set random background color"""
        rospy.set_param('/turtlesim/background_r', random.randint(0, 200))
        rospy.set_param('/turtlesim/background_g', random.randint(0, 200))
        rospy.set_param('/turtlesim/background_b', random.randint(0, 200))
        try:
            rospy.wait_for_service('/clear', timeout=3.0)
            rospy.ServiceProxy('/clear', Empty)()
        except:
            pass

    def spawn_turtles(self):
        kill = rospy.ServiceProxy('/kill', Kill)
        spawn = rospy.ServiceProxy('/spawn', Spawn)

        for name in [self.leader_name, self.follower_a_name, self.follower_b_name, 'turtle1']:
            try:
                kill(name)
            except:
                pass

        rospy.sleep(0.5)

        spawn(CENTER_X, CENTER_Y, 0.0, self.leader_name)

        ax = random.uniform(1.0, 3.5)
        ay = random.uniform(1.0, 3.5)
        at = random.uniform(0, 2 * math.pi)
        spawn(ax, ay, at, self.follower_a_name)

        bx = random.uniform(7.0, 9.5)
        by = random.uniform(7.0, 9.5)
        bt = random.uniform(0, 2 * math.pi)
        spawn(bx, by, bt, self.follower_b_name)

        rospy.loginfo(f"Spawned: Leader at center, A at ({ax:.1f},{ay:.1f}), B at ({bx:.1f},{by:.1f})")

    def pose_callback(self, pose):
        self.leader_pose = pose
        self.broadcast_tf()

    def broadcast_tf(self):
        now = rospy.Time.now()
        q = self.euler_to_quaternion(self.leader_pose.theta)

        # Broadcast leader frame
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "world"
        t.child_frame_id = self.leader_name
        t.transform.translation.x = self.leader_pose.x
        t.transform.translation.y = self.leader_pose.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Broadcast target frame for FollowerA: 1m to the LEFT (positive Y in leader frame)
        ta = geometry_msgs.msg.TransformStamped()
        ta.header.stamp = now
        ta.header.frame_id = self.leader_name
        ta.child_frame_id = "target_FollowerA"
        ta.transform.translation.x = -0.8
        ta.transform.translation.y = 1.0
        ta.transform.translation.z = 0.0
        ta.transform.rotation.x = 0.0
        ta.transform.rotation.y = 0.0
        ta.transform.rotation.z = 0.0
        ta.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(ta)

        # Broadcast target frame for FollowerB: 1m to the RIGHT (negative Y in leader frame)
        tb = geometry_msgs.msg.TransformStamped()
        tb.header.stamp = now
        tb.header.frame_id = self.leader_name
        tb.child_frame_id = "target_FollowerB"
        tb.transform.translation.x = -0.8
        tb.transform.translation.y = -1.0
        tb.transform.translation.z = 0.0
        tb.transform.rotation.x = 0.0
        tb.transform.rotation.y = 0.0
        tb.transform.rotation.z = 0.0
        tb.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tb)

    def euler_to_quaternion(self, yaw):
        return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))

    def set_pen(self, turtle_name, r, g, b, width, off):
        try:
            rospy.wait_for_service(f'/{turtle_name}/set_pen', timeout=2.0)
            set_pen = rospy.ServiceProxy(f'/{turtle_name}/set_pen', SetPen)
            set_pen(r, g, b, width, off)
        except:
            pass

    def go_to_target_handler(self, req):
        success = self.move_turtle_to_goal(
            req.turtle_name, req.goal_x, req.goal_y, req.tolerance
        )
        resp = B00xxxxxxGoToTargetResponse()
        resp.success = success
        return resp

    def move_turtle_to_goal(self, turtle_name, goal_x, goal_y, tolerance):
        vel_pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
        current_pose = [Pose()]

        def cb(p):
            current_pose[0] = p

        sub = rospy.Subscriber(f'/{turtle_name}/pose', Pose, cb)
        rospy.sleep(0.5)

        rate = rospy.Rate(10)
        for _ in range(300):
            p = current_pose[0]
            dist = math.sqrt((goal_x - p.x)**2 + (goal_y - p.y)**2)

            if dist < tolerance:
                vel_pub.publish(Twist())
                sub.unregister()
                return True

            angle = math.atan2(goal_y - p.y, goal_x - p.x)
            diff = angle - p.theta
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi

            twist = Twist()
            twist.linear.x = min(1.5 * dist, 2.0)
            twist.angular.z = 4.0 * diff
            vel_pub.publish(twist)
            rate.sleep()

        vel_pub.publish(Twist())
        sub.unregister()
        return False

    def send_instruction(self, instruction_id, message):
        msg = B00xxxxxxLeaderMessage()
        msg.instructionID = instruction_id
        msg.message = message
        rospy.sleep(0.3)
        self.instruction_pub.publish(msg)
        rospy.loginfo(f"Sent instruction {instruction_id}: {message}")

    def is_near_border(self):
        x = self.leader_pose.x
        y = self.leader_pose.y
        return (x <= WINDOW_MIN + BORDER_THRESHOLD or
                x >= WINDOW_MAX - BORDER_THRESHOLD or
                y <= WINDOW_MIN + BORDER_THRESHOLD or
                y >= WINDOW_MAX - BORDER_THRESHOLD)

    def move_straight_until_border(self):
        self.set_pen(self.leader_name, 255, 0, 0, 2, 0)
        twist = Twist()
        twist.linear.x = LEADER_SPEED
        rate = rospy.Rate(10)
        rospy.loginfo("Leader moving with red pen...")
        while not rospy.is_shutdown():
            if self.is_near_border():
                self.vel_pub.publish(Twist())
                rospy.loginfo("Leader reached border")
                break
            self.vel_pub.publish(twist)
            rate.sleep()

    def return_leader_to_center(self):
        self.set_pen(self.leader_name, 255, 255, 255, 2, 0)
        rospy.loginfo("Leader returning to center via GoToTarget service")
        try:
            rospy.wait_for_service('b00xxxxxxgototarget', timeout=3.0)
            go = rospy.ServiceProxy('b00xxxxxxgototarget', B00xxxxxxGoToTarget)
            req = B00xxxxxxGoToTargetRequest()
            req.goal_x = CENTER_X
            req.goal_y = CENTER_Y
            req.tolerance = 0.4
            req.turtle_name = self.leader_name
            result = go(req)
            if not result.success:
                rospy.logwarn("GoToTarget failed, teleporting leader to center")
                self._teleport_leader_center()
        except Exception as e:
            rospy.logerr(f"GoToTarget error: {e}, teleporting")
            self._teleport_leader_center()

    def _teleport_leader_center(self):
        try:
            rospy.wait_for_service(f'/{self.leader_name}/teleport_absolute', timeout=2.0)
            teleport = rospy.ServiceProxy(f'/{self.leader_name}/teleport_absolute', TeleportAbsolute)
            teleport(CENTER_X, CENTER_Y, 0.0)
        except Exception as e:
            rospy.logerr(f"Teleport failed: {e}")

    def clear_screen(self):
        try:
            rospy.wait_for_service('/clear', timeout=2.0)
            rospy.ServiceProxy('/clear', Empty)()
        except:
            pass

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("=== CYCLE START: Sending Formation instruction ===")
            self.send_instruction(0, 'Formation')

            rospy.loginfo("Waiting 10s for followers to form up...")
            rospy.sleep(10.0)

            # Turn to random direction
            angle = random.uniform(0, 2 * math.pi)
            rospy.loginfo(f"Turning to angle {math.degrees(angle):.1f} degrees")
            try:
                rospy.wait_for_service(f'/{self.leader_name}/teleport_absolute', timeout=2.0)
                teleport = rospy.ServiceProxy(f'/{self.leader_name}/teleport_absolute', TeleportAbsolute)
                teleport(self.leader_pose.x, self.leader_pose.y, angle)
            except Exception as e:
                rospy.logerr(f"Turn failed: {e}")

            rospy.sleep(1.0)

            # Move to border
            self.move_straight_until_border()

            # Send return instruction
            rospy.loginfo("Sending Return instruction")
            self.send_instruction(1, 'Return')
            rospy.sleep(2.0)

            # Leader returns to center
            self.return_leader_to_center()
            rospy.sleep(2.0)

            # Clear and repeat
            self.clear_screen()
            rospy.loginfo("=== CYCLE COMPLETE ===")

if __name__ == '__main__':
    try:
        leader = LeaderTurtle()
        leader.run()
    except rospy.ROSInterruptException:
        pass
