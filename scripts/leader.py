#!/usr/bin/env python3

import rospy
import random
import math
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from com760cw1_b01041751.msg import B01041751LeaderMessage
from com760cw1_b01041751.srv import B01041751GoToTarget, B01041751GoToTargetRequest, B01041751GoToTargetResponse
import tf2_ros
import geometry_msgs.msg

WINDOW_MIN = 0.5
WINDOW_MAX = 10.5
BORDER_THRESHOLD = 1.2
LEADER_SPEED = 2.0
CENTER_X = 5.5
CENTER_Y = 5.5

class LeaderTurtle:
    # Initialize leader node with publishers, subscribers, and services
    def __init__(self):
        rospy.init_node('leader_node', anonymous=True)

        self.leader_name = 'B01041751Leader'
        self.follower_a_name = 'B01041751FollowerA'
        self.follower_b_name = 'B01041751FollowerB'

        self.leader_pose = Pose()
        self.leader_pose.x = CENTER_X
        self.leader_pose.y = CENTER_Y
        self.rate = rospy.Rate(10)
        self.cycles_completed = 0

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo("Waiting for turtlesim...")
        rospy.wait_for_service('/spawn')
        rospy.wait_for_service('/kill')

        self.spawn_turtles()

        self.instruction_pub = rospy.Publisher(
            '/leader_instructions',
            B01041751LeaderMessage,
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
            'B01041751GoToTarget',
            B01041751GoToTarget,
            self.go_to_target_handler
        )

        rospy.loginfo("Leader ready")
        rospy.sleep(0.5)

    # Spawn three turtles in random positions
    def spawn_turtles(self):
        kill = rospy.ServiceProxy('/kill', Kill)
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        for name in ['turtle1']:
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

    # Update leader pose and broadcast transformation frames
    def pose_callback(self, pose):
        self.leader_pose = pose
        self.broadcast_tf()

    # Broadcast leader and follower target frames to TF
    def broadcast_tf(self):
        now = rospy.Time.now()
        q = self.euler_to_quaternion(self.leader_pose.theta)
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

    # Convert yaw angle to quaternion representation
    def euler_to_quaternion(self, yaw):
        return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))

    # Set turtle pen color, width, and up/down state
    def set_pen(self, turtle_name, r, g, b, width, off):
        try:
            rospy.wait_for_service(f'/{turtle_name}/set_pen', timeout=2.0)
            set_pen = rospy.ServiceProxy(f'/{turtle_name}/set_pen', SetPen)
            set_pen(r, g, b, width, off)
        except:
            pass

    # Handle service request to move turtle to goal
    def go_to_target_handler(self, req):
        success = self.move_turtle_to_goal(
            req.turtle_name, req.goal_x, req.goal_y, req.tolerance
        )
        resp = B01041751GoToTargetResponse()
        resp.success = success
        return resp

    # Move turtle to goal position using velocity control
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

    # Publish instruction message to followers
    def send_instruction(self, instruction_id, message):
        msg = B01041751LeaderMessage()
        msg.instructionID = instruction_id
        msg.message = message
        rospy.sleep(0.3)
        self.instruction_pub.publish(msg)
        rospy.loginfo(f"Sent instruction {instruction_id}: {message}")

    # Move leader straight in current direction until hitting window edge
    def move_straight_until_border(self):
        self.set_pen(self.leader_name, 255, 0, 0, 2, 0)
        twist = Twist()
        twist.linear.x = LEADER_SPEED
        rate = rospy.Rate(10)
        rospy.loginfo("Leader moving with red pen...")
        while not rospy.is_shutdown():
            self.vel_pub.publish(twist)
            rate.sleep()

            theta = self.leader_pose.theta
            dx = math.cos(theta)
            dy = math.sin(theta)
            eps = 0.05
            if abs(dx) >= abs(dy):
                if dx > 0 and self.leader_pose.x >= WINDOW_MAX - eps:
                    self.vel_pub.publish(Twist())
                    rospy.loginfo("Leader hit right border")
                    break
                if dx < 0 and self.leader_pose.x <= WINDOW_MIN + eps:
                    self.vel_pub.publish(Twist())
                    rospy.loginfo("Leader hit left border")
                    break
            else:
                if dy > 0 and self.leader_pose.y >= WINDOW_MAX - eps:
                    self.vel_pub.publish(Twist())
                    rospy.loginfo("Leader hit top border")
                    break
                if dy < 0 and self.leader_pose.y <= WINDOW_MIN + eps:
                    self.vel_pub.publish(Twist())
                    rospy.loginfo("Leader hit bottom border")
                    break

    # Move leader back to center of canvas
    def return_leader_to_center(self):
        self.set_pen(self.leader_name, 255, 255, 255, 2, 0)
        rospy.loginfo("Leader returning to center...")
        success = self.move_turtle_to_goal(self.leader_name, CENTER_X, CENTER_Y, 0.05)
        
        if not success:
            rospy.logwarn("Movement failed, teleporting...")
            self._teleport_leader_center()

    # Fallback: teleport leader to center using service
    def _teleport_leader_center(self):
        try:
            rospy.wait_for_service(f'/{self.leader_name}/teleport_absolute', timeout=2.0)
            teleport = rospy.ServiceProxy(f'/{self.leader_name}/teleport_absolute', TeleportAbsolute)
            teleport(CENTER_X, CENTER_Y, 0.0)
        except Exception as e:
            rospy.logerr(f"Teleport failed: {e}")
        else:
            try:
                self.leader_pose.x = CENTER_X
                self.leader_pose.y = CENTER_Y
                self.leader_pose.theta = 0.0
                self.vel_pub.publish(Twist())
                rospy.sleep(0.05)
            except Exception:
                pass

    # Clear turtlesim canvas
    def clear_screen(self):
        try:
            rospy.wait_for_service('/clear', timeout=2.0)
            rospy.ServiceProxy('/clear', Empty)()
        except:
            pass

    # Draw circle at specified position with target duration
    def draw_circle_fast(self, turtle_name, center_x, center_y, radius, start_x, start_y, heading, duration=2.0):
        vel_pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
        angular_vel = (2 * math.pi) / duration
        linear_vel = radius * angular_vel
        
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < (duration + 0.2) and not rospy.is_shutdown():
            vel_pub.publish(twist)
            rate.sleep()
        vel_pub.publish(twist)
        rospy.sleep(0.1)
        vel_pub.publish(Twist())

    # Draw sunflower pattern with six petals around center
    def draw_creative_flower(self):
        rospy.loginfo("Starting creative sunflower drawing mode!")
        try:
            yellow = (255, 200, 0)
            self.set_pen(self.leader_name, yellow[0], yellow[1], yellow[2], 2, 1)
            self.set_pen(self.follower_a_name, yellow[0], yellow[1], yellow[2], 2, 1)
            self.set_pen(self.follower_b_name, yellow[0], yellow[1], yellow[2], 2, 1)
            rospy.sleep(0.5)
            num_petals = 6
            petal_distance = 1.2
            petal_radius = 0.5
            
            turtle_names = [self.leader_name, self.follower_a_name, self.follower_b_name]
            
            for petal_idx in range(num_petals):
                angle = (petal_idx / num_petals) * 2 * math.pi
                px = CENTER_X + petal_distance * math.cos(angle)
                py = CENTER_Y + petal_distance * math.sin(angle)
                turtle = turtle_names[petal_idx % 3]
                start_x = px + petal_radius
                start_y = py
                heading = math.pi
                self.set_pen(turtle, yellow[0], yellow[1], yellow[2], 2, 1)
                rospy.sleep(0.1)
                try:
                    rospy.wait_for_service(f'/{turtle}/teleport_absolute', timeout=1.0)
                    teleport = rospy.ServiceProxy(f'/{turtle}/teleport_absolute', TeleportAbsolute)
                    teleport(start_x, start_y, heading)
                except Exception as e:
                    rospy.logwarn(f"Teleport for {turtle} failed: {e}")
                rospy.sleep(0.1)
                self.set_pen(turtle, yellow[0], yellow[1], yellow[2], 2, 0)
                rospy.sleep(0.1)
                self.draw_circle_fast(turtle, px, py, petal_radius, start_x, start_y, heading, duration=1.6)
                rospy.sleep(0.2)
            center_radius = 0.6
            center_start_x = CENTER_X + center_radius
            center_start_y = CENTER_Y
            center_heading = math.pi
            self.set_pen(self.leader_name, yellow[0], yellow[1], yellow[2], 2, 1)
            rospy.sleep(0.1)
            try:
                rospy.wait_for_service(f'/{self.leader_name}/teleport_absolute', timeout=1.0)
                teleport = rospy.ServiceProxy(f'/{self.leader_name}/teleport_absolute', TeleportAbsolute)
                teleport(center_start_x, center_start_y, center_heading)
            except Exception as e:
                rospy.logwarn(f"Teleport failed: {e}")
            rospy.sleep(0.1)
            self.set_pen(self.leader_name, yellow[0], yellow[1], yellow[2], 2, 0)
            rospy.sleep(0.1)
            
            self.draw_circle_fast(self.leader_name, CENTER_X, CENTER_Y, center_radius, center_start_x, center_start_y, center_heading, duration=1.8)
            rospy.loginfo("Sunflower drawing complete")
            rospy.loginfo("Moving turtles away...")
            self.set_pen(self.leader_name, 255, 255, 255, 2, 1)
            self.set_pen(self.follower_a_name, 255, 255, 255, 2, 1)
            self.set_pen(self.follower_b_name, 255, 255, 255, 2, 1)
            self.move_turtle_to_goal(self.leader_name, 2.0, 2.0, 0.1)
            self.move_turtle_to_goal(self.follower_a_name, 9.0, 2.0, 0.1)
            self.move_turtle_to_goal(self.follower_b_name, 5.5, 8.5, 0.1)
            rospy.loginfo("Turtles waiting away from center...")
            rospy.sleep(2.0)
        except Exception as e:
            rospy.logerr(f"Error during flower drawing: {e}")
        finally:
            self.vel_pub.publish(Twist())
            self.set_pen(self.leader_name, 255, 255, 255, 2, 1)
            self.set_pen(self.follower_a_name, 255, 255, 255, 2, 1)
            self.set_pen(self.follower_b_name, 255, 255, 255, 2, 1)

    # Main control loop for leader behavior
    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("=== CYCLE START: Sending Formation instruction ===")
            self.send_instruction(0, 'Formation')
            rospy.sleep(5.0)
            angle = random.uniform(0, 2 * math.pi)
            rospy.loginfo(f"Turning to angle {math.degrees(angle):.1f} degrees")
            try:
                rospy.wait_for_service(f'/{self.leader_name}/teleport_absolute', timeout=2.0)
                teleport = rospy.ServiceProxy(f'/{self.leader_name}/teleport_absolute', TeleportAbsolute)
                teleport(self.leader_pose.x, self.leader_pose.y, angle)
            except Exception as e:
                rospy.logerr(f"Turn failed: {e}")
            rospy.sleep(0.5)
            self.move_straight_until_border()
            rospy.loginfo("Sending Return instruction")
            self.send_instruction(1, 'Return')
            rospy.sleep(1.0)
            self.return_leader_to_center()
            rospy.sleep(1.0)
            self.clear_screen()
            rospy.loginfo("=== CYCLE COMPLETE ===")
            self.cycles_completed += 1
            if self.cycles_completed >= 2:
                rospy.loginfo("\n\n*** ENTERING CREATIVE MODE AFTER 2 CYCLES ***\n\n")
                self.clear_screen()
                self.draw_creative_flower()
                rospy.loginfo("Returning to center after creative mode...")
                self.set_pen(self.leader_name, 255, 255, 255, 2, 1)
                self.send_instruction(1, 'Return')
                rospy.sleep(0.5)
                self.return_leader_to_center()
                self.set_pen(self.leader_name, 255, 255, 255, 2, 1)
                rospy.sleep(1.0)
                self.cycles_completed = 0
                rospy.loginfo("\n\n*** RESUMING NORMAL ITERATIONS ***\n\n")
                self.clear_screen()
                rospy.sleep(1.0)

# Entry point for leader node
if __name__ == '__main__':
    try:
        leader = LeaderTurtle()
        leader.run()
    except rospy.ROSInterruptException:
        pass
