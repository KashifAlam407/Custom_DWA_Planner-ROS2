# ### This is simple custom dwa local planner node 

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

from visualization_msgs.msg import Marker


class SimpleDWANode(Node):
    def __init__(self):
        super().__init__('simple_dwa_node')

        ## Robot state 
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # laser regions
        self.front = None
        self.left = None
        self.right = None

        ## Goal
        self.goal_x = None
        self.goal_y = None
        self.goal_received = False

        ## DWA Parameters 
        self.dt = 0.1
        self.predict_time = 2.5    

        self.v_samples = [0.0, 0.05, 0.1, 0.15]
        self.w_samples = [-1.0, -0.5, 0.0, 0.5, 1.0]

        self.robot_radius = 0.2
        self.goal_tolerance = 0.2
        self.front_thresh = 0.6

        ## Creating Subscriber and Publisher
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/dwa_trajectories', 10)

        self.create_timer(self.dt, self.control_loop) 
        self.get_logger().info("Your Simple DWA node started")

    ## Callbacks 
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_cb(self, msg):
        front, left, right = [], [], []

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                r = 10.0

            # front (+,- 15) degree
            if -0.26 < angle < 0.26:
                front.append(r)
            # left 30 degree to 90 degree
            elif 0.52 < angle < 1.57:
                left.append(r)
            # right -90 degree to -30 degree
            elif -1.57 < angle < -0.52:
                right.append(r)

            angle += msg.angle_increment

        self.front = min(front) if front else 10.0
        self.left  = min(left)  if left else 10.0
        self.right = min(right) if right else 10.0

    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_received = True
        self.get_logger().info(f"🎯 Goal: {self.goal_x:.2f}, {self.goal_y:.2f}")

    ## Helper functions
    def normalize(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def distance_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    ## Trajectories
    def predict(self, v, w):
        traj = []
        x, y, yaw = self.x, self.y, self.yaw
        t = 0.0
        while t < self.predict_time:
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj.append((x, y))
            t += self.dt
        return traj

    def publish_trajectory(self, traj, mid, color):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dwa"
        marker.id = mid
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        for x, y in traj:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            marker.points.append(p)

        self.marker_pub.publish(marker)

    ## Main control loop
    def control_loop(self):
        if not self.goal_received or self.front is None:
            return

        if self.distance_to_goal() < self.goal_tolerance:
            self.cmd_pub.publish(Twist())
            return

        best_cost = float('inf')
        best_v, best_w = 0.0, 0.0
        best_traj = None
        marker_id = 0

        for v in self.v_samples:
            for w in self.w_samples:
                traj = self.predict(v, w)

                # publish candidate trajectories
                self.publish_trajectory(traj, marker_id, (1.0, 0.0, 0.0))
                marker_id += 1

                gx, gy = traj[-1]
                goal_cost = math.hypot(self.goal_x - gx, self.goal_y - gy)

                obstacle_cost = 0.0
                if self.front < self.front_thresh and v > 0.0:
                    obstacle_cost += 1.0
                    if self.left > self.right:
                        obstacle_cost += abs(w + 0.6)
                    else:
                        obstacle_cost += abs(w - 0.6)

                cost = goal_cost + obstacle_cost

                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_w = w
                    best_traj = traj

        # publish best trajectory
        if best_traj:
            self.publish_trajectory(best_traj, 999, (0.0, 1.0, 0.0))

        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(SimpleDWANode())
    rclpy.shutdown()



