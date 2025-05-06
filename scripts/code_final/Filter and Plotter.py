#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Twist, TransformStamped
import numpy as np
from collections import deque
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import tf2_ros
from tf_transformations import euler_from_quaternion
import time
from tf2_ros import TransformBroadcaster
from queue import PriorityQueue

class IMUWaypointNavigator(Node):
    def __init__(self):
        super().__init__('imu_waypoint_navigator')

        self.create_subscription(Imu, '/esp_05/imu_data', self.imu_callback, 10)
        self.create_subscription(PointStamped, '/aruco_1', self.ground_callback, 10)
        self.create_subscription(Odometry, '/rpi_05/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/rpi_05/cmd_vel', 10)
        self.broadcaster = TransformBroadcaster(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.dt = 0.02
        self.bias = np.zeros(2)
        self.velocity = np.zeros(2)
        self.position = np.zeros(2)
        self.initial_position_set = False
        self.initial_position = np.zeros(2)
        self.last_acc = np.zeros(2)
        self.esp_points = deque(maxlen=100)
        self.rpi_points = deque(maxlen=100)
        self.ground_received = False

        self.stationary_threshold = 0.01
        self.stationary_duration = 10
        self.last_movement_time = time.time()
        self.prev_position = None
        self.waypoint = None

        self.grid_resolution = 0.1
        self.grid_size = int(2.0 / self.grid_resolution)  # 20x20 grid
        self.path = []
        self.current_target_idx = 0

        self.fig, self.ax = plt.subplots()
        self.esp_plot, = self.ax.plot([], [], 'ro-', label='ESP32')
        self.rpi_plot, = self.ax.plot([], [], 'bo-', label='TurtleBot')
        self.waypoint_plot, = self.ax.plot([], [], 'gx', label='Waypoint')
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_title('Waypoint Navigation (2m x 2m Area)')
        self.ax.set_aspect('equal')
        self.ax.legend()
        plt.ion()
        plt.show()

        self.create_timer(self.dt, self.timer_callback)

    def to_grid(self, pos):
        x = int((pos[0] + 1.0) / self.grid_resolution)
        y = int((pos[1] + 1.0) / self.grid_resolution)
        return (x, y)

    def to_world(self, grid_pos):
        x = (grid_pos[0] * self.grid_resolution) - 1.0 + self.grid_resolution / 2
        y = (grid_pos[1] * self.grid_resolution) - 1.0 + self.grid_resolution / 2
        return np.array([x, y])

    def astar(self, start, goal, grid):
        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                neighbor = (current[0]+dx, current[1]+dy)
                if 0 <= neighbor[0] < self.grid_size and 0 <= neighbor[1] < self.grid_size:
                    tentative_g = g_score[current] + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                        open_set.put((f_score[neighbor], neighbor))
        return []

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)

    def ground_callback(self, msg):
        if not self.ground_received:
            self.position = np.array([msg.point.x, msg.point.y])
            self.initial_position = self.position.copy()
            self.initial_position_set = True
            self.ground_received = True
            self.get_logger().info(f"[INIT] Ground truth set: {self.position}")

    def imu_callback(self, msg):
        if not self.ground_received or not self.initial_position_set:
            return

        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y])
        acc_corrected = acc - self.bias

        if np.linalg.norm(acc - self.last_acc) < 0.03:
            self.velocity = np.zeros(2)
        else:
            self.velocity += acc_corrected * self.dt

        self.position += self.velocity * self.dt
        self.last_acc = acc

        relative_position = self.position - self.initial_position
        clipped_pos = np.clip(relative_position, -1.0, 1.0)
        self.esp_points.append(clipped_pos.copy())

        if self.prev_position is not None:
            dist = np.linalg.norm(relative_position - self.prev_position)
            if dist < self.stationary_threshold:
                if time.time() - self.last_movement_time > self.stationary_duration and self.waypoint is None:
                    self.waypoint = clipped_pos.copy()
                    self.get_logger().info(f"[WAYPOINT] Created at: {self.waypoint}")
                    start = self.to_grid(np.array([0.0, 0.0]))
                    goal = self.to_grid(self.waypoint)
                    grid = np.zeros((self.grid_size, self.grid_size))
                    self.path = self.astar(start, goal, grid)
                    self.current_target_idx = 0
            else:
                self.last_movement_time = time.time()

        self.prev_position = relative_position.copy()

    def get_rpi_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            pos = trans.transform.translation
            quat = trans.transform.rotation
            yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
            return np.array([pos.x, pos.y]) - self.initial_position, yaw
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return np.array([0.0, 0.0]), 0.0

    def navigate_to_waypoint(self, rpi_pos, yaw):
        if not self.path or self.current_target_idx >= len(self.path):
            return

        target_world = self.to_world(self.path[self.current_target_idx])
        direction = target_world - rpi_pos
        dist = np.linalg.norm(direction)

        if dist < 0.1:
            self.current_target_idx += 1
            return

        desired_heading = np.arctan2(direction[1], direction[0])
        heading_error = np.arctan2(np.sin(desired_heading - yaw), np.cos(desired_heading - yaw))

        cmd = Twist()
        if abs(heading_error) > 0.1:
            cmd.angular.z = np.clip(heading_error, -0.5, 0.5)
        else:
            cmd.linear.x = min(0.2, dist)
            cmd.angular.z = np.clip(heading_error, -0.3, 0.3)

        self.cmd_pub.publish(cmd)

    def timer_callback(self):
        if not self.ground_received or not self.initial_position_set:
            return

        rpi_pos, yaw = self.get_rpi_transform()
        self.rpi_points.append(np.clip(rpi_pos, -1.0, 1.0))

        if self.esp_points:
            x_esp, y_esp = zip(*self.esp_points)
            self.esp_plot.set_data(x_esp, y_esp)

        if self.rpi_points:
            x_rpi, y_rpi = zip(*self.rpi_points)
            self.rpi_plot.set_data(x_rpi, y_rpi)

        if self.waypoint is not None:
            self.waypoint_plot.set_data([self.waypoint[0]], [self.waypoint[1]])
            self.navigate_to_waypoint(rpi_pos, yaw)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = IMUWaypointNavigator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

