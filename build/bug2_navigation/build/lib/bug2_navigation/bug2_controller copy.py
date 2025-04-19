import math
import numpy as np
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Bug1Navigator(Node):
    def __init__(self):
        super().__init__('bug2_controller')
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile=sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile=sensor_qos
        )
        
        self.timer = self.create_timer(0.1, self.move_robot)
        
        self.min_distance = float('inf')
        self.min_angle = 0.0
        self.goal_reach = False
        self.current_position = [0, 0]
        self.current_orientation = 0.0  # Yaw angle in radians
        self.goal_position = [4.0, 0.0]
        self.path = []
        self.obstacles = []

    def odom_callback(self, msg):
        self.current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.path.append(tuple(self.current_position))
        
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_orientation = euler_from_quaternion(quaternion)

    def scan_callback(self, msg):
        valid_ranges = np.array(msg.ranges)
        valid_ranges = np.where(np.isfinite(valid_ranges), valid_ranges, np.inf)  # Replace nan/inf with a large number

        self.min_distance = np.min(valid_ranges)  # Get the closest valid obstacle distance
        min_index = np.argmin(valid_ranges)  # Get the index of the closest obstacle
        self.min_angle = msg.angle_min + min_index * msg.angle_increment  # Angle in radians

        self.obstacles.clear()
        self.distance_by_angle = {}  # Dictionary to store distance for each angle

        for i, distance in enumerate(msg.ranges):
            if 0.1 < distance < 3.0:  # Ignore invalid or too far values
                angle_rad = msg.angle_min + i * msg.angle_increment  # Compute angle in radians
                angle_deg = (angle_rad * 180.0) / math.pi  # Convert to degrees
                angle_deg = (angle_deg + 360) % 360  # Normalize angle to [0, 360]

                # *Filter: Keep only angles in the range [0°-90°] and [270°-360°]*
                if (0 <= angle_deg <= 60) or (300 <= angle_deg <= 360):
                    obs_x = self.current_position[0] + distance * math.cos(angle_rad)
                    obs_y = self.current_position[1] + distance * math.sin(angle_rad)
                    self.obstacles.append((obs_x, obs_y))
                    self.distance_by_angle[angle_deg] = distance  # Store in degrees

    def move_robot(self):
        if self.goal_reach:
            return  # Stop sending commands if the goal is reached

        twist = Twist()

        # Compute differences between goal and current position
        goal_dx = self.goal_position[0] - self.current_position[0]
        goal_dy = self.goal_position[1] - self.current_position[1]
        sum_goal = math.sqrt(goal_dx**2 + goal_dy**2)

        # Log current state
        self.get_logger().info(
            f"del x: {goal_dx:.2f}, del y: {goal_dy:.2f}, "
            f"x: {self.current_position[0]:.2f}, y: {self.current_position[1]:.2f}, "
            f"sum_goal: {sum_goal:.2f}"
        )

        # 🚀 *Check if goal is reached (Threshold Increased)*
        if sum_goal <= 0.08:  # Increased threshold for better stopping
            self.get_logger().info("🎯 Goal reached! Stopping the robot.")
            self.stop_robot()

            # Publish zero velocity twice to ensure the robot stops completely
            rclpy.sleep(0.5)  # Small delay for ROS execution
            self.stop_robot()

            self.goal_reach = True
            return

        # Initialize movement
        move_forward = True
        obstacle_detected = False
        best_direction = math.atan2(goal_dy, goal_dx)  # Set initial best direction toward the goal

        # Check for obstacles within 0.3m range
        for angle, dist in self.distance_by_angle.items():
            if dist <= 0.3:
                obstacle_detected = True
                self.get_logger().info(f"🚧 Obstacle detected at angle {angle} with distance {dist:.2f}")

                # Decide how to turn based on obstacle position
                if 0 < angle < 60:
                    self.get_logger().info("Obstacle in front-left! Rotating right.")
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5  # Rotate right
                    move_forward = False
                    break  # Stop checking obstacles after setting avoidance action
                elif 300 < angle < 360:
                    self.get_logger().info("Obstacle in front-right! Rotating left.")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5  # Rotate left
                    move_forward = False
                    break  # Stop checking obstacles after setting avoidance action

        # If no obstacles, move toward the goal
        if move_forward and not obstacle_detected:
            # Compute the angular direction to the goal and adjust heading
            angular_error = best_direction - self.current_orientation  

            # Smoothly adjust heading towards the goal
            twist.linear.x = min(0.15, sum_goal)  # Move forward but limit speed
            twist.angular.z = 0.5 * angular_error  # Adjust heading towards goal

            # Ensure robot moves directly toward the goal if no obstacles
            if abs(angular_error) < 0.1:  # If the angle is small enough, move straight
                twist.angular.z = 0.0

        # Publish movement command
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)  # Stop the robot
        rclpy.sleep(0.2)  # Small delay to ensure the command is executed
        self.cmd_vel_pub.publish(twist)  # Send stop command again


def main(args=None):
    rclpy.init(args=args)
    node = Bug1Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
