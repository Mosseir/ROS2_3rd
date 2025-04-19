import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import sys
import math

class Bug2Controller(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('bug2_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.current_x = 0.0
        self.current_y = 0.0
        self.obstacle_detected = False
        self.following_obstacle = False
        self.mline_distance = self.distance_to_goal()
        self.rate = self.create_rate(10)

    def distance_to_goal(self):
        return math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

    def is_at_goal(self):
        return self.distance_to_goal() < 0.2  # หยุดเมื่อใกล้เป้าหมาย

    def can_move_straight_to_goal(self, msg):
        """ ตรวจสอบว่าสามารถเดินตรงไปยังเป้าหมายได้หรือไม่ """
        front_distance = msg.ranges[len(msg.ranges) // 2]
        return front_distance > 0.5

    def lidar_callback(self, msg):
        self.obstacle_detected = not self.can_move_straight_to_goal(msg)
        self.get_logger().info(f"🔍 LiDAR detected obstacle: {self.obstacle_detected}")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.get_logger().info(f"📍 Odom: x={self.current_x}, y={self.current_y}")

    def move_to_goal(self):
        """ เคลื่อนที่ไปยังเป้าหมายโดยตรง """
        twist = Twist()
        while rclpy.ok() and not self.is_at_goal():
            rclpy.spin_once(self, timeout_sec=0.1)  # อัปเดตข้อมูลใหม่

            if not self.obstacle_detected:
                self.get_logger().info("➡️ ไม่มีสิ่งกีดขวาง เคลื่อนที่ตรงไปยังเป้าหมาย")
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            else:
                self.get_logger().info("⚠️ พบสิ่งกีดขวาง เริ่มติดตามขอบ")
                self.follow_obstacle()
                continue  

            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

    def follow_obstacle(self):
        """ ติดตามขอบสิ่งกีดขวาง """
        twist = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if not self.obstacle_detected:
                current_distance = self.distance_to_goal()
                if current_distance < self.mline_distance:
                    self.get_logger().info("🔄 กลับไปที่เส้นตรงไปยังเป้าหมาย")
                    self.mline_distance = current_distance
                    return  

            twist.linear.x = 0.1
            twist.angular.z = 0.3
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

    def run(self):
        self.get_logger().info("🚀 เริ่มการนำทาง Bug2")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.move_to_goal()
        self.get_logger().info("🛑 สิ้นสุดการทำงาน")

def main():
    rclpy.init()
    if len(sys.argv) != 3:
        print("Usage: ros2 run bug2_navigation bug2_controller <goal_x> <goal_y>")
        return
    
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    
    node = Bug2Controller(goal_x, goal_y)
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
