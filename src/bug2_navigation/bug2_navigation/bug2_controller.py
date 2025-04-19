import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import sys
import time

class Bug2Controller(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('bug2_controller')

        # Publisher สำหรับควบคุมการเคลื่อนที่
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # QoS Profile สำหรับ Sensor Data
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # เป้าหมาย
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.current_x = 0.0
        self.current_y = 0.0
        self.obstacle_detected = False
        self.following_obstacle = False
        self.mline_distance = float('inf')  # เริ่มต้นให้ห่างไกล

        # ตัวแปรสำหรับ loop
        self.running = True

    def distance_to_goal(self):
        """ คำนวณระยะทางไปยังเป้าหมาย """
        return math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

    def is_at_goal(self):
        """ ตรวจสอบว่าถึงเป้าหมายหรือยัง """
        return self.distance_to_goal() < 0.2  

    def lidar_callback(self, msg):
        """ อ่านค่าจาก LiDAR และตรวจจับสิ่งกีดขวาง """
        front_distance = msg.ranges[len(msg.ranges) // 2]
        self.obstacle_detected = front_distance < 0.5
        self.get_logger().info(f"🔍 LiDAR: Obstacle={self.obstacle_detected}")

    def odom_callback(self, msg):
        """ อ่านค่าตำแหน่งจาก Odometry """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.get_logger().info(f"📍 Odom: x={self.current_x}, y={self.current_y}")

    def move_to_goal(self):
        """ เคลื่อนที่ไปยังเป้าหมายตาม Bug2 Algorithm """
        twist = Twist()
        while rclpy.ok() and self.running:
            if self.is_at_goal():
                self.get_logger().info("🏁 ถึงเป้าหมายแล้ว!")
                self.stop_robot()
                break

            if not self.obstacle_detected:
                self.get_logger().info("➡️ เคลื่อนที่ตรงไปยังเป้าหมาย")
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            else:
                self.get_logger().info("⚠️ พบสิ่งกีดขวาง ติดตามขอบ")
                self.follow_obstacle()
                continue  

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # ป้องกัน loop เร็วเกินไป

    def follow_obstacle(self):
        """ ติดตามขอบสิ่งกีดขวาง """
        twist = Twist()
        while rclpy.ok() and self.running:
            if not self.obstacle_detected:
                current_distance = self.distance_to_goal()
                if current_distance < self.mline_distance:
                    self.get_logger().info("🔄 กลับไปที่เส้นตรงไปยังเป้าหมาย")
                    self.mline_distance = current_distance
                    return  

            twist.linear.x = 0.1
            twist.angular.z = 0.3
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

    def stop_robot(self):
        """ หยุดหุ่นยนต์และออกจากโปรแกรม """
        self.get_logger().info("🛑 หยุดหุ่นยนต์และออกจากโปรแกรม")
        self.running = False
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def run(self):
        """ เริ่มต้นการทำงานของหุ่นยนต์ """
        self.get_logger().info("🚀 เริ่มการนำทาง Bug2")

        executor = MultiThreadedExecutor()
        executor.add_node(self)

        try:
            while rclpy.ok() and self.running:
                self.move_to_goal()
                time.sleep(0.1)

        except KeyboardInterrupt:
            self.get_logger().info("⌨️ กด Ctrl+C หยุดหุ่นยนต์")
            self.stop_robot()
        
        finally:
            executor.shutdown()
            self.destroy_node()

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
