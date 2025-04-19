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
        """ คำนวณระยะทางไปยังเป้าหมาย """
        return math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

    def is_at_goal(self):
        """ ตรวจสอบว่าหุ่นยนต์ถึงจุดหมายหรือยัง """
        return self.distance_to_goal() < 0.2  # ปรับค่า tolerance ตามต้องการ

    def can_move_straight_to_goal(self, msg):
        """ ตรวจสอบว่าเส้นทางไปยังเป้าหมายโล่งหรือไม่ """
        front_distance = msg.ranges[len(msg.ranges) // 2]
        return front_distance > 0.5  # ไม่มีสิ่งกีดขวางข้างหน้า

    def lidar_callback(self, msg):
        """ อ่านค่าจาก LiDAR และตรวจสอบสิ่งกีดขวาง """
        self.obstacle_detected = not self.can_move_straight_to_goal(msg)

    def odom_callback(self, msg):
        """ อ่านค่าตำแหน่งปัจจุบัน """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def run(self):
        """ ลูปหลักสำหรับการนำทาง """
        twist = Twist()
        
        while rclpy.ok():
            rclpy.spin_once(self)  # อัปเดตข้อมูลจาก LiDAR และ Odom
            
            if self.is_at_goal():
                print("✅ ถึงจุดหมายแล้ว! หยุดหุ่นยนต์")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                break

            if not self.obstacle_detected:
                print("➡️ ไม่มีสิ่งกีดขวาง เคลื่อนที่ตรงไปยังเป้าหมาย")
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            else:
                print("⚠️ พบสิ่งกีดขวาง! เริ่มติดตามขอบ")
                self.follow_obstacle()
                continue  # หลังจากเลี่ยงเสร็จ ให้กลับไปตรวจสอบใหม่

            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

    def follow_obstacle(self):
        """ ติดตามขอบสิ่งกีดขวางและกลับไป M-line เมื่อเป็นไปได้ """
        twist = Twist()
        while rclpy.ok():
            rclpy.spin_once(self)  # อัปเดตข้อมูล
            
            if not self.obstacle_detected:
                current_distance = self.distance_to_goal()
                if current_distance < self.mline_distance:
                    print("🔄 กลับไปที่ M-line และเคลื่อนที่สู่เป้าหมาย")
                    self.mline_distance = current_distance
                    return  # กลับไปเดินตรง

            # ติดตามขอบสิ่งกีดขวาง
            twist.linear.x = 0.1  # เคลื่อนที่ช้าๆ
            twist.angular.z = 0.3  # หมุนเพื่อเลี่ยงสิ่งกีดขวาง
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

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
