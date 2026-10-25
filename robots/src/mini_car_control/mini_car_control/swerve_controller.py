#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist        # /cmd_vel 的类型
from std_msgs.msg import Float64           # 关节话题的类型

  # 轮子位置（和 URDF 里的 hub_x/hub_y 保持一致！）
WHEELS = {
      'FL': (+0.30, +0.30),
      'FR': (+0.30, -0.30),
      'RL': (-0.30, +0.30),
      'RR': (-0.30, -0.30),
  }
WHEEL_RADIUS = 0.05

class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')
          # ① 建 8 个发布者（4 转向 + 4 驱动）
        self.steer_pubs = {}
        self.wheel_pubs = {}
        self.last_cmd_time = self.get_clock().now()          # ① 记下"上次收到命令的时间"
        self.create_timer(0.1, self.watchdog)                #    每 0.1 秒检查一次
        for tag in WHEELS:                      # 遍历轮子字典，自动生成 8 个
            self.steer_pubs[tag] = self.create_publisher(
                Float64, f'/mini_car/steer_{tag}/cmd_pos', 10)
            self.wheel_pubs[tag] = self.create_publisher(
                Float64, f'/mini_car/wheel_{tag}/cmd_vel', 10)

          # ② 订阅 /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

    def on_cmd_vel(self, msg):
        self.last_cmd_time = self.get_clock().now()          # ② 每收到命令就刷新时间
        self.drive(msg.linear.x, msg.linear.y, msg.angular.z)
    
    def drive(self, vx, vy, w):                              # ③ 【新】把发送逻辑抽出来
          for tag, (x, y) in WHEELS.items():
            v_ix = vx + (-w * y)
            v_iy = vy + ( w * x)
            theta = math.atan2(v_iy, v_ix)
            speed = math.hypot(v_ix, v_iy) / WHEEL_RADIUS
            self.steer_pubs[tag].publish(Float64(data=theta))
            self.wheel_pubs[tag].publish(Float64(data=speed))

    def watchdog(self):
          dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
          if dt > 0.5:                                         # 超过 0.5 秒没收到命令
              self.drive(0.0, 0.0, 0.0)                        #    主动停车

def main():
      rclpy.init()
      node = SwerveController()
      rclpy.spin(node)          # 一直跑

if __name__ == '__main__':
    main()