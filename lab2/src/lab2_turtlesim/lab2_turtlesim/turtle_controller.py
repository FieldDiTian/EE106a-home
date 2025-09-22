#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP_TEXT = """控制指令 (回车发送)：
  w: 前进
  s: 后退
  a: 左转
  d: 右转
  q: 左前(前进+左转)
  e: 右前(前进+右转)
  z: 左后(后退+左转)
  c: 右后(后退+右转)
  space/空行: 停止
  h: 显示帮助
  x: 退出节点
"""

KEY_MAP = {
    'w': (1.5, 0.0),   # (linear, angular)
    's': (-1.5, 0.0),
    'a': (0.0, 2.0),
    'd': (0.0, -2.0),
    'q': (1.0, 2.0),
    'e': (1.0, -2.0),
    'z': (-1.0, 2.0),
    'c': (-1.0, -2.0),
}

class TurtleController(Node):
    def __init__(self, turtle_name: str):
        super().__init__('turtle_controller_' + turtle_name)
        self.turtle_name = turtle_name
        topic = f'/{turtle_name}/cmd_vel'
        self.publisher_ = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(f'控制节点启动，控制乌龟: {turtle_name}, 发布到 {topic}')
        print(HELP_TEXT)

    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        self.get_logger().debug(f'发布 Twist: linear.x={linear:.2f}, angular.z={angular:.2f}')


def main(argv=None):
    if argv is None:
        argv = sys.argv
    rclpy.init(args=argv)

    if len(argv) < 2:
        print('用法: ros2 run lab2_turtlesim turtle_controller <turtle_name>')
        return 1

    turtle_name = argv[1]
    node = TurtleController(turtle_name)

    try:
        while rclpy.ok():
            try:
                key = input(f'[{turtle_name}] 输入指令 (h 查看帮助): ').strip().lower()
            except EOFError:
                break
            if key == '':
                node.publish_cmd(0.0, 0.0)
                continue
            if key == 'x':
                print('退出...')
                break
            if key == 'h':
                print(HELP_TEXT)
                continue
            if key in KEY_MAP:
                lin, ang = KEY_MAP[key]
                node.publish_cmd(lin, ang)
            else:
                print('未知指令，输入 h 查看帮助')
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0

if __name__ == '__main__':
    main()
