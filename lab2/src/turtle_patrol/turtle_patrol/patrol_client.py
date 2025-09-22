import sys
import rclpy
from rclpy.node import Node
from turtle_patrol_interface.srv import Patrol


USAGE = (
    "用法: ros2 run turtle_patrol patrol_client <turtle_name> <vel> <omega> [x y theta]\n"
    "示例1(仅速度): ros2 run turtle_patrol patrol_client turtle1 2.0 1.0\n"
    "示例2(含瞬移): ros2 run turtle_patrol patrol_client turtle2 1.5 0.8 3.0 5.0 1.57\n"
    "说明: 若提供 x y theta 且 x>0 y>0 则服务器尝试瞬移后再设速度"
)


class TurtlePatrolClient(Node):
    def __init__(self, turtle_name: str, vel: float, omega: float, x: float = -1.0, y: float = -1.0, theta: float = 0.0):
        super().__init__(f'{turtle_name}_patrol_client')
        self._service_name = f'/{turtle_name}/patrol'
        self._client = self.create_client(Patrol, self._service_name)

        self.get_logger().info(f"Waiting for service {self._service_name} ...")
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {self._service_name} not available, waiting...")

        self.get_logger().info(
            f"Requesting patrol: vel={vel}, omega={omega}, x={x}, y={y}, theta={theta}"
        )
        req = Patrol.Request()
        req.vel = float(vel)
        req.omega = float(omega)
        # 只有当用户输入了 x y theta 时才赋值(仍然传递给服务端; 约定 x,y <=0 服务端忽略)
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)

        self._future = self._client.call_async(req)


def main(args=None):
    # Let rclpy strip ROS remapping args; use sys.argv for positional user args
    rclpy.init(args=args)
    argv = sys.argv

    # argv: [program, turtle_name, vel, omega, (optional) x y theta]
    if len(argv) < 4:
        print(USAGE)
        rclpy.shutdown()
        return 1

    turtle_name = argv[1]
    vel = float(argv[2])
    omega = float(argv[3])
    x = -1.0
    y = -1.0
    theta = 0.0
    if len(argv) >= 7:
        x = float(argv[4])
        y = float(argv[5])
        theta = float(argv[6])
    node = TurtlePatrolClient(turtle_name, vel, omega, x, y, theta)

    rclpy.spin_until_future_complete(node, node._future)

    if node._future.done():
        result = node._future.result()
        if result is not None:
            cmd = result.cmd
            node.get_logger().info(
                f"Service response Twist: lin.x={cmd.linear.x:.2f}, ang.z={cmd.angular.z:.2f}"
            )
        else:
            node.get_logger().error('Service call failed: no result returned.')
    else:
        node.get_logger().error('Service call did not complete.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


