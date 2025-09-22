import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class TurtlePatrolServer(Node):
    def __init__(self, turtle_name: str):
        super().__init__(f'{turtle_name}_patrol_server')
        self._turtle = turtle_name

        self._cmd_topic = f'/{self._turtle}/cmd_vel'
        self._service_name = f'/{self._turtle}/patrol'
        self._teleport_service_name = f'/{self._turtle}/teleport_absolute'

        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._srv = self.create_service(Patrol, self._service_name, self.patrol_callback)

        self._lin = 0.0
        self._ang = 0.0
        self._pub_timer = self.create_timer(0.1, self._publish_current_cmd)

        self.get_logger().info(
            f'Patrol server ready for {self._turtle}. Service: {self._service_name} publishing {self._cmd_topic}'
        )

    # -------------------------------------------------------
    # Timer publishes current Twist
    # -------------------------------------------------------
    def _publish_current_cmd(self):
        msg = Twist()
        msg.linear.x = self._lin
        msg.angular.z = self._ang
        self._cmd_pub.publish(msg)

    # -------------------------------------------------------
    # Service callback: update speeds
    # -------------------------------------------------------
    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        self.get_logger().info(
            f"Patrol request: vel={request.vel:.2f}, omega={request.omega:.2f}, x={request.x:.2f}, y={request.y:.2f}, theta={request.theta:.2f}"
        )

        # Optional teleport if x,y > 0 (简单约定：<=0 忽略)
        if request.x > 0.0 and request.y > 0.0:
            teleport_cli = self.create_client(TeleportAbsolute, self._teleport_service_name)
            if not teleport_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Teleport service not available, skip teleport.')
            else:
                tele_req = TeleportAbsolute.Request()
                tele_req.x = request.x
                tele_req.y = request.y
                tele_req.theta = request.theta
                future = teleport_cli.call_async(tele_req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.done() and future.result() is not None:
                    self.get_logger().info('Teleport success.')
                else:
                    self.get_logger().warn('Teleport may have failed.')

        # Update speeds
        self._lin = float(request.vel)
        self._ang = float(request.omega)

        cmd = Twist()
        cmd.linear.x = self._lin
        cmd.angular.z = self._ang
        response.cmd = cmd

        self.get_logger().info(
            f"Streaming cmd_vel: lin.x={self._lin:.2f}, ang.z={self._ang:.2f} (10 Hz)"
        )
        return response


def main(args=None):
    # Initialize rclpy first; use sys.argv for positional arguments because rclpy.init
    # will strip ROS remapping args from sys.argv but still leaves our custom ones.
    rclpy.init(args=args)
    argv = sys.argv
    turtle_name = 'turtle1'
    if len(argv) > 1:
        turtle_name = argv[1]
    # Log chosen turtle for clarity
    print(f'[patrol_server] Using turtle: {turtle_name}')
    node = TurtlePatrolServer(turtle_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
