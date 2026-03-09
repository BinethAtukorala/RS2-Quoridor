import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class GameState(Node):

    def __init__(self):
        super().__init__('game_state')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    game_state = GameState()

    rclpy.spin(game_state)

    game_state.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()