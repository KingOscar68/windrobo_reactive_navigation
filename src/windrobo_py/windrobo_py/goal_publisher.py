import rclpy
from rclpy.node import Node
from windrobo_msgs.msg import Goal


class FinalGoalPublisher(Node):

    def __init__(self):
        super().__init__('final_goal_publisher')

        self.publisher_ = self.create_publisher(Goal, 'final_goal', 10)

        try:
            x = float(input("Enter FINAL goal X: "))
            y = float(input("Enter FINAL goal Y: "))
        except ValueError:
            self.get_logger().error("Invalid input.")
            rclpy.shutdown()
            return

        msg = Goal()
        msg.goal_x = x
        msg.goal_y = y

        self.publisher_.publish(msg)

        self.get_logger().info(f"Published FINAL goal: ({x}, {y})")

        # publicar por 1 segundo para asegurar recepción
        self.timer = self.create_timer(0.2, lambda: self.publisher_.publish(msg))


def main(args=None):
    rclpy.init(args=args)
    node = FinalGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
