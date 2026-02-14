import rclpy
from rclpy.node import Node
from windrobo_msgs.msg import Goal
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math


class WaypointManager(Node):

    def __init__(self):
        super().__init__('waypoint_manager')

        # =========================
        # Parámetros ajustables
        # =========================
        self.tolerance = 0.25  # ligeramente mayor para evitar quedarse corto

        # =========================
        # Subscripciones
        # =========================
        self.goal_sub = self.create_subscription(
            Goal,
            'final_goal',
            self.goal_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.env_sub = self.create_subscription(
            Bool,
            'environment_ready',
            self.env_callback,
            10
        )

        # =========================
        # Publicador
        # =========================
        self.publisher_ = self.create_publisher(
            Goal,
            'current_goal',
            10
        )

        # =========================
        # Variables internas
        # =========================
        self.waypoints = []
        self.current_index = 0

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.goal_received = False
        self.environment_ready = False
        self.completed = False

        # Timer
        self.timer = self.create_timer(
            0.1,
            self.publish_current_waypoint
        )

        self.get_logger().info("Waypoint Manager started.")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def goal_callback(self, msg):

        # Si ya tenemos un objetivo activo, ignoramos duplicados
        if self.goal_received and not self.completed:
            self.get_logger().warn("Goal already active. Ignoring new goal.")
            return

        final_x = msg.goal_x
        final_y = msg.goal_y

        self.waypoints = [
            (final_x * 0.33, final_y * 0.33),
            (final_x * 0.66, final_y * 0.66),
            (final_x, final_y)
        ]

        self.current_index = 0
        self.goal_received = True
        self.completed = False

        self.get_logger().info(
            f"Waypoints generated: {self.waypoints}"
        )

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def env_callback(self, msg):
        self.environment_ready = msg.data

    # ============================================================
    # CONTROL LOOP
    # ============================================================

    def publish_current_waypoint(self):

        if not self.goal_received:
            return

        if not self.environment_ready:
            return

        if self.completed:
            return

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed.")
            self.completed = True
            return

        target_x, target_y = self.waypoints[self.current_index]

        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(
            f"WP {self.current_index} | "
            f"Target: ({target_x:.2f}, {target_y:.2f}) | "
            f"Distance: {distance:.3f}",
            throttle_duration_sec=2.0
        )

        # ====================================================
        # Si alcanzó el waypoint
        # ====================================================
        if distance < self.tolerance:

            self.get_logger().info(
                f"Waypoint {self.current_index} reached "
                f"(distance: {distance:.3f})"
            )

            self.current_index += 1

            # Si aún quedan, publicar inmediatamente el siguiente
            if self.current_index < len(self.waypoints):
                next_x, next_y = self.waypoints[self.current_index]

                msg = Goal()
                msg.goal_x = next_x
                msg.goal_y = next_y
                self.publisher_.publish(msg)

                self.get_logger().info(
                    f"Switching to waypoint {self.current_index}: "
                    f"({next_x:.2f}, {next_y:.2f})"
                )

            return

        # ====================================================
        # Publicación normal
        # ====================================================
        msg = Goal()
        msg.goal_x = target_x
        msg.goal_y = target_y
        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Publishing waypoint {self.current_index}",
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
