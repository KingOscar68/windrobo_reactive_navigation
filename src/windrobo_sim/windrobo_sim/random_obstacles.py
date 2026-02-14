import rclpy
from rclpy.node import Node
import random
import subprocess

from windrobo_msgs.msg import Goal
from std_msgs.msg import Bool


class RandomObstacles(Node):

    def __init__(self):
        super().__init__('random_obstacles')

        self.subscription = self.create_subscription(
            Goal,
            'final_goal',
            self.goal_callback,
            10
        )

        self.ready_pub = self.create_publisher(
            Bool,
            'environment_ready',
            10
        )

        self.obstacles_spawned = False
        self.num_obstacles = 8
        self.radius = 0.15

        self.get_logger().info("Waiting for FINAL goal...")

    def goal_callback(self, msg):

        if self.obstacles_spawned:
            return

        goal_x = msg.goal_x
        goal_y = msg.goal_y

        self.get_logger().info(
            f"Generating obstacles for goal ({goal_x}, {goal_y})"
        )

        for i in range(self.num_obstacles):

            x = random.uniform(0.3, goal_x - 0.3)
            y = random.uniform(0.3, goal_y - 0.3)

            self.spawn_sphere(i, x, y, self.radius)

        self.obstacles_spawned = True

        # Publicar que el entorno está listo
        ready_msg = Bool()
        ready_msg.data = True
        self.ready_pub.publish(ready_msg)

        self.get_logger().info("Environment ready.")

    def spawn_sphere(self, idx, x, y, radius):

        sdf = f"""
        <sdf version='1.6'>
          <model name='obstacle_{idx}'>
            <static>true</static>
            <link name='link'>
              <collision name='collision'>
                <geometry>
                  <sphere>
                    <radius>{radius}</radius>
                  </sphere>
                </geometry>
              </collision>
              <visual name='visual'>
                <geometry>
                  <sphere>
                    <radius>{radius}</radius>
                  </sphere>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """

        file_path = f"/tmp/obstacle_{idx}.sdf"

        with open(file_path, "w") as f:
            f.write(sdf)

        subprocess.run([
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", f"obstacle_{idx}",
            "-file", file_path,
            "-x", str(x),
            "-y", str(y),
            "-z", str(radius)
        ])


def main(args=None):
    rclpy.init(args=args)
    node = RandomObstacles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
