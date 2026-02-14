from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    set_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            )
        )
    )

    random_obstacles = Node(
        package='windrobo_sim',
        executable='random_obstacles',
        output='screen'
    )

    reactive_controller = Node(
        package='windrobo_cpp',
        executable='goal_subscriber',
        output='screen'
    )


    return LaunchDescription([
        set_model,
        gazebo,
        random_obstacles,
        reactive_controller,
    ])
