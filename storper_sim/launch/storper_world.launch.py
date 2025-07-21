from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('storper_sim'),
        'worlds',
        'flat.world'
    )

    urdf_path = os.path.join(
        get_package_share_directory('storper_sim'),
        'urdf',
        'storper.urdf'
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                '--headless',
                '--verbose',
                '-s',  # auto-start
                world_path
            ],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    ])

