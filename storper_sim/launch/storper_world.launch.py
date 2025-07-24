from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = '/root/ros2_ws/src/storper_sim/urdf/storper.urdf'
    world_file = '/root/ros2_ws/install/share/storper_sim/worlds/flat.world'

    generate_sdf = ExecuteProcess(
        cmd=[
            'bash', '-c',
            #f'xacro {urdf_file} > /tmp/storper.urdf && gz sdf -p /tmp/storper.urdf > /tmp/storper.sdf'
            f'gz sdf -p {urdf_file} > /tmp/storper.sdf'
        ],
        output='screen'
    )

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'storper',
            '-file', '/tmp/storper.sdf',
            '-x', '0', '-y', '0', '-z', '0.05'
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}],
        output='screen'
    )

    return LaunchDescription([
        generate_sdf,
        gz_server,
        spawn_entity,
        robot_state_publisher
    ])

