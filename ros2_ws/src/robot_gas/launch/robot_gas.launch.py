from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose'],
        output='screen'
    )

    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'set', '/gazebo', 'use_sim_time', 'true'
        ],
        shell=True
    )

    sensor_node = Node(
        package='robot_gas',
        executable='gas_sensor.py',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        sensor_node
    ])
