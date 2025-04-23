#ros2 launch m1ct_d2 d2.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    SetEnvironmentVariable('RCL_LOG_LEVEL', 'DEBUG'),

    config = os.path.join(
    get_package_share_directory('m1ct_d2'),
    'params',
    'm1ct_d2.yaml'
    )

    # 启动tarkbot_robot节点
    m1ct_d2_node = Node(
        package='m1ct_d2',
        executable='m1ct_d2',
        name='m1ct_d2_node',
        output='screen',
        respawn=False,
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        m1ct_d2_node
    ])