from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the launch directory of the package
    robot_launch_dir = os.path.join(get_package_share_directory('tarkbot_robot'), 'launch')
    lidar_launch_dir = os.path.join(get_package_share_directory('m1ct_d2'), 'launch')
    rosbridge_server_dir = os.path.join(get_package_share_directory('rosbridge_server'), 'launch')

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_launch_dir, 'robot.launch.py')
            )
        ),

        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_launch_dir, 'd2.launch.py')
            )
        ),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(rosbridge_server_dir, 'rosbridge_websocket_launch.xml')
            )
        ),

        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        )
    ])