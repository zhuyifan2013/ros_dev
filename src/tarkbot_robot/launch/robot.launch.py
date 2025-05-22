from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明启动参数
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link'
    )
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom'
    )
    imu_frame_arg = DeclareLaunchArgument(
        'imu_frame',
        default_value='imu_link'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='odom'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='imu'
    )
    battery_topic_arg = DeclareLaunchArgument(
        'battery_topic',
        default_value='bat_vol'
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel'
    )
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='r20_fwd'
    )
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf',
        default_value='true'
    )

    # 启动tarkbot_robot节点
    tarkbot_robot_node = Node(
        package='tarkbot_robot',
        executable='tarkbot_robot_pipi',
        name='tarkbot_robot',
        output='screen',
        respawn=False,
        parameters=[
            {
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'imu_frame': LaunchConfiguration('imu_frame'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'battery_topic': LaunchConfiguration('battery_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'robot_port': '/dev/tarkbot_base',
                'robot_port_baud': 230400,
                'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
                'robot_type_send': LaunchConfiguration('robot_type')
            }
        ]
    )

    return LaunchDescription([
        base_frame_arg,
        odom_frame_arg,
        imu_frame_arg,
        odom_topic_arg,
        imu_topic_arg,
        battery_topic_arg,
        cmd_vel_topic_arg,
        robot_type_arg,
        pub_odom_tf_arg,
        tarkbot_robot_node
    ])