# ros2 launch pipibot_description display.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    ld = LaunchDescription()

    urdf_launch_package = FindPackageShare('pipibot_description')
    default_model_path = PathJoinSubstitution(['urdf', 'pipibot.urdf'])
    default_rviz_config_path = PathJoinSubstitution([urdf_launch_package, 'rviz', 'urdf.rviz'])

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))


    ld.add_action(DeclareLaunchArgument(name='jsp_gui', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui'))

    ld.add_action(DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([urdf_launch_package, 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': "pipibot_description",
            'urdf_package_path':LaunchConfiguration('model')}.items()
    
    ))

    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))

    return ld