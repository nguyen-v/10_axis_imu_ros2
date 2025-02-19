import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    port_name_arg = DeclareLaunchArgument(
        'port_name', default_value='/dev/ttyUSB0',
        description='Choose which port to use'
    )

    # Define the nodes to be launched
    imu_node = Node(
        package='imu',
        executable='imu_node',
        parameters=[{'port_name': LaunchConfiguration('port_name')}]
    )

    imu_view_file_path = os.path.join(get_package_share_directory('imu'), 'rviz', 'imu_view.rviz')

    # Define the RViz2 node to launch RViz if enabled
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', imu_view_file_path]
    )

    # Return the launch description with all defined actions
    return LaunchDescription([
        port_name_arg,
        imu_node,
        rviz_node
    ])