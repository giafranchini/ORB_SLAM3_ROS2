from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    log_level = 'INFO' # 'INFO', 'DEBUG', 'WARN', 'ERROR', 'FATAL'
    log_level_args = ['--ros-args', '--log-level', log_level, '--log-level', 'rcl:=INFO', '--log-level', 'rclcpp:=INFO', '--log-level', 'rmw_fastrtps_cpp:=INFO']

    config = os.path.join(get_package_share_directory('orbslam3'), 'config/monocular', 'optris_pi640_uma.yaml')
    vocabulary = os.path.join(get_package_share_directory('orbslam3'), 'vocabulary', 'rec_diablo_1_thermal.txt')

    print(config)

    mono_node = Node(
        package='orbslam3',
        executable='mono',
        name='mono',
        output='screen',
        namespace='spaceuma',
        arguments=[vocabulary, config],
        remappings=[
          ('/spaceuma/camera', '/spaceuma/thermal_camera_node/normalized'),
        ]
    )
    return LaunchDescription([mono_node])