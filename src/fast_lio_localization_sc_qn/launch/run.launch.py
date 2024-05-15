import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio_localization_sc_qn')
    default_config_path = os.path.join(package_path, 'config', 'config.yaml')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'localization_rviz.rviz')
    default_map_path = os.path.join(package_path, 'maps', 'result_bag')

    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz_use')
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_rviz_use_cmd = DeclareLaunchArgument(
        'rviz_use', default_value='true',
        description='Use RViz to monitor results'
    )

    fast_lio_sam_node = Node(
        package='fast_lio_localization_sc_qn',
        executable='fast_lio_localization_sc_qn_node',
        name='fast_lio_localization_sc_qn_node',
        output='screen',
        parameters=[config_path,
                    { 'basic.saved_map': default_map_path},],

    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_use_cmd)
    ld.add_action(fast_lio_sam_node)
    ld.add_action(rviz_node)
    
    return ld
