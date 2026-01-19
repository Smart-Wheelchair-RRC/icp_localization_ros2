import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import TimerAction, Shutdown, DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue

sys.path.append(os.path.join(get_package_share_directory('icp_localization_ros2'), 'launch'))


def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter

    node_params = os.path.join(
        get_package_share_directory('icp_localization_ros2'), 'config', 'node_params.yaml')

    input_filters_config_path = os.path.join(
        get_package_share_directory('icp_localization_ros2'), 'config', 'input_filters_velodyne_puck.yaml')
    
    icp_config_path = os.path.join(
        get_package_share_directory('icp_localization_ros2'), 'config', 'icp.yaml')

    # Launch argument for rate
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='ICPlocalization loop rate in Hz'
    )

    icp_node = Node(
        package='icp_localization_ros2',
        executable='icp_localization',
        output='screen',
        parameters=[
            node_params,
            {
                'icp_config_path': icp_config_path,
                'input_filters_config_path': input_filters_config_path,
                # Pass the rate as a typed parameter
                'icp_localization_ros2.rate': ParameterValue(LaunchConfiguration('rate'), value_type=float),
            }
        ],
    )
    
    delay_node = TimerAction(
        period=1.5,
        actions=[icp_node],
    )
    
    return LaunchDescription([
        rate_arg,
        delay_node, 
    ])
