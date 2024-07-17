import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 定位到功能包的地址
    package_name = 'sensor_pub'

    euroc_config_path = os.path.join(
        get_package_share_directory(package_name), 'config', 'euroc_config.yaml'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name), 'config', 'rviz_vins_config.rviz'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ld = LaunchDescription()

    euroc_pub_node = Node(
        package=package_name,
        executable="euroc_pub_node",
        name="euroc_pub_node",
        parameters=[euroc_config_path],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ld.add_action(rviz_node)
    ld.add_action(euroc_pub_node)

    return ld
