from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 1️⃣ Gazebo world 
    #################### Adjust these file directories as per your turtlebot3 installation directory #########################
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    tb3_world_launch = os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_world_launch)
    )
    # 2️⃣ Map server
    tb3_nav2_pkg = get_package_share_directory('turtlebot3_navigation2')
    map_file = os.path.join(tb3_nav2_pkg, 'map', 'map.yaml')
##############################################################################################################################

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file
        }],
    )

    # 3️⃣ AMCL (localization)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'scan_topic': 'scan'
        }],
    )

    # 4️⃣ Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
    )

    # 5️⃣ RViz Visualization
    rviz_config = os.path.join(tb3_nav2_pkg, 'rviz', 'tb3_navigation2.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # 6️⃣ A* Global Planner
    a_star_node = Node(
        package='path_planner',
        executable='a_star_node',
        name='a_star_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 7️⃣ Path Smoother (Savitzky-Golay)
    smoother_node = Node(
        package='path_smoother',
        executable='savitzky_golay_smoother',
        name='savitzky_golay_smoother',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/input_path', '/a_star/path')],
    )

    # 8️⃣ Trajectory Generator (Time Parameterizer)
    trajectory_node = Node(
        package='trajectory_generator',
        executable='trajectory',
        name='time_parameterizer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/smoothed_path', '/smoothed_path')],
    )

    # 9️⃣ PD Motion Controller (robot controller)
    controller_node = Node(
        package='robot_controller',
        executable='controller',
        name='pd_motion_planner_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/time_trajectory', '/time_trajectory')],
    )

    # Combine all components
    return LaunchDescription([
        gazebo_launch,   
        map_server,
        amcl_node,
        lifecycle_manager,
        rviz_node,
        a_star_node,
        smoother_node,
        trajectory_node,
        controller_node,
    ])
