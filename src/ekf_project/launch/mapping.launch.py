import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_ekf_project = get_package_share_directory('ekf_project')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # 1. Start the Simulation (Robot + Bridge + Rviz)
    # We reuse your existing simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ekf_project, 'launch', 'simulation.launch.py')
        )
    )

    # 2. SLAM Toolbox (The Mapping Node)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(pkg_slam_toolbox, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    # ... inside generate_launch_description ...
    
    # 3. Odom to TF Patch
    # We point directly to the file in your src folder
    script_path = os.path.join(
        os.getenv('HOME'), 
        'ros2_ws/src/ekf_project/scripts/odom_to_tf.py'
    )

    odom_tf_node = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen'
    )

    return LaunchDescription([
        simulation_launch,
        slam_toolbox,
        odom_tf_node  # <--- Add this!
    ])