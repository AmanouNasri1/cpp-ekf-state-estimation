import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    sdf_file = os.path.join(
        get_package_share_directory('ekf_project'), 'models', 'turtlebot3_burger.sdf'
    )

    # 1. IGNITION GAZEBO (Starts the simulation and loads the world/robot)
    ignition_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items(),
    )

    # 2. BRIDGE
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Generic sensors
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            
            # Odometry (From Gazebo DiffDrive plugin)
            '/model/turtlebot3_burger/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            
            # TF (Crucial for connecting Odom -> Base)
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        ],
        remappings=[
            # This renames the complex Gazebo topic to your desired ROS topic
            ('/model/turtlebot3_burger/odometry', '/odom_gt'),
        ],
        output='screen'
    )

    # 3. STATIC TRANSFORMS
    # These connect the Gazebo frames to standard ROS frames
    
    # FIX: Connect Gazebo's 'base' frame to ROS standard 'base_link'
    # The DiffDrive plugin publishes 'odom' -> 'base'. We need 'base' -> 'base_link'.
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf',
        arguments=['0','0','0','0','0','0', 'base', 'base_link']
    )

    # Lidar TF (Matches SDF Position: -0.032 0 0.171)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=[
            '--x', '-0.032', '--y', '0', '--z', '0.171',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar' 
        ]
    )

    # Camera TF (Matches SDF Position: 0.05 0 0.12)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            '--x', '0.05', '--y', '0', '--z', '0.12',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )

    # IMU TF (Matches SDF Position: 0 0 0.07 inside base)
    # Usually IMU is at center of base_link for these robots
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.07',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ]
    )

    # 4. RVIZ2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        ignition_spawn,
        bridge,
        base_link_tf,
        lidar_tf,
        camera_tf,
        imu_tf,
        TimerAction(period=3.0, actions=[rviz_node])
    ])