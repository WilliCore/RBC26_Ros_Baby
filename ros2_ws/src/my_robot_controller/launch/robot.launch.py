import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # --- HARDWARE CONFIG ---
    lidar_serial_port = '/dev/ttyUSB1' 
    
    # 1. RPLIDAR Driver (Input: Real World -> Output: /scan)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
        ]),
        launch_arguments={
            'serial_port': lidar_serial_port, 
            'serial_baudrate': '115200', 
            'inverted': 'false' # Set to 'false' because we fixed the double-flip issue
        }.items()
    )

    # 2. LASER FILTER (Input: /scan -> Output: /scan_filtered)
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[
            os.path.join(get_package_share_directory('my_robot_controller'), 'config', 'laser_filter.yaml')
        ],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ]
    )

    # 3. TF Transform (Links Lidar to Robot Base)
    # X=0, Y=0, Z=-0.05, Yaw=0, Pitch=0, Roll=3.14 (Upside Down)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=['0', '0', '-0.05', '0', '0', '3.14159', 'base_link', 'laser']
    )

    # 4. Odometry Node
    odom_node = Node(
        package='my_robot_controller',
        executable='odometry',
        name='odometry_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # --- MISSION & APPS ---
    
    tracker_node = Node(
        package='my_robot_controller',
        executable='tracker',
        name='object_tracker'
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090, 'delay_between_messages': 0.0}]
    )

    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server'
    )

    mission_node = Node(
        package='my_robot_controller',
        executable='mission',
        name='mission_controller'
    )

    pkg_name = 'my_robot_controller'
    file_subpath = 'description/urdf/my_robot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': False}] # Set True if using Gazebo
    )

    # Notice: NO SLAM TOOLBOX HERE!
    return LaunchDescription([
        lidar_launch,
        laser_filter_node,
        lidar_tf,
        odom_node,
        tracker_node,
        rosbridge_node,
        web_video,
        mission_node,
        node_robot_state_publisher
    ])