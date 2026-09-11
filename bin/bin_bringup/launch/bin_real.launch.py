from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    description_pkg = get_package_share_directory('bin_description')
    bringup_pkg = get_package_share_directory('bin_bringup')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    nav2_launch_pkg = get_package_share_directory('nav2_bringup')
    rviz_config_file = os.path.join(description_pkg, 'rviz', 'slam.rviz')

    # Stable names from bin_bringup/udev/99-bin-robot.rules, not /dev/ttyUSB0 and
    # /dev/ttyUSB1. Those numbers are assigned in enumeration order, so if the Nano
    # resets and re-enumerates the two devices swap and the motor driver starts
    # talking to the laser. Run bin_bringup/udev/setup_udev.sh once on the robot.
    # Override with motor_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1 if needed.
    declare_motor_port = DeclareLaunchArgument(
        'motor_port', default_value='/dev/bin_motors',
        description='Serial port for the Arduino motor controller')
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/bin_lidar',
        description='Serial port for the LD19 laser')
    # Named use_camera, not camera: camera_node takes a parameter also called
    # 'camera' (the libcamera device index) and the two are unrelated.
    declare_use_camera = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Start the CSI camera (camera_ros). false saves Pi CPU and WiFi')

    robot_description_config = ParameterValue(
        Command([
            'xacro ',
            os.path.join(description_pkg, 'urdf', 'bin_robot.urdf.xacro'),
            ' use_gazebo:=false',
            ' motor_port:=', LaunchConfiguration('motor_port'),
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False
        }]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
        {'robot_description': robot_description_config},
        os.path.join(bringup_pkg, 'config', 'bin_controllers.yaml'),
        {'use_sim_time': False}
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': False}]
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        parameters=[{'use_sim_time': False}]
    )

    bin_cover_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bin_cover_controller'],
        parameters=[{'use_sim_time': False}]
    )

    # The LD19 rides on the lid, so its scan plane tilts when the lid opens. Gate the
    # raw feed on bin_cover_joint being closed and settled; everything downstream
    # (slam_toolbox, both Nav2 costmaps, collision_monitor, slam.rviz) reads /lidar.
    scan_gate = Node(
        package='bin_perception',
        executable='scan_gate',
        name='scan_gate',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'input_topic': '/lidar_raw',
            'output_topic': '/lidar',
            'joint_name': 'bin_cover_joint',
            'closed_tolerance': 0.05,
            'settle_time': 0.5,
        }]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'frame_id': 'base_footprint'
                     }],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ]
    )

    # ── Camera ────────────────────────────────────────────────────────────
    # camera_ros drives the Pi CSI module through libcamera. Topics and frame are
    # remapped onto the names the sim's gz bridge already produces, so color_detector
    # and yolo_detector work unchanged against either.
    #
    # Both image topics need explicit remaps: camera_ros creates ~/image_raw/compressed
    # with its own create_publisher rather than through image_transport, so remapping
    # the base topic alone would leave the compressed one behind on /camera/.
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        parameters=[{
            'use_sim_time': False,
            'camera': 0,                             # libcamera index, or sensor name string
            'width': 640,                            # matches the sim's 640x480
            'height': 480,
            'format': 'RGB888',                      # else auto-selects XRGB8888: a wasted
                                                     # alpha byte per pixel over WiFi
            'role': 'video',
            'frame_id': 'camera_rgb_optical_frame',  # must match bin_camera.xacro and gz_frame_id
            'jpeg_quality': 75,                      # 95 default is heavy over WiFi
        }],
        remappings=[
            ('~/image_raw', '/camera/rgb/image_raw'),
            ('~/image_raw/compressed', '/camera/rgb/image_raw/compressed'),
            ('~/camera_info', '/camera/rgb/camera_info'),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    #____LiDAR________________________________________________
    ldlidar_node = Node( 
        package='ldlidar_stl_ros2', 
        executable='ldlidar_stl_ros2_node', 
        name='ldlidar_node', 
        output='screen', 
        parameters=[{ 
            'product_name':           'LDLiDAR_LD19',
            # Raw feed — scan_gate republishes this as /lidar only while the lid is
            # closed, because the LD19 is mounted on the lid and pitches with it.
            'topic_name':             'lidar_raw',
            'frame_id':               'lidar_link',
            'port_name':              LaunchConfiguration('lidar_port'),
            'port_baudrate':          230400,
            'laser_scan_dir':         True,
            'enable_angle_crop_func': False,
            'angle_crop_min':         0.0,
            'angle_crop_max':         0.0
        }] ) 

    
        # ── binny bridge ─────────────────────────────────────────────────────
    # Serves the rosbridge-style websocket that binny (@foxglove/ws-protocol)
    # connects to. Port 8765 is foxglove_bridge's default and is what
    # NEXT_PUBLIC_FOXGLOVE_WS_URL should point at.
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'use_sim_time': False,
        }]
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                bringup_pkg, 'config', 'mapper_params_online_async.yaml'
            ),
            'use_sim_time': 'false',
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(bringup_pkg, 'config', 'nav2_params.yaml'),
        }.items()
    )

    return LaunchDescription([
        declare_motor_port,
        declare_lidar_port,
        declare_use_camera,
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        bin_cover_controller_spawner,
        twist_stamper,
        ldlidar_node,
        foxglove_bridge_node,
        # Mapping and RViz run on the laptop. The robot only publishes hardware,
        # odometry, joint states, TF, and the gated laser scan.
        scan_gate,
        camera_node,
        # rviz_node
    ])