from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.substitutions import Command    
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    description_pkg = get_package_share_directory('bin_description')
    bringup_pkg = get_package_share_directory('bin_bringup')
    rviz_config_file = os.path.join(description_pkg, 'rviz', 'odom.rviz')



    robot_description_config = ParameterValue(
        Command([
            'xacro ',
            os.path.join(description_pkg, 'urdf', 'bin_robot.urdf.xacro'),
            ' use_gazebo:=false'
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
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{'image_size': [640, 480]}]
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
            'topic_name':             'scan',
            'frame_id':               'lidar_link',
            'port_name':              '/dev/ttyUSB1',
            'port_baudrate':          230400,
            'laser_scan_dir':         True,
            'enable_angle_crop_func': False,
            'angle_crop_min':         0.0,
            'angle_crop_max':         0.0
        }] ) 



    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        twist_stamper, 
        ldlidar_node,
        # camera_node,
        # rviz_node
    ])
