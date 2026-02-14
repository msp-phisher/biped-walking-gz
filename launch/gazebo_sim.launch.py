import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Get Package Paths
    pkg_share = get_package_share_directory('biped_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'biped_control.xacro')

    # 2. Parse Robot Description (URDF)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            xacro_file
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # 3. Start Gazebo (Ignition Fortress)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf -v 4'
        }.items()
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 5. Bridge (Clock + IMU)
    # FIX: Link name changed from 'base_link' to 'torso' to match URDF
    ign_imu_topic = '/world/empty/model/simple_biped/link/torso/sensor/imu_sensor/imu'
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            f'{ign_imu_topic}@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ],
        remappings=[
            (ign_imu_topic, '/imu')
        ],
        output='screen'
    )

    # 6. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_biped',
            '-topic', 'robot_description',
            '-z', '0.8' # Spawn slightly higher to avoid floor collision
        ],
        output='screen'
    )

    # 7. Spawn Controllers
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    biped_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['biped_controller'],
        output='screen'
    )

    # Delay controllers to allow Gazebo/ros2_control plugin to load
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster, biped_controller]
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_robot,
        delayed_controllers
    ])
