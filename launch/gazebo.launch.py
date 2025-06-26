from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'rc_teleop' 
    urdf_file = 'robot_description.xacro'

    urdf_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'urdf',
        urdf_file
    ])

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        urdf_path
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        gazebo_launch,

        # Publica o robot_description para outros nós
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        # Spawna o robô no mundo do Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'meu_robo'],
            output='screen'
        ),

        # Manager do controller
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                PathJoinSubstitution([
                    FindPackageShare(pkg_name),
                    'config',
                    'diff_drive_controller.yaml'
                ]),
                {'use_sim_time': True}  
            ],
            output='screen'
        ),

        # Broadcaster de estados das juntas
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Controlador diff drive
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
