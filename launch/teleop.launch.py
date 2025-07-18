from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Teleop Manager Node
        Node(
            package='laser_rahcm_teleop',
            executable='TeleopManager',
            name='teleop_manager',
            output='screen',
            parameters=[]
        ),
        
        # Wheel Controller Node (for ground mode)
        Node(
            package='laser_rahcm_teleop',
            executable='WhellController',
            name='wheel_controller',
            output='screen',
            parameters=[
                {'axis_linear': 1},
                {'axis_angular': 3},
                {'scale_linear': 1.0},
                {'scale_angular': 1.0}
            ]
        ),
        
        # Thruster Controller Node (for aquatic mode)
        Node(
            package='laser_rahcm_teleop',
            executable='ThrusterController',
            name='thruster_controller',
            output='screen',
            parameters=[
                {'axis_linear': 1},
                {'axis_angular': 3},
                {'scale_linear': 1.0},
                {'scale_angular': 1.0}
            ]
        ),
        
        # Optional: Joy node for joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'dev': '/dev/input/js0'},
                {'deadzone': 0.1},
                {'autorepeat_rate': 20.0}
            ]
        )
    ])