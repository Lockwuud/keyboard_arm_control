from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_arm_control',
            executable='keyboard_arm_control_node',
            name='keyboard_arm_control',
            output='screen',
            parameters=[{
                'control_mode': 'cartesian',
                'max_linear_velocity': 0.05,
                'max_angular_velocity': 0.3,
                'joint_velocity': 0.2
            }]
        )
    ])