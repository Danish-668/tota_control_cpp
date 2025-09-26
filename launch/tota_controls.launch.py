from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    tota_control_cpp_launch = Node(
        package='tota_control_cpp',
        executable='joystick_controller',
        name='joystick_controller'
    )

    return LaunchDescription([
        joy_node,
        tota_control_cpp_launch,
    ])