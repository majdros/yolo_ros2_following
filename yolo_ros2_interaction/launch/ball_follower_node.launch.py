from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Availabe Launch Arguments
    image_width = DeclareLaunchArgument(name = 'image_width', default_value = '640')
    linear_speed_gain = DeclareLaunchArgument(name = 'linear_speed_gain', default_value = '3.0')
    stop_distance_m = DeclareLaunchArgument(name = 'stop_distance_m', default_value = '0.30')

    # Launch Configurations
    image_width_config = LaunchConfiguration('image_width')
    linear_speed_gain_config = LaunchConfiguration('linear_speed_gain')
    stop_distance_m_config = LaunchConfiguration('stop_distance_m')

    # ball_follower_node
    ball_follower_node = Node(
        package = 'yolo_ros2_interaction',
        executable= 'ball_follower_node',
        name = 'ball_follower_node',
        parameters = [{
            'image_width' : image_width_config,
            'linear_speed_gain' : linear_speed_gain_config,
            'stop_distance_m' : stop_distance_m_config,
        }]
    )


    return LaunchDescription([
    image_width,
    linear_speed_gain,
    stop_distance_m,
    ball_follower_node,
    ])