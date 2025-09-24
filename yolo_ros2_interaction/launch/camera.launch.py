from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Available Launch Arguments
    camera_index = DeclareLaunchArgument(name = 'camera_index', default_value = '0')
    frame_rate = DeclareLaunchArgument(name = 'frame_rate', default_value = '30.0')
    camera_calibration_file = DeclareLaunchArgument(name = 'camera_calibration_file', default_value = 'usb_cam.yaml')

    # Launch Configurations
    camera_index_config = LaunchConfiguration('camera_index')
    frame_rate_config = LaunchConfiguration('frame_rate')
    camera_calibration_file_config = LaunchConfiguration ('camera_calibration_file')

    # Camera Node
    camera_node = Node(
        package = 'yolo_ros2_interaction',
        executable = 'camera_node',
        namespace = 'rgb_camera',
        parameters = [{
            'camera_index' : camera_index_config,
            'frame_rate' : frame_rate_config,
            'camera_calibration_file' : camera_calibration_file_config,
                    }]
    )


    return LaunchDescription([
    camera_index,
    frame_rate,
    camera_calibration_file,
    camera_node,
    ])