import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Package directory
    interaction_pkg_dir = get_package_share_directory('yolo_ros2_interaction')
    yolo_pkg_dir = get_package_share_directory('yolo_bringup')
    
    # Available Launch Arguments
    ## Camera_node
    camera_index = DeclareLaunchArgument('camera_index', default_value='0')
    frame_rate = DeclareLaunchArgument('frame_rate', default_value='30.0')
    camera_calibration_file = DeclareLaunchArgument('camera_calibration_file', default_value='usb_cam.yaml')

    ## ball_follower_node
    image_width = DeclareLaunchArgument('image_width', default_value='640')
    linear_speed_gain = DeclareLaunchArgument('linear_speed_gain', default_value='2.5')
    stop_distance_m = DeclareLaunchArgument('stop_distance_m', default_value='0.30')

    # Include Camera Launch File
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(interaction_pkg_dir, 'launch', 'camera.launch.py')
        ]),
        launch_arguments={
            'camera_index': LaunchConfiguration('camera_index'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'camera_calibration_file': LaunchConfiguration('camera_calibration_file'),
        }.items()
    )

    # Include Ball Follower Launch File
    ball_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(interaction_pkg_dir, 'launch', 'ball_follower_node.launch.py')
        ]),
        launch_arguments={
            'image_width': LaunchConfiguration('image_width'),
            'linear_speed_gain': LaunchConfiguration('linear_speed_gain'),
            'stop_distance_m': LaunchConfiguration('stop_distance_m'),
        }.items()
    )

    # Include yolo Launch File
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(yolo_pkg_dir, 'launch', 'yolo.launch.py')
        ]),
    )


    return LaunchDescription([
        camera_index,
        frame_rate,
        camera_calibration_file,
        image_width,
        linear_speed_gain,
        stop_distance_m,

        camera_launch,
        ball_follower_launch,
        yolo_launch,
    ])