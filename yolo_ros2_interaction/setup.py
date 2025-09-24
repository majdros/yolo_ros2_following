from setuptools import find_packages, setup

package_name = 'yolo_ros2_interaction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',[
            'launch/camera.launch.py',
            'launch/ball_follower_node.launch.py',
            'launch/yolo_ros2_interaction_bringup.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/web_cam.yaml',
            'config/usb_cam.yaml',
        ]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='majd',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = yolo_ros2_interaction.camera_node:main',
            'ball_follower_node = yolo_ros2_interaction.ball_follower_node:main',
        ],
    },
)
