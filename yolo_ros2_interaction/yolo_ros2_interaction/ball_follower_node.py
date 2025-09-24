#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower_node')

        # Params
        self.declare_parameter(name= 'image_width', value= 640)
        self.declare_parameter(name= 'linear_speed_gain', value= 3.0)   # Scaling factor for linear velocity
        self.declare_parameter(name='stop_distance_m' ,value=0.30)      # Z

        # the three ball-klasses
        self.declare_parameter(name='tennisball_diameter_m', value=0.07)    # Y
        self.declare_parameter(name='baseball_diameter_m', value=0.1)       # Y
        self.declare_parameter(name='football_diameter_m', value=0.220)     # Y


        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.linear_speed_gain = self.get_parameter('linear_speed_gain').get_parameter_value().double_value
        self.stop_distance_m = self.get_parameter('stop_distance_m').get_parameter_value().double_value
        # self.target_size = self.get_parameter('target_size').get_parameter_value().double_value

        # self.ball_diameter_m = self.get_parameter('ball_diameter_m').get_parameter_value().double_value
        self.tennisball_diameter_m = self.get_parameter('tennisball_diameter_m').get_parameter_value().double_value
        self.baseball_diameter_m = self.get_parameter('baseball_diameter_m').get_parameter_value().double_value
        self.football_diameter_m = self.get_parameter('football_diameter_m').get_parameter_value().double_value

        self.focal_length_px = None
        self.target_sizes_px = {}

        latched_qos = QoSProfile(depth=1,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=QoSReliabilityPolicy.RELIABLE)

        self.detections_sub = self.create_subscription(DetectionArray, '/yolo/detections', self.detections_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/rgb_camera/camera_info', self.camera_info_callback, latched_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_yolo', 10)

        self.get_logger().info('Starting ball_follower_node')


    def camera_info_callback(self, msg: CameraInfo):
        try:
            k = msg.k
            fx, fy = float(k[0]), float(k[4])         # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.focal_length_px = (fx + fy) / 2.0    # the ball is round

            self.target_sizes_px = {                  # y = (f * Y) / Z
                'tennisball': (self.focal_length_px * self.tennisball_diameter_m) / self.stop_distance_m,
                'baseball': (self.focal_length_px * self.baseball_diameter_m) / self.stop_distance_m,
                'football': (self.focal_length_px * self.football_diameter_m) / self.stop_distance_m
            }

        except Exception as e:
            self.get_logger().error(f'Error in camera_info_callback: {str(e)}')


    def detections_callback(self, msg: DetectionArray):
        try:
            if not self.target_sizes_px:        # in case no CameraInfo 
                return

            cmd_vel = Twist()

            for det in msg.detections:
                ball_class = det.class_name
                if ball_class not in self.target_sizes_px:
                    continue

                # Center point x
                center_x = det.bbox.center.position.x
                box_width = det.bbox.size.x

                target_size = self.target_sizes_px[ball_class]

                # Error calculation: deviation from image center
                error_x = center_x - self.image_width / 2.0     # relative to half the image width
                kp_angular = 1 / (self.image_width / 2.0)
                angular_z = -float(error_x) * kp_angular

                # Size error: difference between target and estimation
                size_error = target_size - box_width
                kp_linear = 1 / target_size
                linear_x = float(size_error) * kp_linear

                # Final Speed
                final_linear_x = linear_x * self.linear_speed_gain
                final_angular_z = angular_z

                # Set Speed limits
                final_linear_x = max(min(final_linear_x, 1.1), -1.1)
                final_angular_z = max(min(final_angular_z, 1.7), -1.7)

                # Set Twist
                cmd_vel.linear.x = final_linear_x
                cmd_vel.angular.z = final_angular_z

                break       # only the first detection

            # publish command_velocity
            self.cmd_pub.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f'Error in detections_callback: {str(e)}')
    

def main(args=None):
    rclpy.init(args=args)

    ball_follower = BallFollower()
    rclpy.spin(ball_follower)
    ball_follower.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()