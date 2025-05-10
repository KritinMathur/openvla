

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from ackermann_msgs.msg import AckermannDriveStamped
# from cv_bridge import CvBridge
# import requests
# import base64
# import numpy as np
# import cv2  # ← make sure cv2 is imported!

# SERVER_URL = 'http://34.224.93.220:5000/predict'

# class VLAClientNode(Node):
#     def __init__(self):
#         super().__init__('openvla_client')
#         self.bridge = CvBridge()
#         self.latest_image = None
#         self.instruction = None

#         self.get_logger().info('Initializing VLAClientNode…')

#         # Subscribe to RealSense color topic
#         self.create_subscription(
#             Image,
#             '/camera/color/image_raw',
#             self.image_callback,
#             10
#         )
#         # Subscribe to instruction topic
#         self.create_subscription(
#             String,
#             '/vla_instruction',
#             self.instruction_callback,
#             10
#         )
#         # Publisher for drive commands
#         self.drive_pub = self.create_publisher(
#             AckermannDriveStamped,
#             '/drive',
#             10
#         )

#     def image_callback(self, msg: Image):
#         # self.get_logger().info('image_callback fired')
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
#             self.latest_image = cv_image
#             # self.get_logger().info(f'Received image of shape {cv_image.shape}')
#         except Exception as e:
#             self.get_logger().error(f'Failed to convert image: {e}')

#     def instruction_callback(self, msg: String):
#         self.get_logger().info('instruction_callback fired')
#         self.instruction = msg.data
#         self.get_logger().info(f'Received instruction: "{self.instruction}"')
#         if self.latest_image is not None and self.instruction:
#             self.get_logger().info('Both image & instruction available, calling server…')
#             self.call_server_and_publish()
#         else:
#             self.get_logger().warn('Waiting for both image and instruction before calling server')

#     def call_server_and_publish(self):
#         self.get_logger().info('call_server_and_publish entered')
#         try:
#             # Resize to 224×224 before encoding
#             self.get_logger().info(f'Resizing image from {self.latest_image.shape} to (224, 224)')
#             resized = cv2.resize(self.latest_image, (224, 224))

#             _, buffer = cv2.imencode('.jpg', resized)
#             jpg_b64 = base64.b64encode(buffer).decode('utf-8')
#             payload = {'image': jpg_b64, 'instruction': self.instruction}

#             self.get_logger().info('Sending request to server…')
#             resp = requests.post(SERVER_URL, json=payload, timeout=5)
#             resp.raise_for_status()

#             data = resp.json()
#             action = data.get('action', [])
#             self.get_logger().info(f'Server response action: {action}')

#             if len(action) >= 2:
#                 drive_msg = AckermannDriveStamped()
#                 drive_msg.header.stamp = self.get_clock().now().to_msg()
#                 drive_msg.drive.speed = 0.3 * float(action[0])
#                 # drive_msg.drive.speed = 
#                 drive_msg.drive.steering_angle = 10.0 *float(action[1])
#                 self.drive_pub.publish(drive_msg)
#                 self.get_logger().info(
#                     f"Published drive action: speed={drive_msg.drive.speed}, steering={drive_msg.drive.steering_angle}"
#                 )
#             else:
#                 self.get_logger().warn('Action from server malformed or too short')
#         except Exception as e:
#             self.get_logger().error(f"Inference request failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = VLAClientNode()
#     node.get_logger().info('Spinning VLAClientNode…')
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('KeyboardInterrupt received, shutting down')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         print('Node destroyed, rclpy shutdown complete')

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# import os
# import base64
# import requests
# import numpy as np
# import cv2

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from ackermann_msgs.msg import AckermannDriveStamped
# from cv_bridge import CvBridge

# SERVER_URL = 'http://34.224.93.220:5000/predict'

# class VLAClientNode(Node):
#     def __init__(self):
#         super().__init__('openvla_client')
#         self.bridge = CvBridge()
#         self.latest_image = None
#         self.instruction = None

#         # Directory where we'll save .npz files
#         self.save_dir = os.path.expanduser('~/vla_data')
#         os.makedirs(self.save_dir, exist_ok=True)
#         self.save_counter = 0

#         self.get_logger().info('Initializing VLAClientNode…')

#         # Subscribe to RealSense color images
#         self.create_subscription(
#             Image,
#             '/camera/color/image_raw',
#             self.image_callback,
#             10
#         )
#         # Subscribe to instruction strings
#         self.create_subscription(
#             String,
#             '/vla_instruction',
#             self.instruction_callback,
#             10
#         )
#         # Publisher for drive commands
#         self.drive_pub = self.create_publisher(
#             AckermannDriveStamped,
#             '/drive',
#             10
#         )

#     def image_callback(self, msg: Image):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
#             self.latest_image = cv_image
#         except Exception as e:
#             self.get_logger().error(f'Failed to convert image: {e}')

#     def instruction_callback(self, msg: String):
#         self.get_logger().info('instruction_callback fired')
#         self.instruction = msg.data
#         self.get_logger().info(f'Received instruction: "{self.instruction}"')
#         if self.latest_image is not None and self.instruction:
#             self.get_logger().info('Both image & instruction available, calling server…')
#             self.call_server_and_publish()
#         else:
#             self.get_logger().warn('Waiting for both image and instruction before calling server')

#     def call_server_and_publish(self):
#         self.get_logger().info('call_server_and_publish entered')
#         try:
#             # 1) Resize image to 224×224
#             self.get_logger().info(f'Resizing image from {self.latest_image.shape} to (224, 224)')
#             resized = cv2.resize(self.latest_image, (224, 224))

#             # 2) Encode and send to server
#             _, buffer = cv2.imencode('.jpg', resized)
#             jpg_b64 = base64.b64encode(buffer).decode('utf-8')
#             payload = {'image': jpg_b64, 'instruction': self.instruction}

#             self.get_logger().info('Sending request to server…')
#             resp = requests.post(SERVER_URL, json=payload, timeout=5)
#             resp.raise_for_status()
#             data = resp.json()
#             action = data.get('action', [])
#             self.get_logger().info(f'Server response action: {action}')

#             # 3) Save image + action + instruction in .npz
#             try:
#                 action_array = np.array(action, dtype=np.float32)
#                 filename = os.path.join(self.save_dir, f'data_{self.save_counter:06d}.npz')
#                 np.savez_compressed(
#                     filename,
#                     image=resized,
#                     action=action_array,
#                     instruction=self.instruction
#                 )
#                 self.get_logger().info(f'Saved data to {filename}')
#                 self.save_counter += 1
#             except Exception as e:
#                 self.get_logger().error(f'Failed to save data: {e}')

#             # 4) Publish drive command
#             if len(action) >= 2:
#                 drive_msg = AckermannDriveStamped()
#                 drive_msg.header.stamp = self.get_clock().now().to_msg()
#                 drive_msg.drive.speed = 0.3 * float(action[0])
#                 drive_msg.drive.steering_angle = 10.0 * float(action[1])
#                 self.drive_pub.publish(drive_msg)
#                 self.get_logger().info(
#                     f"Published drive action: speed={drive_msg.drive.speed}, steering={drive_msg.drive.steering_angle}"
#                 )
#             else:
#                 self.get_logger().warn('Action from server malformed or too short')

#         except Exception as e:
#             self.get_logger().error(f"Inference request failed: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = VLAClientNode()
#     node.get_logger().info('Spinning VLAClientNode…')
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('KeyboardInterrupt received, shutting down')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         print('Node destroyed, rclpy shutdown complete')

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import os
import base64
import requests
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge

SERVER_URL = 'http://34.224.93.220:5000/predict'

class VLAClientNode(Node):
    def __init__(self):
        super().__init__('openvla_client')
        self.bridge = CvBridge()
        self.latest_image = None
        self.instruction = None
        # latest x,y pose from the particle filter
        self.latest_pose = (0.0, 0.0)

        # Directory where we'll save .npz files
        self.save_dir = os.path.expanduser('~/vla_data')
        os.makedirs(self.save_dir, exist_ok=True)
        self.save_counter = 0

        self.get_logger().info('Initializing VLAClientNode…')

        # Subscribe to RealSense color images
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        # Subscribe to instruction strings
        self.create_subscription(
            String,
            '/vla_instruction',
            self.instruction_callback,
            10
        )
        # Subscribe to PF odometry
        self.create_subscription(
            Odometry,
            '/pf/pose/odom',
            self.odom_callback,
            10
        )
        # Publisher for drive commands
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def instruction_callback(self, msg: String):
        self.instruction = msg.data
        self.get_logger().info(f'Received instruction: "{self.instruction}"')
        if self.latest_image is not None and self.instruction:
            self.call_server_and_publish()
        else:
            self.get_logger().warn('Waiting for both image and instruction before calling server')

    def odom_callback(self, msg: Odometry):
        # update latest x,y from PF
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.latest_pose = (x, y)

    def call_server_and_publish(self):
        try:
            # 1) Resize image
            resized = cv2.resize(self.latest_image, (224, 224))

            # 2) Encode & send to server
            _, buffer = cv2.imencode('.jpg', resized)
            jpg_b64 = base64.b64encode(buffer).decode('utf-8')
            payload = {'image': jpg_b64, 'instruction': self.instruction}

            resp = requests.post(SERVER_URL, json=payload, timeout=5)
            resp.raise_for_status()
            data = resp.json()
            action = data.get('action', [])
            self.get_logger().info(f'Server response action: {action}')

            # 3) Save .npz including x,y
            # try:
            #     action_array = np.array(action, dtype=np.float32)
            #     x, y = self.latest_pose
            #     fn = os.path.join(self.save_dir, f'data_{self.save_counter:06d}.npz')
            #     np.savez_compressed(
            #         fn,
            #         image=resized,
            #         action=action_array,
            #         instruction=self.instruction,
            #         x=np.float32(x),
            #         y=np.float32(y)
            #     )
            #     self.get_logger().info(f'Saved data to {fn}')
            #     self.save_counter += 1
            # except Exception as e:
            #     self.get_logger().error(f'Failed to save data: {e}')

            # 4) Publish drive command
            if len(action) >= 2:
                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = self.get_clock().now().to_msg()
                drive_msg.drive.speed = 0.2 * float(action[0])
                drive_msg.drive.steering_angle = 5.0 * float(action[1])
                self.drive_pub.publish(drive_msg)
                self.get_logger().info(
                    f"Published drive action: speed={drive_msg.drive.speed}, steering={drive_msg.drive.steering_angle}"
                )
            else:
                self.get_logger().warn('Action from server malformed or too short')

        except Exception as e:
            self.get_logger().error(f"Inference request failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VLAClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down VLAClientNode')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
