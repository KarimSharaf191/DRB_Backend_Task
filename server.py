#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_interface.msg import Cust
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests

class UnifiedShapeDetector(Node):
    def __init__(self):
        super().__init__('unified_shape_detector')

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        # ROI: center square
        self.roi_x, self.roi_y, self.roi_w, self.roi_h = 200, 100, 400, 400

        # Camera → Robot base transformation
        self.T_base_camera = np.array([
            [1, 0, 0, -0.735],
            [0, 1, 0,  0.000],
            [0, 0, 1,  0.000],
            [0, 0, 0,  1.000]
        ])

        # Publisher for detected shapes
        self.shape_pub = self.create_publisher(Cust, 'shape_detected_topic', 10)

        # ✅ Correct Roboflow model details
        self.roboflow_url = "https://detect.roboflow.com/shapedetection-dathy/2"
        self.roboflow_api_key = "guGIFWjFEbreiDFllXjT"

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Extract ROI
        roi = frame[self.roi_y:self.roi_y + self.roi_h, self.roi_x:self.roi_x + self.roi_w]
        image_path = "/tmp/roi_frame.jpg"
        cv2.imwrite(image_path, roi)

        # Send image to Roboflow
        try:
            with open(image_path, "rb") as img_file:
                response = requests.post(
                    self.roboflow_url,
                    params={"api_key": self.roboflow_api_key},
                    files={"file": img_file}
                )
            response.raise_for_status()
        except Exception as e:
            self.get_logger().error(f"Roboflow request failed: {e}")
            return

        predictions = response.json().get("predictions", [])

        for pred in predictions:
            class_name = pred["class"]
            x = int(pred["x"])
            y = int(pred["y"])

            # Convert from ROI to full frame coordinates
            u = x + self.roi_x
            v = y + self.roi_y
            depth = 1.0  # Simulated depth (meters)

            # Pixel to base frame
            point_camera = np.array([u * 0.001, v * 0.001, depth])
            P_camera_hom = np.append(point_camera, 1.0).reshape(4, 1)
            P_base_hom = self.T_base_camera @ P_camera_hom
            x_base, y_base, z_base = P_base_hom[:3].flatten()

            # Log
            self.get_logger().info(f'🎯 Detected {class_name} at:')
            self.get_logger().info(f'  - Pixel: ({u}, {v}), Depth: {depth:.3f} m')
            self.get_logger().info(f'  - Base Frame: X={x_base:.3f}, Y={y_base:.3f}, Z={z_base:.3f}')

            # Publish
            shape_msg = Cust()
            shape_msg.coords = 1
            shape_msg.shape = class_name
            shape_msg.x = float(x_base)
            shape_msg.y = float(y_base)
            shape_msg.z = float(z_base)
            self.shape_pub.publish(shape_msg)

            # Draw on frame
            cv2.putText(frame, class_name, (u, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.circle(frame, (u, v), 5, (0, 255, 0), -1)

        # Draw ROI
        cv2.rectangle(frame, (self.roi_x, self.roi_y),
                      (self.roi_x + self.roi_w, self.roi_y + self.roi_h), (255, 0, 0), 2)

        cv2.imshow("Unified Shape Detection", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedShapeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
