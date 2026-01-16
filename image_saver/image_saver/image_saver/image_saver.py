#!/usr/bin/python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.counter = 0
        self.image_detected = False
        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.image_callback, 10)

        self.detect_pub = self.create_publisher(
            Bool,
            f'/{self.vehicle_name}/red_object_detected',
            10
        )
        self.get_logger().info('Image Saver Started')

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.int8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is None:

            self.get_logger().warn("Failed to decode image")

            return
        if self.detect_object(image):
            if not self.image_detected:
                msg = Bool()
                msg.data = True
                self.detect_pub.publish(msg)
            self.image_detected = True
            filename = os.path.join(self.output_dir, f'image_{self.counter}.jpg')
            cv2.imwrite(filename, image)
            self.get_logger().info(f"Object detected! Saved {filename}")
            self.counter += 1
        else:
            self.image_detected = False
            msg = Bool()
            msg.data = False
            self.detect_pub.publish(msg)



    def detect_object(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0,160,80])
        upper_red1 = np.array([8,255,255])

        lower_red2 = np.array([172,160,80])
        upper_red2 = np.array([180,255,255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        red_pixels = cv2.countNonZero(mask)

        return red_pixels > 750
def main():
    rclpy.init()
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
