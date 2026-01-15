#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)

        self.vehicle_name = os.getenv('VEHICLE_NAME')
        if not self.vehicle_name:
            raise RuntimeError("VEHICLE_NAME environment variable not set")

        self.counter = 0

        self.create_subscription(
            CompressedImage,
            f'/{self.vehicle_name}/camera_node/image/compressed',
            self.save_image,
            10
        )

    def save_image(self, msg):
        self.counter += 1

        # Save every 30th frame
        if self.counter % 30 != 0:
            return

        filename = os.path.join(self.output_dir, f"{self.counter}.jpg")

        with open(filename, 'wb') as f:
            f.write(msg.data)

        self.get_logger().info(f"Saved image: {filename}")


def main():
    rclpy.init()
    node = ImageSaver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

