#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA


class CounterLED(Node):
   

    def __init__(self, blink_hz: float = 1.0):
        super().__init__('counter_led')

        # Vehicle name from environment
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        if not self.vehicle_name:
            raise RuntimeError("VEHICLE_NAME environment variable not set")

        # LED publisher
        self.publisher = self.create_publisher(
            LEDPattern,
            f'/{self.vehicle_name}/led_pattern',
            10  # QoS depth
        )

        
        self.counter = 0

        
        self.blink_hz = blink_hz
        self.period = 1.0 / self.blink_hz
        self.timer = self.create_timer(self.period, self.publish_pattern)

        self.get_logger().info(f"CounterLED node started for {self.vehicle_name}")

    def publish_pattern(self):
        """
        Publish LEDPattern using a counter:
          - All LEDs blink every other tick
          - One LED moves across the strip like a chase
        """
        msg = LEDPattern()
        msg.rgb_vals = []

        # Blink all LEDs on/off every other tick
        blink_on = (self.counter % 2 == 0)

        # Chase LED index (0-4)
        chase_index = self.counter % 5

        for i in range(5):
            if blink_on or i == chase_index:
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  
            else:
                color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)  
            msg.rgb_vals.append(color)

        self.publisher.publish(msg)

        # Increment counter for next animation frame
        self.counter += 1


def main():
    rclpy.init()
    node = CounterLED(blink_hz=2.0)  # 2 Hz blink
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
