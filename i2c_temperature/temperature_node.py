import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import smbus2
import time

class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('i2c_temperature_node')
        self.publisher_ = self.create_publisher(Temperature, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.read_and_publish)

        self.bus_number = 1
        self.address = 0x40
        self.bus = smbus2.SMBus(self.bus_number)

    def read_and_publish(self):
        try:
            data = self.bus.read_i2c_block_data(self.address, 0x00, 2)
            temp_raw = (data[1] << 8) | data[0]
            temp_c = temp_raw * 0.01

            msg = Temperature()
            msg.temperature = temp_c
            msg.variance = 0.0
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'i2c_sensor'

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published temperature: {temp_c:.2f} °C')

        except Exception as e:
            self.get_logger().warn(f'I2C read failed: {e}')

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
