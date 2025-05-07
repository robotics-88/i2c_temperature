import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import smbus2
import time
from smbus2 import i2c_msg

class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('i2c_temperature_node')
        self.publisher_ = self.create_publisher(Temperature, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.read_and_publish)

        self.bus_number = 1
        self.address = 0x48
        self.bus = smbus2.SMBus(self.bus_number)
        self.last_log_time = 0.0

    def read_and_publish(self):
        try:
            # 1) Trigger one-shot conversion (no-hold mode)
            write = i2c_msg.write(self.address, [0xF3])
            self.bus.i2c_rdwr(write)

            # 2) Wait for conversion to complete (max ~10 ms; give extra)
            time.sleep(0.15)

            # 3) Read exactly 2 bytes (MSB, LSB)
            read = i2c_msg.read(self.address, 2)
            self.bus.i2c_rdwr(read)
            msb, lsb = list(read)

            # 4) Assemble and convert per datasheet
            raw = (msb << 8) | lsb
            temp_c = ((175.72 * raw) / 65536.0) - 46.85

            # 6) Publish ROS msg
            msg = Temperature()
            msg.temperature = temp_c
            msg.variance    = 0.0
            msg.header.stamp     = self.get_clock().now().to_msg()
            msg.header.frame_id = 'i2c_sensor'
            self.publisher_.publish(msg)

            current_time = time.time()
            if current_time - self.last_log_time >= 9.5:
                rclpy.logging.get_logger('i2c_temperature_node').info(f'Temperature: {temp_c:.2f} °C')
                self.last_log_time = current_time

        except Exception as e:
            self.get_logger().warn(f"I2C read failed: {e}")

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
