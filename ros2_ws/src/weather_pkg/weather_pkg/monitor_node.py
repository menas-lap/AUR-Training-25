import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import os

class MonitorNode(Node):
  def __init__(self):
    super().__init__('temprature_node')
    self.temp = 0
    self.hum = 0
    self.press = 0
    self.subscription_temp = self.create_subscription(
      Int32, '/temprature', self.temp_callback, 10
    )
    self.subscription_hum = self.create_subscription(
      Int32, '/humidity', self.hum_callback, 10
    )
    self.subscription_press = self.create_subscription(
      Int32, '/pressure', self.press_callback, 10
    )

    self.log_file = 'weather_log.txt'
    self.get_logger().info('Monitor node started')

  def temp_callback(self, msg):
    self.temp = msg.data
    self.print_combined()

  def hum_callback(self, msg):
    self.hum = msg.data
    self.print_combined()

  def press_callback(self, msg):
    self.press = msg.data
    self.print_combined()

  def print_combined(self):
    output = f"\nTemp = {self.temp}°C.\nHumidity = {self.hum}%.\nPressure = {self.press} hPa."
    self.get_logger().info(output)
    with open(self.log_file, 'a') as File:
      File.write(output + '\n')

def main(args=None):
  rclpy.init(args=args)
  node = MonitorNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()