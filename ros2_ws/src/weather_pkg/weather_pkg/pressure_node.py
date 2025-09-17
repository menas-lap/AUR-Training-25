import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random
from time import sleep

class PressureNode(Node):
  def __init__(self):
    super().__init__('temprature_node')
    self.publisher = self.create_publisher(Int32, '/pressure', 10)
    self.timer = self.create_timer(3.0, self.timer_callback)
    self.get_logger().info('Pressure node started')

  def timer_callback(self):
    msg = Int32()
    msg.data = random.randint(900, 1100)
    self.publisher.publish(msg)
    self.get_logger().info(f'Publishing Pressure: {msg.data}hPa')

def main(args=None):
  rclpy.init(args=args)
  node = PressureNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()