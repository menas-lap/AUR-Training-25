import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random
from time import sleep

class HumidityNode(Node):
  def __init__(self):
    super().__init__('humidity_node')
    self.publisher = self.create_publisher(Int32, '/humidity', 10)
    self.timer = self.create_timer(2.0, self.timer_callback)
    self.get_logger().info('Humidity node started')

  def timer_callback(self):
    msg = Int32()
    msg.data = random.randint(20, 100)
    self.publisher.publish(msg)
    self.get_logger().info(f'Publishing Humidity: {msg.data}%')

def main(args=None):
  rclpy.init(args=args)
  node = HumidityNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()