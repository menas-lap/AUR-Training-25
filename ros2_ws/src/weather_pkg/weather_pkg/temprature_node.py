import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random
from time import sleep

class TempratureNode(Node):
  def __init__(self):
    super().__init__('temprature_node')
    self.publisher = self.create_publisher(Int32, '/temprature', 10)
    self.timer = self.create_timer(1.0, self.timer_callback)
    self.get_logger().info('Temprature node started')

  def timer_callback(self):
    msg = Int32()
    msg.data = random.randint(15, 40)
    self.publisher.publish(msg)
    self.get_logger().info(f'Publishing Temp: {msg.data}°C')

def main(args=None):
  rclpy.init(args=args)
  node = TempratureNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()