import rclpy
from rclpy.node import Node

class TimerNode(Node):
  def __init__(self):
    super().__init__('timer_node')
    self.count = 10 # Defining the start of the countdown
    self.timer = self.create_timer(1.0, self.countDown) # Starting the timer

  def countDown(self):
    # Counting Down
    if self.count > 0:
      self.get_logger().info(str(self.count))
      self.count -= 1
    
    # Ending the Count Down with Timer Up Message
    else:
      self.get_logger().info("0")
      self.get_logger().info("Time is up!")
      self.timer.cancel()

def main(args=None):
  rclpy.init(args=args)
  node = TimerNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()