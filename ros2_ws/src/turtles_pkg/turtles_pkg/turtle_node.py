import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class TurtleNode(Node):
  def __init__(self):
    super().__init__('turtle_node')
    self.turtle1_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    self.turtle2_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
    self.linear_speed = 2.0 # UP/DOWN Speed
    self.angular_speed = 1.8 # Rotations Speed (rad/sec)
    self.keys_pressed = set() # Listening For Keys

    # Start Listening
    self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
    self.listener.start()
    self.get_logger().info('Keyboard node started! Arrows for Turtle1, WASD for Turtle2. Ctrl+C to exit')

  # Detecting Key Press
  def on_press(self, key):
    key_str = str(key)

    if hasattr(key, 'char'):
      key_str = key.char.lower() if key.char else key_str

    self.keys_pressed.add(key_str)
    self.update_velocity()

  # Detecting Realeases
  def on_release(self, key):
    key_str = str(key)

    if hasattr(key, 'char'):
      key_str = key.char.lower() if key.char else key_str

    self.keys_pressed.discard(key_str)
    self.update_velocity

  # Updating Velocities
  def update_velocity(self):
    # Turtle1 ---> Arrows
    twist1 = Twist()
    if 'Key.up' in self.keys_pressed:
      twist1.linear.x = self.linear_speed

    if 'Key.down' in self.keys_pressed:
      twist1.linear.x = -self.linear_speed

    if 'Key.left' in self.keys_pressed:
      twist1.angular.z = self.angular_speed

    if 'Key.right' in self.keys_pressed:
      twist1.angular.z = -self.angular_speed

    self.turtle1_pub.publish(twist1)

    # Turtle2 ---> WASD
    twist2 = Twist()
    if 'w' in self.keys_pressed:
      twist2.linear.x = self.linear_speed

    if 's' in self.keys_pressed:
      twist2.linear.x = -self.linear_speed

    if 'a' in self.keys_pressed:
      twist2.angular.z = self.angular_speed

    if 'd' in self.keys_pressed:
      twist2.angular.z = -self.angular_speed

    self.turtle2_pub.publish(twist2)

def main(args=None):
  rclpy.init(args=args)
  node = TurtleNode()
  
  try:
    rclpy.spin(node)

  except KeyboardInterrupt:
    pass

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()