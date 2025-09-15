#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim_msgs.msg import Pose
from turtlesim_msgs.srv import Spawn, Kill
from std_msgs.msg import Int32
import random
import math
import time

class TurtleChase(Node):
    def __init__(self):
        super().__init__('turtle_chase')
        # Publishing the score
        self.score_pub = self.create_publisher(Int32, 'score', 10)
        self.score = 0

        # Storing the Poses
        self.player_pose = None
        self.enemy_positions = {}
        self.enemy_subs = {}

        # Clients for the services
        self.spawn_cli = self.create_client(Spawn, 'spawn')
        self.kill_cli = self.create_client(Kill, 'kill')

        # Waiting for the services
        self.get_logger().info('Waiting for spawn/kill services...')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for spawn service...')
        while not self.kill_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for kill service...')

        # Player Pose Subscriber
        self.create_subscription(Pose, 'turtle1/pose', self.player_callback, 10)

        # Spawning the 3 Enemies
        for i in range(1, 4):
            name = f'enemy{i}'
            self.spawn_enemy(name)

        # Collision Timer for checking
        self.create_timer(0.1, self.check_collisions)

    def player_callback(self, msg: Pose):
        self.player_pose = msg

    def enemy_callback_factory(self, name):
        def callback(msg: Pose):
            self.enemy_positions[name] = msg
        return callback

    def spawn_enemy(self, name):
      req = Spawn.Request()
      req.x = random.uniform(1.0, 10.0)
      req.y = random.uniform(1.0, 10.0)
      req.theta = 0.0
      req.name = name   # force same name

      future = self.spawn_cli.call_async(req)

      def callback(fut):
          try:
              res = fut.result()
          except Exception as e:
              self.get_logger().error(f"Failed to spawn {name}: {e}")
              return

          # turtlesim always returns the actual name used
          self.get_logger().info(f"Spawned {name} at ({req.x:.2f},{req.y:.2f})")

          topic = f'/{name}/pose'
          # IMPORTANT: remove old sub if exists
          if name in self.enemy_subs:
              self.destroy_subscription(self.enemy_subs[name])

          sub = self.create_subscription(Pose, topic, self.enemy_callback_factory(name), 10)
          self.enemy_subs[name] = sub

      future.add_done_callback(callback)


    def kill_enemy(self, name):
        req = Kill.Request()
        req.name = name
        future = self.kill_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None:
            self.get_logger().info(f'Killed {name}')
        else:
            self.get_logger().error('Failed to kill ' + name)
        # Removing Cache
        if name in self.enemy_positions:
            del self.enemy_positions[name]
        if name in self.enemy_subs:
            del self.enemy_subs[name]

    def find_distance(self, p1: Pose, p2: Pose):
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def check_collisions(self):
      if self.player_pose is None:
          return

      for name, pose in list(self.enemy_positions.items()):
          dist = self.find_distance(self.player_pose, pose)
          if dist < 1:  # threshold
              self.get_logger().info(f'Hit detected: {name} (dist={dist:.2f})')
              self.score += 1
              self.score_pub.publish(Int32(data=self.score))

              # kill & respawn this enemy only
              self.kill_enemy(name)
              self.spawn_enemy(name)



def main(args=None):
    rclpy.init(args=args)
    node = TurtleChase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()