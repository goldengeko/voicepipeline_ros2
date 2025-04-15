#!/usr/bin/env python3
import rclpy
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
import py_trees
from py_trees.common import Status
from bt_commander.config.commands import load_command_mappings
from bt_commander.tree.builder import build_behavior_tree

class SpotBT(Node):
    def __init__(self):
        super().__init__('spot_bt')
        self.vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.logger = self.get_logger()

        # Context variables
        self.current_state = 'idle'
        self.last_command = None
        self.last_movement_command = None
        self.last_command_time = 0.0
        self.speed = 0.5
        self.turn = 0.5
        self.max_speed = 2.0
        self.min_speed = 0.1
        self.timeout_duration = 10.0

        # Load command mappings
        self.command_mappings = load_command_mappings('/home/guts/voicepipeline_ros2/src/commands.yaml')

        # Build the behavior tree
        self.tree = build_behavior_tree(self)
        self.bt_runner = py_trees.trees.BehaviourTree(self.tree)
        self.logger.info("Spot BT initialized")

        # Use the global blackboard (no register_key)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set("current_state", self.current_state)

        # Subscription
        self.create_subscription(String, '/whisper_transcript', self.command_callback, 10)

    def command_callback(self, msg):
        for node in self.tree.children:
            if isinstance(node, py_trees.composites.Sequence):
                listen_node = node.children[0]  # First child is ListenCommand
                listen_node.set_command(msg)
        self.logger.info(f"Updated command to: '{msg.data}' in state '{self.current_state}'")

    def tick(self):
        self.bt_runner.tick()
        tip = self.bt_runner.tip()
        self.current_state = self.blackboard.get("current_state")
        self.logger.info(f"Current tree status: {tip.name if tip else 'None'} - {tip.status if tip else 'N/A'} - "
                         f"State: {self.current_state} - Last Command: {self.last_command} - "
                         f"Last Movement: {self.last_movement_command} - Speed: {self.speed} - Turn: {self.turn}")
        if tip and tip.status == Status.SUCCESS and tip.name in ["Stop", "SpeedUp", "SlowDown", "StandUp", "SitDown", "RollOver", "SelfRight"]:
            self.logger.info("Resetting commands after completed action.")
            for node in self.tree.children:
                if isinstance(node, py_trees.composites.Sequence):
                    node.children[0].command = None  # Reset ListenCommand

def main():
    rclpy.init()
    node = SpotBT()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            node.tick()
            executor.spin_once(timeout_sec=0.1)  # Use the executor to spin
    except Exception as e:
        node.logger.error(f"Error in main loop: {e}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
