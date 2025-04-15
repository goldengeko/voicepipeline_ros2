import py_trees
from py_trees.common import Status
from geometry_msgs.msg import TwistStamped
import rclpy

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="MoveForward"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        self.logger.info("Moving forward...")
        cmd = TwistStamped()
        cmd.twist.linear.x = self.spot_bt.speed
        self.vel_pub.publish(cmd)
        self.spot_bt.last_command = self.spot_bt.listen_node_follow.command
        self.spot_bt.last_movement_command = self.spot_bt.last_command
        self.blackboard.set("current_state", "following")
        return Status.RUNNING

class TurnLeft(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="TurnLeft"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        self.logger.info("Turning left...")
        cmd = TwistStamped()
        cmd.twist.angular.z = self.spot_bt.turn
        self.vel_pub.publish(cmd)
        self.spot_bt.last_command = self.spot_bt.listen_node_turn_left.command
        self.spot_bt.last_movement_command = self.spot_bt.last_command
        self.blackboard.set("current_state", "turning_left")
        return Status.RUNNING

class TurnRight(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="TurnRight"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        self.logger.info("Turning right...")
        cmd = TwistStamped()
        cmd.twist.angular.z = -self.spot_bt.turn
        self.vel_pub.publish(cmd)
        self.spot_bt.last_command = self.spot_bt.listen_node_turn_right.command
        self.spot_bt.last_movement_command = self.spot_bt.last_command
        self.blackboard.set("current_state", "turning_right")
        return Status.RUNNING

class BackUp(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="BackUp"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        self.logger.info("Backing up...")
        cmd = TwistStamped()
        cmd.twist.linear.x = -self.spot_bt.speed
        self.vel_pub.publish(cmd)
        self.spot_bt.last_command = self.spot_bt.listen_node_back_up.command
        self.spot_bt.last_movement_command = self.spot_bt.last_command
        self.blackboard.set("current_state", "backing_up")
        return Status.RUNNING

class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="Stop"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        self.logger.info("Stopping...")
        cmd = TwistStamped()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.vel_pub.publish(cmd)
        self.spot_bt.last_command = self.spot_bt.listen_node_stop.command
        self.blackboard.set("current_state", "stopped")
        return Status.SUCCESS
