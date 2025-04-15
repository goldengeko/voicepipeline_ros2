import rclpy
from rclpy.node import Node
import py_trees
from py_trees.common import Status
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from spot_msgs.srv import SetLocomotion
from std_srvs.srv import Trigger
from py_trees.display import render_dot_tree
import time
import yaml

class SpotBT(Node):
    def __init__(self):
        super().__init__('spot_bt')
        self.vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.logger = self.get_logger()

        # Load command mappings from YAML
        with open('/home/guts/voicepipeline_ros2/src/commands.yaml', 'r') as f:
            self.command_mappings = yaml.safe_load(f)['commands']

        # Context variables
        self.current_state = 'idle'
        self.last_command = None  # Last executed command (for "continue")
        self.last_movement_command = None  # Last movement command specifically
        self.last_command_time = time.time()
        self.speed = 0.5
        self.turn = 0.5
        self.max_speed = 2.0
        self.min_speed = 0.1
        self.timeout_duration = 10.0

        self.create_subscription(String, '/whisper_transcript', self.command_callback, 10)

        # Root Selector
        self.tree = py_trees.composites.Selector("Root", memory=False)

        # Behaviors
        self.listen_node_stop = ListenCommand(self, name="ListenCommand_Stop")
        self.listen_node_follow = ListenCommand(self, name="ListenCommand_Follow")
        self.listen_node_turn_left = ListenCommand(self, name="ListenCommand_TurnLeft")
        self.listen_node_turn_right = ListenCommand(self, name="ListenCommand_TurnRight")
        self.listen_node_back_up = ListenCommand(self, name="ListenCommand_BackUp")
        self.listen_node_speed_up = ListenCommand(self, name="ListenCommand_SpeedUp")
        self.listen_node_slow_down = ListenCommand(self, name="ListenCommand_SlowDown")
        self.listen_node_stand_up = ListenCommand(self, name="ListenCommand_StandUp")
        self.listen_node_sit_down = ListenCommand(self, name="ListenCommand_SitDown")
        self.listen_node_roll_over = ListenCommand(self, name="ListenCommand_RollOver")
        self.listen_node_self_right = ListenCommand(self, name="ListenCommand_SelfRight")
        self.listen_node_push_ups = ListenCommand(self, name="ListenCommand_PushUps")

        self.move_forward = MoveForward(self, self.vel_pub)
        self.turn_left = TurnLeft(self, self.vel_pub)
        self.turn_right = TurnRight(self, self.vel_pub)
        self.back_up = BackUp(self, self.vel_pub)
        self.stop = Stop(self, self.vel_pub)
        self.speed_up = SpeedUp(self)
        self.slow_down = SlowDown(self)
        self.stand_up = StandUp(self)
        self.sit_down = SitDown(self)
        self.roll_over = RollOver(self)
        self.self_right = SelfRight(self)
        self.push_ups = PushUps(self)

        # Sequences
        stop_sequence = py_trees.composites.Sequence("Stop Sequence", memory=True)
        stop_sequence.add_children([self.listen_node_stop, self.stop])

        follow_sequence = py_trees.composites.Sequence("Follow Sequence", memory=True)
        follow_sequence.add_children([self.listen_node_follow, self.move_forward])

        turn_left_sequence = py_trees.composites.Sequence("Turn Left Sequence", memory=True)
        turn_left_sequence.add_children([self.listen_node_turn_left, self.turn_left])

        turn_right_sequence = py_trees.composites.Sequence("Turn Right Sequence", memory=True)
        turn_right_sequence.add_children([self.listen_node_turn_right, self.turn_right])

        back_up_sequence = py_trees.composites.Sequence("Back Up Sequence", memory=True)
        back_up_sequence.add_children([self.listen_node_back_up, self.back_up])

        speed_up_sequence = py_trees.composites.Sequence("Speed Up Sequence", memory=True)
        speed_up_sequence.add_children([self.listen_node_speed_up, self.speed_up])

        slow_down_sequence = py_trees.composites.Sequence("Slow Down Sequence", memory=True)
        slow_down_sequence.add_children([self.listen_node_slow_down, self.slow_down])

        stand_up_sequence = py_trees.composites.Sequence("Stand Up Sequence", memory=True)
        stand_up_sequence.add_children([self.listen_node_stand_up, self.stand_up])

        sit_down_sequence = py_trees.composites.Sequence("Sit Down Sequence", memory=True)
        sit_down_sequence.add_children([self.listen_node_sit_down, self.sit_down])

        roll_over_sequence = py_trees.composites.Sequence("Roll Over Sequence", memory=True)
        roll_over_sequence.add_children([self.listen_node_roll_over, self.roll_over])

        self_right_sequence = py_trees.composites.Sequence("Self Right Sequence", memory=True)
        self_right_sequence.add_children([self.listen_node_self_right, self.self_right])

        push_ups_sequence = py_trees.composites.Sequence("Push Ups Sequence", memory=True)
        push_ups_sequence.add_children([self.listen_node_push_ups, self.push_ups])

        # Idle fallback
        self.idle_behavior = py_trees.behaviours.Running("Idle")
        self.idle_behavior.update = lambda: self._idle_update()

        self.tree.add_children([
            stop_sequence,
            follow_sequence,
            turn_left_sequence,
            turn_right_sequence,
            back_up_sequence,
            speed_up_sequence,
            slow_down_sequence,
            stand_up_sequence,
            sit_down_sequence,
            roll_over_sequence,
            self_right_sequence,
            push_ups_sequence,
            self.idle_behavior
        ])
        self.bt_runner = py_trees.trees.BehaviourTree(self.tree)
        self.logger.info("Spot BT initialized")
        self.save_tree()

    def _idle_update(self):
        if self.current_state != 'idle':
            self.logger.info(f"Transitioning to idle from '{self.current_state}'.")
            self.current_state = 'idle'
        if self.last_command_time + self.timeout_duration < time.time():
            self.logger.info("Timeout reached, resetting commands.")
            for node in [self.listen_node_stop, self.listen_node_follow, self.listen_node_turn_left,
                         self.listen_node_turn_right, self.listen_node_back_up, self.listen_node_speed_up,
                         self.listen_node_slow_down, self.listen_node_stand_up, self.listen_node_sit_down,
                         self.listen_node_roll_over, self.listen_node_self_right, self.listen_node_push_ups]:
                node.command = None
        return Status.RUNNING

    def command_callback(self, msg):
        for node in [self.listen_node_stop, self.listen_node_follow, self.listen_node_turn_left,
                     self.listen_node_turn_right, self.listen_node_back_up, self.listen_node_speed_up,
                     self.listen_node_slow_down, self.listen_node_stand_up, self.listen_node_sit_down,
                     self.listen_node_roll_over, self.listen_node_self_right, self.listen_node_push_ups]:
            node.set_command(msg)
        self.logger.info(f"Updated command to: '{msg.data}' in state '{self.current_state}'")

    def tick(self):
        self.bt_runner.tick()
        tip = self.bt_runner.tip()
        self.logger.info(f"Current tree status: {tip.name if tip else 'None'} - {tip.status if tip else 'N/A'} - "
                         f"State: {self.current_state} - Last Command: {self.last_command} - "
                         f"Last Movement: {self.last_movement_command} - Speed: {self.speed} - Turn: {self.turn}")
        if tip and tip.status == Status.SUCCESS and tip.name in ["Stop", "StandUp", "SitDown", "RollOver", "SelfRight"]:
            self.logger.info("Resetting commands after completed action.")
            for node in [self.listen_node_stop, self.listen_node_follow, self.listen_node_turn_left,
                         self.listen_node_turn_right, self.listen_node_back_up, self.listen_node_speed_up,
                         self.listen_node_slow_down, self.listen_node_stand_up, self.listen_node_sit_down,
                         self.listen_node_roll_over, self.listen_node_self_right, self.listen_node_push_ups]:
                node.command = None

    def save_tree(self):
        render_dot_tree(self.tree)

class ListenCommand(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="ListenCommand"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.command = None
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        if self.command is None or self.spot_bt.last_command_time + self.spot_bt.timeout_duration < time.time():
            self.logger.info("No command or timeout reached. Returning RUNNING.")
            return Status.RUNNING

        current_state = self.spot_bt.current_state
        moving_states = ['following', 'turning_left', 'turning_right', 'backing_up']
        idle_stopped_states = ['idle', 'stopped']
        cmd_map = self.spot_bt.command_mappings
        cmd_name = self.name.lower().replace("listencommand_", "")

        # Special handling for "continue"
        if self.command == "continue" and cmd_name in ["follow", "turnleft", "turnright", "backup"] and current_state in idle_stopped_states:
            if self.spot_bt.last_movement_command in cmd_map[cmd_name]:
                self.logger.info(f"Continue resuming last movement '{self.spot_bt.last_movement_command}' in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            return Status.FAILURE

        # Normal command handling
        if cmd_name in cmd_map and self.command in cmd_map[cmd_name]:
            if cmd_name == "stop" and current_state in moving_states:
                self.logger.info(f"Stop command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "follow" and current_state in idle_stopped_states:
                self.logger.info(f"Follow command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "turnleft" and current_state in idle_stopped_states:
                self.logger.info(f"Turn left command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "turnright" and current_state in idle_stopped_states:
                self.logger.info(f"Turn right command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "backup" and current_state in idle_stopped_states:
                self.logger.info(f"Back up command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "speedup":
                self.logger.info(f"Speed up command '{self.command}' valid. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "slowdown":
                self.logger.info(f"Slow down command '{self.command}' valid. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "standup" and current_state != 'standing':
                self.logger.info(f"Stand up command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "sitdown" and current_state != 'sitting':
                self.logger.info(f"Sit down command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "rollover" and current_state in ['sitting', 'stopped']:
                self.logger.info(f"Roll over command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "selfright":
                self.logger.info(f"Self right command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            elif cmd_name == "pushups" and current_state in idle_stopped_states:
                self.logger.info(f"Push ups command '{self.command}' valid in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS

        self.logger.info(f"Command '{self.command}' invalid for '{self.name}' in state '{current_state}'. Returning FAILURE.")
        return Status.FAILURE

    def set_command(self, msg):
        self.command = msg.data.lower().strip().strip('.')
        self.spot_bt.last_command_time = time.time()
        self.logger.info(f"Received and stored command: '{self.command}' in state '{self.spot_bt.current_state}'")

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="MoveForward"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        self.logger.info("Moving forward...")
        cmd = TwistStamped()
        cmd.twist.linear.x = self.spot_bt.speed
        self.vel_pub.publish(cmd)
        self.spot_bt.current_state = 'following'
        self.spot_bt.last_command = self.spot_bt.listen_node_follow.command
        self.spot_bt.last_movement_command = self.spot_bt.last_command  # Update last movement
        return Status.RUNNING

class TurnLeft(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="TurnLeft"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        self.logger.info("Turning left...")
        cmd = TwistStamped()
        cmd.twist.angular.z = self.spot_bt.turn
        self.vel_pub.publish(cmd)
        self.spot_bt.current_state = 'turning_left'
        self.spot_bt.last_command = self.spot_bt.listen_node_turn_left.command
        self.spot_bt.last_movement_command = self.spot_bt.last_command
        return Status.RUNNING

class TurnRight(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="TurnRight"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        self.logger.info("Turning right...")
        cmd = TwistStamped()
        cmd.twist.angular.z = -self.spot_bt.turn
        self.vel_pub.publish(cmd)
        self.spot_bt.current_state = 'turning_right'
        self.spot_bt.last_command = self.spot_bt.listen_node_turn_right.command
        self.spot_bt.last_movement_command = self.spot_bt.last_command
        return Status.RUNNING

class BackUp(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="BackUp"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        self.logger.info("Backing up...")
        cmd = TwistStamped()
        cmd.twist.linear.x = -self.spot_bt.speed
        self.vel_pub.publish(cmd)
        self.spot_bt.current_state = 'backing_up'
        self.spot_bt.last_command = self.spot_bt.listen_node_back_up.command
        self.spot_bt.last_movement_command = self.spot_bt.last_command
        return Status.RUNNING

class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, vel_pub, name="Stop"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.vel_pub = vel_pub
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        self.logger.info("Stopping...")
        cmd = TwistStamped()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.vel_pub.publish(cmd)
        self.spot_bt.current_state = 'stopped'
        self.spot_bt.last_command = self.spot_bt.listen_node_stop.command
        # Do not update last_movement_command here
        return Status.SUCCESS

class SpeedUp(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="SpeedUp"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        new_speed = self.spot_bt.speed + 0.1
        new_turn = self.spot_bt.turn + 0.1
        if new_speed <= self.spot_bt.max_speed and new_turn <= self.spot_bt.max_speed:
            self.spot_bt.speed = new_speed
            self.spot_bt.turn = new_turn
            self.logger.info(f"Speed increased to {self.spot_bt.speed}, Turn to {self.spot_bt.turn}")
            return Status.SUCCESS
        self.logger.info(f"Speed or turn at max ({self.spot_bt.max_speed}). Returning FAILURE.")
        return Status.FAILURE

class SlowDown(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="SlowDown"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        new_speed = self.spot_bt.speed - 0.1
        new_turn = self.spot_bt.turn - 0.1
        if new_speed >= self.spot_bt.min_speed and new_turn >= self.spot_bt.min_speed:
            self.spot_bt.speed = new_speed
            self.spot_bt.turn = new_turn
            self.logger.info(f"Speed decreased to {self.spot_bt.speed}, Turn to {self.spot_bt.turn}")
            return Status.SUCCESS
        self.logger.info(f"Speed or turn at min ({self.spot_bt.min_speed}). Returning FAILURE.")
        return Status.FAILURE

class StandUp(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="StandUp"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.client = self.spot_bt.create_client(Trigger, '/stand')
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Stand service unavailable.")
            return Status.FAILURE
        self.logger.info("Standing up...")
        req = Trigger.Request()
        self.spot_bt.get_node().executor.create_task(self.client.call_async, req)
        self.spot_bt.current_state = 'standing'
        self.spot_bt.last_command = self.spot_bt.listen_node_stand_up.command
        return Status.SUCCESS

class SitDown(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="SitDown"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.client = self.spot_bt.create_client(Trigger, '/sit')
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Sit service unavailable.")
            return Status.FAILURE
        self.logger.info("Sitting down...")
        req = Trigger.Request()
        self.spot_bt.get_node().executor.create_task(self.client.call_async, req)
        self.spot_bt.current_state = 'sitting'
        self.spot_bt.last_command = self.spot_bt.listen_node_sit_down.command
        return Status.SUCCESS

class RollOver(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="RollOver"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.client = self.spot_bt.create_client(Trigger, '/rollover')
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Rollover service unavailable.")
            return Status.FAILURE
        self.logger.info("Rolling over...")
        req = Trigger.Request()
        self.spot_bt.get_node().executor.create_task(self.client.call_async, req)
        self.spot_bt.current_state = 'stopped'
        self.spot_bt.last_command = self.spot_bt.listen_node_roll_over.command
        return Status.SUCCESS

class SelfRight(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="SelfRight"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.client = self.spot_bt.create_client(Trigger, '/self_right')
        self.logger = rclpy.logging.get_logger(name)

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Self right service unavailable.")
            return Status.FAILURE
        self.logger.info("Self-righting...")
        req = Trigger.Request()
        self.spot_bt.get_node().executor.create_task(self.client.call_async, req)
        self.spot_bt.current_state = 'standing'
        self.spot_bt.last_command = self.spot_bt.listen_node_self_right.command
        return Status.SUCCESS

class PushUps(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="PushUps"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.sit_client = self.spot_bt.create_client(Trigger, '/sit')
        self.stand_client = self.spot_bt.create_client(Trigger, '/stand')
        self.logger = rclpy.logging.get_logger(name)
        self.count = 0
        self.target = 2  # Default to 2 push-ups if no number specified

    def update(self):
        if not self.sit_client.wait_for_service(timeout_sec=1.0) or not self.stand_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Push-ups services unavailable.")
            return Status.FAILURE

        if self.count == 0:
            self.logger.info(f"Starting {self.target} push-ups...")
            # Parse number from command if present (simplified for demo)
            cmd = self.spot_bt.listen_node_push_ups.command
            numbers = [int(n) for n in cmd.split() if n.isdigit()]
            self.target = numbers[0] * 2 if numbers else 2  # 2 actions (sit, stand) per push-up

        if self.count < self.target:
            req = Trigger.Request()
            if self.count % 2 == 0:
                self.logger.info("Sitting for push-up...")
                self.spot_bt.get_node().executor.create_task(self.sit_client.call_async, req)
                self.spot_bt.current_state = 'sitting'
            else:
                self.logger.info("Standing for push-up...")
                self.spot_bt.get_node().executor.create_task(self.stand_client.call_async, req)
                self.spot_bt.current_state = 'standing'
            self.count += 1
            return Status.RUNNING
        else:
            self.logger.info("Push-ups completed.")
            self.count = 0
            self.spot_bt.current_state = 'stopped'
            self.spot_bt.last_command = self.spot_bt.listen_node_push_ups.command
            return Status.SUCCESS

def main():
    rclpy.init()
    node = SpotBT()
    try:
        while rclpy.ok():
            node.tick()
            rclpy.spin_once(node, timeout_sec=1.0)
    except Exception as e:
        node.logger.error(f"Error in main loop: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
