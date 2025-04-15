import py_trees
from py_trees.common import Status
import time
from bt_commander.behaviors.movement import MoveForward, TurnLeft, TurnRight, BackUp, Stop
from bt_commander.behaviors.speed import SpeedUp, SlowDown
from bt_commander.behaviors.services import StandUp, SitDown, RollOver, SelfRight
from bt_commander.behaviors.pushups import PushUps
import rclpy

class ListenCommand(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="ListenCommand"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.command = None
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()  # Global blackboard

    def update(self):
        if self.command is None or self.spot_bt.last_command_time + self.spot_bt.timeout_duration < time.time():
            self.logger.info("No command or timeout reached. Returning RUNNING.")
            return Status.RUNNING

        current_state = self.blackboard.get("current_state")
        moving_states = ['following', 'turning_left', 'turning_right', 'backing_up']
        idle_stopped_states = ['idle', 'stopped']
        cmd_map = self.spot_bt.command_mappings
        cmd_name = self.name.lower().replace("listencommand_", "")

        if self.command == "continue" and cmd_name in ["follow", "turnleft", "turnright", "backup"] and current_state in idle_stopped_states:
            if self.spot_bt.last_movement_command in cmd_map[cmd_name]:
                self.logger.info(f"Continue resuming last movement '{self.spot_bt.last_movement_command}' in state '{current_state}'. Returning SUCCESS.")
                return Status.SUCCESS
            return Status.FAILURE

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

def build_behavior_tree(spot_bt):
    """Construct the behavior tree for SpotBT."""
    tree = py_trees.composites.Selector("Root", memory=False)

    # Instantiate behaviors
    spot_bt.listen_node_stop = ListenCommand(spot_bt, name="ListenCommand_Stop")
    spot_bt.listen_node_follow = ListenCommand(spot_bt, name="ListenCommand_Follow")
    spot_bt.listen_node_turn_left = ListenCommand(spot_bt, name="ListenCommand_TurnLeft")
    spot_bt.listen_node_turn_right = ListenCommand(spot_bt, name="ListenCommand_TurnRight")
    spot_bt.listen_node_back_up = ListenCommand(spot_bt, name="ListenCommand_BackUp")
    spot_bt.listen_node_speed_up = ListenCommand(spot_bt, name="ListenCommand_SpeedUp")
    spot_bt.listen_node_slow_down = ListenCommand(spot_bt, name="ListenCommand_SlowDown")
    spot_bt.listen_node_stand_up = ListenCommand(spot_bt, name="ListenCommand_StandUp")
    spot_bt.listen_node_sit_down = ListenCommand(spot_bt, name="ListenCommand_SitDown")
    spot_bt.listen_node_roll_over = ListenCommand(spot_bt, name="ListenCommand_RollOver")
    spot_bt.listen_node_self_right = ListenCommand(spot_bt, name="ListenCommand_SelfRight")
    spot_bt.listen_node_push_ups = ListenCommand(spot_bt, name="ListenCommand_PushUps")

    # Sequences
    stop_sequence = py_trees.composites.Sequence("Stop Sequence", memory=True)
    stop_sequence.add_children([spot_bt.listen_node_stop, Stop(spot_bt, spot_bt.vel_pub)])

    follow_sequence = py_trees.composites.Sequence("Follow Sequence", memory=True)
    follow_sequence.add_children([spot_bt.listen_node_follow, MoveForward(spot_bt, spot_bt.vel_pub)])

    turn_left_sequence = py_trees.composites.Sequence("Turn Left Sequence", memory=True)
    turn_left_sequence.add_children([spot_bt.listen_node_turn_left, TurnLeft(spot_bt, spot_bt.vel_pub)])

    turn_right_sequence = py_trees.composites.Sequence("Turn Right Sequence", memory=True)
    turn_right_sequence.add_children([spot_bt.listen_node_turn_right, TurnRight(spot_bt, spot_bt.vel_pub)])

    back_up_sequence = py_trees.composites.Sequence("Back Up Sequence", memory=True)
    back_up_sequence.add_children([spot_bt.listen_node_back_up, BackUp(spot_bt, spot_bt.vel_pub)])

    speed_up_sequence = py_trees.composites.Sequence("Speed Up Sequence", memory=True)
    speed_up_sequence.add_children([spot_bt.listen_node_speed_up, SpeedUp(spot_bt)])

    slow_down_sequence = py_trees.composites.Sequence("Slow Down Sequence", memory=True)
    slow_down_sequence.add_children([spot_bt.listen_node_slow_down, SlowDown(spot_bt)])

    stand_up_sequence = py_trees.composites.Sequence("Stand Up Sequence", memory=True)
    stand_up_sequence.add_children([spot_bt.listen_node_stand_up, StandUp(spot_bt)])

    sit_down_sequence = py_trees.composites.Sequence("Sit Down Sequence", memory=True)
    sit_down_sequence.add_children([spot_bt.listen_node_sit_down, SitDown(spot_bt)])

    roll_over_sequence = py_trees.composites.Sequence("Roll Over Sequence", memory=True)
    roll_over_sequence.add_children([spot_bt.listen_node_roll_over, RollOver(spot_bt)])

    self_right_sequence = py_trees.composites.Sequence("Self Right Sequence", memory=True)
    self_right_sequence.add_children([spot_bt.listen_node_self_right, SelfRight(spot_bt)])

    push_ups_sequence = py_trees.composites.Sequence("Push Ups Sequence", memory=True)
    push_ups_sequence.add_children([spot_bt.listen_node_push_ups, PushUps(spot_bt)])

    idle_behavior = py_trees.behaviours.Running("Idle")
    idle_behavior.update = lambda: idle_update(spot_bt)

    tree.add_children([
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
        idle_behavior
    ])
    return tree

def idle_update(spot_bt):
    """Custom update for Idle behavior."""
    blackboard = py_trees.blackboard.Blackboard()
    current_state = blackboard.get("current_state")
    if current_state != 'idle':
        spot_bt.logger.info(f"Transitioning to idle from '{current_state}'.")
        blackboard.set("current_state", "idle")
    if spot_bt.last_command_time + spot_bt.timeout_duration < time.time():
        spot_bt.logger.info("Timeout reached, resetting commands.")
        for node in spot_bt.tree.children:
            if isinstance(node, py_trees.composites.Sequence):
                node.children[0].command = None
    return Status.RUNNING
