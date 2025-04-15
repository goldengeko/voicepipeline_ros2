import py_trees
from py_trees.common import Status
import rclpy

class SpeedUp(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="SpeedUp"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.logger = rclpy.logging.get_logger(name)
        self.has_run = False
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if self.has_run:
            self.logger.info("SpeedUp already executed, returning SUCCESS.")
            return Status.SUCCESS

        new_speed = self.spot_bt.speed + 0.1
        new_turn = self.spot_bt.turn + 0.1
        if new_speed <= self.spot_bt.max_speed and new_turn <= self.spot_bt.max_speed:
            self.spot_bt.speed = new_speed
            self.spot_bt.turn = new_turn
            self.logger.info(f"Speed increased to {self.spot_bt.speed}, Turn to {self.spot_bt.turn}")
            self.has_run = True
            return Status.SUCCESS
        self.logger.info(f"Speed or turn at max ({self.spot_bt.max_speed}). Returning FAILURE.")
        return Status.FAILURE

    def terminate(self, new_status):
        self.has_run = False
        super().terminate(new_status)

class SlowDown(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="SlowDown"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.logger = rclpy.logging.get_logger(name)
        self.has_run = False
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if self.has_run:
            self.logger.info("SlowDown already executed, returning SUCCESS.")
            return Status.SUCCESS

        new_speed = self.spot_bt.speed - 0.1
        new_turn = self.spot_bt.turn - 0.1
        if new_speed >= self.spot_bt.min_speed and new_turn >= self.spot_bt.min_speed:
            self.spot_bt.speed = new_speed
            self.spot_bt.turn = new_turn
            self.logger.info(f"Speed decreased to {self.spot_bt.speed}, Turn to {self.spot_bt.turn}")
            self.has_run = True
            return Status.SUCCESS
        self.logger.info(f"Speed or turn at min ({self.spot_bt.min_speed}). Returning FAILURE.")
        return Status.FAILURE

    def terminate(self, new_status):
        self.has_run = False
        super().terminate(new_status)
