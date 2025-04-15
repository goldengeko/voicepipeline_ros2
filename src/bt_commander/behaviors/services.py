import py_trees
from py_trees.common import Status
from std_srvs.srv import Trigger
import rclpy

class StandUp(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="StandUp"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.client = self.spot_bt.create_client(Trigger, '/stand')
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Stand service unavailable.")
            return Status.FAILURE
        self.logger.info("Standing up...")
        req = Trigger.Request()
        self.spot_bt.executor.create_task(self.client.call_async, req)
        self.spot_bt.last_command = self.spot_bt.listen_node_stand_up.command
        self.blackboard.set("current_state", "standing")
        return Status.SUCCESS

class SitDown(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="SitDown"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.client = self.spot_bt.create_client(Trigger, '/sit')
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Sit service unavailable.")
            return Status.FAILURE
        self.logger.info("Sitting down...")
        req = Trigger.Request()
        self.spot_bt.executor.create_task(self.client.call_async, req)
        self.spot_bt.last_command = self.spot_bt.listen_node_sit_down.command
        self.blackboard.set("current_state", "sitting")
        return Status.SUCCESS

class RollOver(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="RollOver"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.client = self.spot_bt.create_client(Trigger, '/rollover')
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Rollover service unavailable.")
            return Status.FAILURE
        self.logger.info("Rolling over...")
        req = Trigger.Request()
        self.spot_bt.executor.create_task(self.client.call_async, req)
        self.spot_bt.last_command = self.spot_bt.listen_node_roll_over.command
        self.blackboard.set("current_state", "stopped")
        return Status.SUCCESS

class SelfRight(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="SelfRight"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.client = self.spot_bt.create_client(Trigger, '/self_right')
        self.logger = rclpy.logging.get_logger(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Self right service unavailable.")
            return Status.FAILURE
        self.logger.info("Self-righting...")
        req = Trigger.Request()
        self.spot_bt.executor.create_task(self.client.call_async, req)
        self.spot_bt.last_command = self.spot_bt.listen_node_self_right.command
        self.blackboard.set("current_state", "standing")
        return Status.SUCCESS
