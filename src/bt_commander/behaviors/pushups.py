import py_trees
from py_trees.common import Status
from std_srvs.srv import Trigger
import rclpy

class PushUps(py_trees.behaviour.Behaviour):
    def __init__(self, spot_bt, name="PushUps"):
        super().__init__(name)
        self.spot_bt = spot_bt
        self.sit_client = self.spot_bt.create_client(Trigger, '/sit')
        self.stand_client = self.spot_bt.create_client(Trigger, '/stand')
        self.logger = rclpy.logging.get_logger(name)
        self.count = 0
        self.target = 2
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if not self.sit_client.wait_for_service(timeout_sec=1.0) or not self.stand_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Push-ups services unavailable.")
            return Status.FAILURE

        if self.count == 0:
            self.logger.info(f"Starting {self.target} push-ups...")
            cmd = self.spot_bt.listen_node_push_ups.command
            numbers = [int(n) for n in cmd.split() if n.isdigit()]
            self.target = numbers[0] * 2 if numbers else 2

        if self.count < self.target:
            req = Trigger.Request()
            if self.count % 2 == 0:
                self.logger.info("Sitting for push-up...")
                self.spot_bt.executor.create_task(self.sit_client.call_async, req)
                self.blackboard.set("current_state", "sitting")
            else:
                self.logger.info("Standing for push-up...")
                self.spot_bt.executor.create_task(self.stand_client.call_async, req)
                self.blackboard.set("current_state", "standing")
            self.count += 1
            return Status.RUNNING
        else:
            self.logger.info("Push-ups completed.")
            self.count = 0
            self.spot_bt.last_command = self.spot_bt.listen_node_push_ups.command
            self.blackboard.set("current_state", "stopped")
            return Status.SUCCESS
