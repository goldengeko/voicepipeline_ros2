#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
import time
import numpy as np

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

class Commander(Node):
    def __init__(self):
        super().__init__('whisper_commander')

        # Kinova Kortex API setup
        self.username = "admin"
        self.password = "admin"
        self.ip = "192.168.50.9"  # Adjust as needed
        self.tcp_port = 10000

        self.transport = TCPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)
        self.transport.connect(self.ip, self.tcp_port)

        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = self.username
        session_info.password = self.password
        session_info.session_inactivity_timeout = 10000  # milliseconds
        session_info.connection_inactivity_timeout = 2000  # milliseconds

        self.session_manager = SessionManager(self.router)
        self.get_logger().info(f"Logging as {self.username} on device {self.ip}")
        self.session_manager.CreateSession(session_info)

        self.base = BaseClient(self.router)

        # Clear faults if any
        if self.base.GetArmState().active_state == Base_pb2.ARMSTATE_IN_FAULT:
            self.base.ClearFaults()
            time.sleep(1)

        # Set servoing mode to Single Level Servoing
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)

        # Create subscribers and publishers (for debugging, keep publisher optional)
        self.command_subscriber = self.create_subscription(
            String, 'whisper_transcript', self.transcript_callback, 10
        )
        self.command_publisher = self.create_publisher(
            TwistStamped, '/twist_controller/commands', 10
        )  # Optional, for debugging

        # Command state variables
        self.twist_command = Base_pb2.TwistCommand()
        self.twist_command.reference_frame = Base_pb2.CARTESIAN_JOYSTICK
        self.twist_command.duration = 0

        self.last_command = None
        self.speed = 0.2  # Linear speed (m/s)
        self.turn = 0.2  # Angular speed (rad/s)
        self.max_speed = 2.0
        self.min_speed = 0.1
        self.is_moving = False

        # Publishing rate for continuous command (20 Hz)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.publish_last_command)

        # Timeout for stopping the robot (10 seconds)
        self.timeout_duration = 10.0
        self.timeout_timer = self.create_timer(self.timeout_duration, self.stop_on_timeout)
        self.latest_cmd_end_time = 0.0

        # Command dictionaries
        self.straight = ['go straight', 'move forward', 'advance', 'proceed', 'forward', 'straight']
        self.backward = ['go back', 'move back', 'reverse', 'retreat', 'backward']
        self.left = ['go left', 'move left', 'turn left', 'left']
        self.right = ['go right', 'move right', 'turn right', 'right']
        self.look_right = ['look right']
        self.look_left = ['look left']
        self.stop = ['stop', 'halt', 'cease', 'pause', 'standby']
        self.speed_up = ['speed up', 'faster', 'accelerate', 'increase speed']
        self.slow_down = ['slow down', 'slower', 'decelerate', 'reduce speed']

    def move_forward(self):
        self.twist_command.twist.linear_x = self.speed
        self.last_command = 'forward'
        self.get_logger().info("Moving forward")
        self.is_moving = True

    def move_backward(self):
        self.twist_command.twist.linear_x = -self.speed
        self.last_command = 'backward'
        self.get_logger().info("Moving backward")
        self.is_moving = True

    def move_left(self):
        self.twist_command.twist.linear_y = self.speed
        self.last_command = 'left'
        self.get_logger().info("Moving left")
        self.is_moving = True

    def move_right(self):
        self.twist_command.twist.linear_y = -self.speed
        self.last_command = 'right'
        self.get_logger().info("Moving right")
        self.is_moving = True

    def rotate_left(self):
        self.twist_command.twist.angular_z = self.turn
        self.last_command = 'look_left'
        self.get_logger().info("Rotating left")
        self.is_moving = True

    def rotate_right(self):
        self.twist_command.twist.angular_z = -self.turn
        self.last_command = 'look_right'
        self.get_logger().info("Rotating right")
        self.is_moving = True

    def stop_movement(self):
        self.twist_command.twist.linear_x = 0.0
        self.twist_command.twist.linear_y = 0.0
        self.twist_command.twist.linear_z = 0.0
        self.twist_command.twist.angular_x = 0.0
        self.twist_command.twist.angular_y = 0.0
        self.twist_command.twist.angular_z = 0.0
        self.last_command = 'stop'
        self.get_logger().info("Stopping")
        self.is_moving = False

    def adjust_speed(self, delta):
        new_speed = self.speed + delta
        if self.min_speed <= new_speed <= self.max_speed:
            self.speed = new_speed
            self.get_logger().info(f"Speed adjusted to: {self.speed}")
        else:
            self.get_logger().info(f"Speed adjustment out of bounds: {new_speed}")

    def publish_last_command(self):
        if self.is_moving and self.last_command != 'stop':
            self.latest_cmd_end_time = self.get_clock().now().nanoseconds / 1e9 + 0.2
            self.base.SendTwistCommand(self.twist_command)
            self.get_logger().info(
                f"Sent: linear=({self.twist_command.twist.linear_x}, "
                f"{self.twist_command.twist.linear_y}, {self.twist_command.twist.linear_z}), "
                f"angular=({self.twist_command.twist.angular_x}, "
                f"{self.twist_command.twist.angular_y}, {self.twist_command.twist.angular_z})"
            )
            # Optional: Publish to ROS topic for debugging
            ros_twist = TwistStamped()
            ros_twist.header.frame_id = 'tool_frame'
            ros_twist.header.stamp = self.get_clock().now().to_msg()
            ros_twist.twist.linear.x = float(self.twist_command.twist.linear_x)
            ros_twist.twist.linear.y = float(self.twist_command.twist.linear_y)
            ros_twist.twist.linear.z = float(self.twist_command.twist.linear_z)
            ros_twist.twist.angular.x = float(self.twist_command.twist.angular_x)
            ros_twist.twist.angular.y = float(self.twist_command.twist.angular_y)
            ros_twist.twist.angular.z = float(self.twist_command.twist.angular_z)
            self.command_publisher.publish(ros_twist)
        else:
            if self.latest_cmd_end_time > 0.0 and \
               self.get_clock().now().nanoseconds / 1e9 > self.latest_cmd_end_time:
                self.base.Stop()
                self.latest_cmd_end_time = 0.0

    def stop_on_timeout(self):
        if self.is_moving:
            self.get_logger().info("Timeout reached, stopping the robot.")
            self.stop_movement()
            self.base.Stop()

    def transcript_callback(self, msg):
        transcription = msg.data.lower()
        if not transcription.strip():
            self.get_logger().info("Ignoring empty transcription")
            return

        # Reset twist components to avoid residual commands
        self.stop_movement()

        command_phrases = [
            phrase.strip().rstrip('.').rstrip(',').rstrip('!').rstrip('?')
            for phrase in transcription.split(',')
        ]

        for phrase in command_phrases:
            if phrase in self.straight:
                self.move_forward()
                self.reset_timeout_timer()
                break
            elif phrase in self.backward:
                self.move_backward()
                self.reset_timeout_timer()
                break
            elif phrase in self.left:
                self.move_left()
                self.reset_timeout_timer()
                break
            elif phrase in self.right:
                self.move_right()
                self.reset_timeout_timer()
                break
            elif phrase in self.look_left:
                self.rotate_left()
                self.reset_timeout_timer()
                break
            elif phrase in self.look_right:
                self.rotate_right()
                self.reset_timeout_timer()
                break
            elif phrase in self.stop:
                self.stop_movement()
                self.reset_timeout_timer()
                break
            elif phrase in self.speed_up:
                self.adjust_speed(0.1)
                break
            elif phrase in self.slow_down:
                self.adjust_speed(-0.1)
                break
            else:
                self.get_logger().info(f"Unknown command: '{phrase}'")
                self.stop_movement()

    def reset_timeout_timer(self):
        self.timeout_timer.cancel()
        self.timeout_timer = self.create_timer(self.timeout_duration, self.stop_on_timeout)

    def destroy_node(self):
        self.get_logger().info("Shutting down commander node.")
        self.stop_movement()
        self.base.Stop()
        time.sleep(1)
        self.session_manager.CloseSession()
        self.transport.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
