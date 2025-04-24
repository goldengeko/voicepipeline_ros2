#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import time
import re

class Commander(Node):
    def __init__(self):
        super().__init__('whisper_commander')

        # Create subscribers and publishers
        self.command_subscriber = self.create_subscription(
            String, 'whisper_transcript', self.transcript_callback, 10
        )
        self.twist_publisher = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )

        # Command state variables
        self.last_command = None
        self.linear_speed = 0.2  # m/s (default for translations)
        self.angular_speed = 0.3  # rad/s (default for rotations)
        self.max_linear_speed = 0.5
        self.min_linear_speed = 0.05
        self.max_angular_speed = 1.0
        self.min_angular_speed = 0.05
        self.is_moving = False

        # Coordinate frame (toggle between end-effector and base)
        self.frames = ['panda_hand', 'panda_link0']
        self.frame_index = 0  # Start with panda_hand
        self.frame_id = self.frames[self.frame_index]

        # Current twist command
        self.twist_command = TwistStamped()
        self.twist_command.header.frame_id = self.frame_id

        # Publishing rate (20 Hz for smooth servo control)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.publish_last_command)

        # Timeout (10 seconds)
        self.timeout_duration = 10.0
        self.timeout_timer = self.create_timer(self.timeout_duration, self.stop_on_timeout)

        # Command dictionaries
        self.move_up = ['move up', 'go up', 'up']
        self.move_down = ['move down', 'go down', 'down']
        self.move_left = ['move left', 'go left', 'left']
        self.move_right = ['move right', 'go right', 'right']
        self.look_up = ['look up']
        self.look_down = ['look down']
        self.look_left = ['look left']
        self.look_right = ['look right']
        self.switch_frame = ['switch frame', 'change frame', 'toggle frame']
        self.stop = ['stop', 'halt', 'cease', 'pause', 'standby']
        self.speed_up = ['speed up', 'faster', 'accelerate', 'increase speed']
        self.slow_down = ['slow down', 'slower', 'decelerate', 'reduce speed']

    def move_up_cmd(self):
        self.twist_command.twist.linear.x = 0.0
        self.twist_command.twist.linear.y = 0.0
        self.twist_command.twist.linear.z = self.linear_speed
        self.twist_command.twist.angular.x = 0.0
        self.twist_command.twist.angular.y = 0.0
        self.twist_command.twist.angular.z = 0.0
        self.last_command = 'move_up'
        self.get_logger().info(f"Moving up (Z+) in {self.frame_id}")
        self.is_moving = True

    def move_down_cmd(self):
        self.twist_command.twist.linear.x = 0.0
        self.twist_command.twist.linear.y = 0.0
        self.twist_command.twist.linear.z = -self.linear_speed
        self.twist_command.twist.angular.x = 0.0
        self.twist_command.twist.angular.y = 0.0
        self.twist_command.twist.angular.z = 0.0
        self.last_command = 'move_down'
        self.get_logger().info(f"Moving down (Z-) in {self.frame_id}")
        self.is_moving = True

    def move_left_cmd(self):
        self.twist_command.twist.linear.x = 0.0
        self.twist_command.twist.linear.y = self.linear_speed
        self.twist_command.twist.linear.z = 0.0
        self.twist_command.twist.angular.x = 0.0
        self.twist_command.twist.angular.y = 0.0
        self.twist_command.twist.angular.z = 0.0
        self.last_command = 'move_left'
        self.get_logger().info(f"Moving left (Y+) in {self.frame_id}")
        self.is_moving = True

    def move_right_cmd(self):
        self.twist_command.twist.linear.x = 0.0
        self.twist_command.twist.linear.y = -self.linear_speed
        self.twist_command.twist.linear.z = 0.0
        self.twist_command.twist.angular.x = 0.0
        self.twist_command.twist.angular.y = 0.0
        self.twist_command.twist.angular.z = 0.0
        self.last_command = 'move_right'
        self.get_logger().info(f"Moving right (Y-) in {self.frame_id}")
        self.is_moving = True

    def look_up_cmd(self):
        self.twist_command.twist.linear.x = 0.0
        self.twist_command.twist.linear.y = 0.0
        self.twist_command.twist.linear.z = 0.0
        self.twist_command.twist.angular.x = 0.0
        self.twist_command.twist.angular.y = self.angular_speed
        self.twist_command.twist.angular.z = 0.0
        self.last_command = 'look_up'
        self.get_logger().info(f"Looking up (pitch+) in {self.frame_id}")
        self.is_moving = True

    def look_down_cmd(self):
        self.twist_command.twist.linear.x = 0.0
        self.twist_command.twist.linear.y = 0.0
        self.twist_command.twist.linear.z = 0.0
        self.twist_command.twist.angular.x = 0.0
        self.twist_command.twist.angular.y = -self.angular_speed
        self.twist_command.twist.angular.z = 0.0
        self.last_command = 'look_down'
        self.get_logger().info(f"Looking down (pitch-) in {self.frame_id}")
        self.is_moving = True

    def look_left_cmd(self):
        self.twist_command.twist.linear.x = 0.0
        self.twist_command.twist.linear.y = 0.0
        self.twist_command.twist.linear.z = 0.0
        self.twist_command.twist.angular.x = 0.0
        self.twist_command.twist.angular.y = 0.0
        self.twist_command.twist.angular.z = self.angular_speed
        self.last_command = 'look_left'
        self.get_logger().info(f"Looking left (yaw+) in {self.frame_id}")
        self.is_moving = True

    def look_right_cmd(self):
        self.twist_command.twist.linear.x = 0.0
        self.twist_command.twist.linear.y = 0.0
        self.twist_command.twist.linear.z = 0.0
        self.twist_command.twist.angular.x = 0.0
        self.twist_command.twist.angular.y = 0.0
        self.twist_command.twist.angular.z = -self.angular_speed
        self.last_command = 'look_right'
        self.get_logger().info(f"Looking right (yaw-) in {self.frame_id}")
        self.is_moving = True

    def switch_frame_cmd(self):
        self.frame_index = (self.frame_index + 1) % len(self.frames)
        self.frame_id = self.frames[self.frame_index]
        self.twist_command.header.frame_id = self.frame_id
        self.get_logger().info(f"Switched to frame: {self.frame_id}")
        self.stop_movement()  # Stop motion during frame switch

    def stop_movement(self):
        self.twist_command.twist.linear.x = 0.0
        self.twist_command.twist.linear.y = 0.0
        self.twist_command.twist.linear.z = 0.0
        self.twist_command.twist.angular.x = 0.0
        self.twist_command.twist.angular.y = 0.0
        self.twist_command.twist.angular.z = 0.0
        self.last_command = 'stop'
        self.get_logger().info("Stopping")
        self.is_moving = False

    def adjust_speed(self, delta):
        new_linear_speed = self.linear_speed + delta
        new_angular_speed = self.angular_speed + (delta * 2.0)  # Scale angular delta
        if (self.min_linear_speed <= new_linear_speed <= self.max_linear_speed and
                self.min_angular_speed <= new_angular_speed <= self.max_angular_speed):
            self.linear_speed = new_linear_speed
            self.angular_speed = new_angular_speed
            self.get_logger().info(
                f"Speeds adjusted: linear={self.linear_speed} m/s, angular={self.angular_speed} rad/s"
            )
        else:
            self.get_logger().info(
                f"Speed adjustment out of bounds: linear={new_linear_speed}, angular={new_angular_speed}"
            )

    def publish_last_command(self):
        self.twist_command.header.stamp = self.get_clock().now().to_msg()
        self.twist_publisher.publish(self.twist_command)
        if self.is_moving and self.last_command != 'stop':
            self.get_logger().debug(
                f"Published TwistStamped: linear=[{self.twist_command.twist.linear.x}, "
                f"{self.twist_command.twist.linear.y}, {self.twist_command.twist.linear.z}], "
                f"angular=[{self.twist_command.twist.angular.x}, {self.twist_command.twist.angular.y}, "
                f"{self.twist_command.twist.angular.z}] in {self.frame_id}"
            )
        else:
            self.get_logger().debug("Published zero TwistStamped")

    def stop_on_timeout(self):
        if self.is_moving:
            self.get_logger().info("Timeout reached, stopping.")
            self.stop_movement()
            self.publish_last_command()

    def transcript_callback(self, msg):
        transcription = msg.data.lower()
        if not transcription.strip():
            self.get_logger().info("Ignoring empty transcription")
            return

        command_phrases = [
            phrase.strip().rstrip('.').rstrip(',').rstrip('!').rstrip('?')
            for phrase in transcription.split(',')
        ]

        for phrase in command_phrases:
            if phrase in self.move_up:
                self.move_up_cmd()
                self.reset_timeout_timer()
                break
            elif phrase in self.move_down:
                self.move_down_cmd()
                self.reset_timeout_timer()
                break
            elif phrase in self.move_left:
                self.move_left_cmd()
                self.reset_timeout_timer()
                break
            elif phrase in self.move_right:
                self.move_right_cmd()
                self.reset_timeout_timer()
                break
            elif phrase in self.look_up:
                self.look_up_cmd()
                self.reset_timeout_timer()
                break
            elif phrase in self.look_down:
                self.look_down_cmd()
                self.reset_timeout_timer()
                break
            elif phrase in self.look_left:
                self.look_left_cmd()
                self.reset_timeout_timer()
                break
            elif phrase in self.look_right:
                self.look_right_cmd()
                self.reset_timeout_timer()
                break
            elif phrase in self.switch_frame:
                self.switch_frame_cmd()
                self.reset_timeout_timer()
                break
            elif phrase in self.stop:
                self.stop_movement()
                self.reset_timeout_timer()
                break
            elif phrase in self.speed_up:
                self.adjust_speed(0.05)
                break
            elif phrase in self.slow_down:
                self.adjust_speed(-0.05)
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
        self.publish_last_command()
        time.sleep(0.1)
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
