import rclpy
from omni_1_interfaces.msg import MotorCommandArray

WHEEL_RADIUS = 0.04


class MotorDriver:
    def init(self, webots_node, properties):
        self.robot_ = webots_node.robot

        self.motor_60_ = self.robot_.getDevice("wheel-frame60")
        self.motor_180_ = self.robot_.getDevice("wheel-frame180")
        self.motor_300_ = self.robot_.getDevice("wheel-frame300")

        self.motor_60_.setPosition(float("inf"))
        self.motor_60_.setVelocity(0)
        self.motor_180_.setPosition(float("inf"))
        self.motor_180_.setVelocity(0)
        self.motor_300_.setPosition(float("inf"))
        self.motor_300_.setVelocity(0)

        self.target_motor_cmd_ = MotorCommandArray()

        rclpy.init(args=None)
        self.node_ = rclpy.create_node("motor_driver")
        self.cmd_sub_ = self.node_.create_subscription(
            MotorCommandArray, "motor_cmd", self.cmd_vel_callback, 1
        )

        self.motor_60_vel_ = 0
        self.motor_180_vel_ = 0
        self.motor_300_vel_ = 0

        self.node_.get_logger().info("Webots motor driver is initialized.")

    def cmd_vel_callback(self, motor_cmd):
        if len(motor_cmd.motor_command) == 3:
            self.motor_60_vel_ = motor_cmd.motor_command[0]
            self.motor_180_vel_ = motor_cmd.motor_command[1]
            self.motor_300_vel_ = motor_cmd.motor_command[2]
        else:
            self.node_.get_logger().info("Recieved invalid motor command.")

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)

        self.motor_60_.setVelocity(self.motor_60_vel_)
        self.motor_180_.setVelocity(self.motor_180_vel_)
        self.motor_300_.setVelocity(self.motor_300_vel_)
