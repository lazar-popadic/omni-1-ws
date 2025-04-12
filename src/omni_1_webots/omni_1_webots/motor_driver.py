import rclpy
from omni_1_interfaces.msg import MotorCommandArray

WHEEL_RADIUS = 0.04


class MotorDriver:
    def init(self, webots_node, properties):
        self.robot_ = webots_node.robot

        self.motor_0_ = self.robot_.getDevice("wheel-frame0")
        self.motor_120_ = self.robot_.getDevice("wheel-frame120")
        self.motor_240_ = self.robot_.getDevice("wheel-frame240")

        self.motor_0_.setPosition(float("inf"))
        self.motor_0_.setVelocity(0)
        self.motor_120_.setPosition(float("inf"))
        self.motor_120_.setVelocity(0)
        self.motor_240_.setPosition(float("inf"))
        self.motor_240_.setVelocity(0)

        self.target_motor_cmd_ = MotorCommandArray()

        rclpy.init(args=None)
        self.node_ = rclpy.create_node("motor_driver")
        self.cmd_sub_ = self.node_.create_subscription(
            MotorCommandArray, "motor_cmd", self.cmd_vel_callback, 1
        )

        self.motor_0_vel_ = 0
        self.motor_120_vel_ = 0
        self.motor_240_vel_ = 0

        self.node_.get_logger().info("Webots motor driver is initialized.")

    def cmd_vel_callback(self, motor_cmd):
        if len(motor_cmd.motor_command) == 3:
            self.motor_0_vel_ = motor_cmd.motor_command[0]
            self.motor_120_vel_ = motor_cmd.motor_command[1]
            self.motor_240_vel_ = motor_cmd.motor_command[2]
        else:
            self.node_.get_logger().info("Recieved invalid motor command.")

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)

        self.motor_0_.setVelocity(self.motor_0_vel_)
        self.motor_120_.setVelocity(self.motor_120_vel_)
        self.motor_240_.setVelocity(self.motor_240_vel_)
