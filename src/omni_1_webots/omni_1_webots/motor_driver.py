import rclpy
from geometry_msgs.msg import Twist

WHEEL_RADIUS = 0.04


class MotorDriver:
    def init(self, webots_node, properties):
        self.robot_ = webots_node.robot

        self.motor_0_= self.robot_.getDevice("wheel_0_joint")
        self.motor_120_ = self.robot_.getDevice("wheel_120_joint")
        self.motor_240_ = self.robot_.getDevice("wheel_240_joint")

        self.motor_0_.setPosition(float("inf"))
        self.motor_0_.setVelocity(0)
        self.motor_120_.setPosition(float("inf"))
        self.motor_120_.setVelocity(0)
        self.motor_240_.setPosition(float("inf"))
        self.motor_240_.setVelocity(0)

        self.target_twist_ = Twist()

        rclpy.init(args=None)
        self.node_ = rclpy.create_node("motor_driver")
        self.subscriber_ = self.node_.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 1
        )
        self.node_.get_logger().info("Webots motor driver is initialized.")

    def cmd_vel_callback(self, twist):
        self.target_twist_ = twist

    def step(self):
        rclpy.spin_once(self.node_, timeout_sec=0)

        motor_0_vel_ = self.target_twist_.angular.x
        motor_120_vel_ = self.target_twist_.angular.y
        motor_240_vel_ = self.target_twist_.angular.z

        self.motor_0_.setVelocity(motor_0_vel_)
        self.motor_120_.setVelocity(motor_120_vel_)
        self.motor_240_.setVelocity(motor_240_vel_)
