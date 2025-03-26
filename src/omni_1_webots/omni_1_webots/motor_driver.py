import rclpy
from geometry_msgs.msg import Twist
from example_interfaces.msg import String

WHEEL_RADIUS = 0.04


class MotorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__motor_0 = self.__robot.getDevice("wheel_0_joint")
        self.__motor_120 = self.__robot.getDevice("wheel_120_joint")
        self.__motor_240 = self.__robot.getDevice("wheel_240_joint")

        self.__motor_0.setPosition(float("inf"))
        self.__motor_0.setVelocity(0)
        self.__motor_120.setPosition(float("inf"))
        self.__motor_120.setVelocity(0)
        self.__motor_240.setPosition(float("inf"))
        self.__motor_240.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node("motor_driver")
        self.__subscriber = self.__node.create_subscription(
            Twist, "cmd_vel", self.__cmd_vel_callback, 1
        )
        self.__publisher = self.__node.create_publisher(String, "motor_driver_pub", 1)
        self.timer_ = self.__node.create_timer(1, self.publish_news)

        self.__node.get_logger().info("Webots motor driver is initialized.")

    def publish_news(self):
        msg = String()
        msg.data = "Webots motor driver news."
        self.__publisher.publish(msg)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        __motor_0_vel = self.__target_twist.angular.x
        __motor_120_vel = self.__target_twist.angular.y
        __motor_240_vel = self.__target_twist.angular.z

        self.__motor_0.setVelocity(__motor_0_vel)
        self.__motor_120.setVelocity(__motor_120_vel)
        self.__motor_240.setVelocity(__motor_240_vel)
