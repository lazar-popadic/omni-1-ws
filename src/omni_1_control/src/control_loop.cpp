#include "omni_1_interfaces/msg/motor_command_array.hpp"
#include "rclcpp/rclcpp.hpp"

class ControlLoopNode : public rclcpp::Node
{
public:
  ControlLoopNode () : Node ("control_loop")
  {
    this->declare_parameter ("freq_hz", 10.0);
    freq_hz_ = this->get_parameter ("freq_hz").as_double ();
    period_us_ = 1000000 / freq_hz_;

    // TODO:
    // - uzima mete: action
    timer_ = this->create_wall_timer (
        std::chrono::microseconds (period_us_),
        std::bind (&ControlLoopNode::control_loop, this));
    motor_cmd_publisher_
        = this->create_publisher<omni_1_interfaces::msg::MotorCommandArray> (
            "motor_cmd", 10);

    RCLCPP_INFO (this->get_logger (), "Control loop node is running.");
  }

private:
  double freq_hz_;
  int64_t period_us_;
  double x_ref, y_ref, phi_ref;
  double w0 = 0, w120 = 0, w240 = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<omni_1_interfaces::msg::MotorCommandArray>::SharedPtr
      motor_cmd_publisher_;

  void
  control_loop ()
  {
    // input:   reference pose            (x, y, φ)

    // output:  reference motor commands  (ω0, ω120, ω240)
    this->publish_motor_cmd ();
  }

  void
  publish_motor_cmd ()
  {
    auto msg = omni_1_interfaces::msg::MotorCommandArray ();
    msg.motor_command.resize (3);
    msg.motor_command[0] = w0;
    msg.motor_command[1] = w120;
    msg.motor_command[2] = w240;
    motor_cmd_publisher_->publish (msg);
  }
};

int
main (int argc, char **argv)
{
  rclcpp::init (argc, argv);
  auto node = std::make_shared<ControlLoopNode> ();
  rclcpp::spin (node);
  rclcpp::shutdown ();
  return 0;
}