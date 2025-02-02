#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

using std::placeholders::_1;

class ControlCmdSwitcher : public rclcpp::Node
{
public:
  ControlCmdSwitcher()
  : Node("control_cmd_switcher"), current_mode_(1)
  {
    lidar_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd/lidar",
      rclcpp::QoS(1),
      std::bind(&ControlCmdSwitcher::lidar_callback, this, _1)
    );

    gnss_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd/gnss",
      rclcpp::QoS(1),
      std::bind(&ControlCmdSwitcher::gnss_callback, this, _1)
    );

    mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/aw/mode",
      rclcpp::QoS(10),
      std::bind(&ControlCmdSwitcher::mode_callback, this, _1)
    );

    cmd_pub_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd",
      rclcpp::QoS(1)
    );

    mode_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/aw/cur_mode",
      rclcpp::QoS(10)
    );

    RCLCPP_INFO(this->get_logger(), "ControlCmdSwitcher 노드가 시작되었습니다.");
  }

private:
  void lidar_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    lidar_cmd_ = msg;
    if (current_mode_ == 1) {
      cmd_pub_->publish(*msg);
    }
  }

  void gnss_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    gnss_cmd_ = msg;
    if (current_mode_ == 2) {
      cmd_pub_->publish(*msg);
    }
  }

  void mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int new_mode = msg->data;
    if (current_mode_ != new_mode) {
      RCLCPP_INFO(this->get_logger(), "모드 변경: %d -> %d", current_mode_, new_mode);
      current_mode_ = new_mode;

      std_msgs::msg::Int32 mode_msg;
      mode_msg.data = new_mode;
      mode_pub_->publish(mode_msg);
    }
  }

  int current_mode_;
  autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr lidar_cmd_;
  autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr gnss_cmd_;

  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr lidar_sub_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr gnss_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlCmdSwitcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
