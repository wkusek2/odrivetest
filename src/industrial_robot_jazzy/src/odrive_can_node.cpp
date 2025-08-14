#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

class OdriveCANNode : public rclcpp::Node
{
public:
  OdriveCANNode() : Node("odrive_can_node")
  {
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 10);
    subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      "can_rx", 10,
      std::bind(&OdriveCANNode::on_frame, this, std::placeholders::_1));
  }

private:
  void on_frame(const can_msgs::msg::Frame::SharedPtr msg)
  {
    (void)msg;
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdriveCANNode>());
  rclcpp::shutdown();
  return 0;
}
