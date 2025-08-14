#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <memory>

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        // Publisher for ODrive commands
        odrive_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/odrive/commands", 10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&RobotController::sendOdriveCommand, this));
            
        RCLCPP_INFO(this->get_logger(), "Robot Controller started for ROS2 Jazzy");
    }
    
private:
    void sendOdriveCommand()
    {
        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        static bool toggle = false;
        if (toggle) {
            cmd_msg.data = {0.5, 0.3};
        } else {
            cmd_msg.data = {-0.5, -0.3};
        }
        toggle = !toggle;

        odrive_cmd_pub_->publish(cmd_msg);

        RCLCPP_INFO(this->get_logger(), "ODrive command sent: [%.2f, %.2f]",
                    cmd_msg.data[0], cmd_msg.data[1]);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr odrive_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}