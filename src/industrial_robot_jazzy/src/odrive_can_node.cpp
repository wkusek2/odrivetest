#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <utility>
#include <chrono>

class ODriveCANNode : public rclcpp::Node
{
public:
    ODriveCANNode() : Node("odrive_can_node")
    {
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ODriveCANNode::publishJointState, this));
    }

private:
    std::pair<double, double> readAxisState(std::size_t axis)
    {
        // TODO: Replace with actual ODrive CAN communication
        (void)axis;
        return {0.0, 0.0};
    }

    void publishJointState()
    {
        auto [pos1, vel1] = readAxisState(0);
        auto [pos2, vel2] = readAxisState(1);

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        msg.name = {"joint1", "joint2"};
        msg.position = {pos1, pos2};
        msg.velocity = {vel1, vel2};
        joint_state_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODriveCANNode>());
    rclcpp::shutdown();
    return 0;
}
