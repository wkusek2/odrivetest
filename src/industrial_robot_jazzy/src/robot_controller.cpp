#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>
#include <memory>
#include <cmath>

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        // Publishers
        position_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
            
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/trajectory_controller/joint_trajectory", 10);
            
        // Subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            std::bind(&RobotController::jointStateCallback, this, std::placeholders::_1));
            
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&RobotController::sendTrajectory, this));
            
        RCLCPP_INFO(this->get_logger(), "Robot Controller started for ROS2 Jazzy");
    }
    
private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_positions_ = msg->position;
        // RCLCPP_INFO(this->get_logger(), "Current joint positions received");
    }
    
    void sendSimplePositionCommand()
    {
        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        static double angle = 0.0;
        angle += 0.1;
        if (angle > 3.14) angle = -3.14;
        
        cmd_msg.data = {angle, std::sin(angle * 0.5)};
        position_cmd_pub_->publish(cmd_msg);
        
        RCLCPP_INFO(this->get_logger(), "Position command sent: [%.2f, %.2f]", angle, std::sin(angle * 0.5));
    }
    
    void sendTrajectory()
    {
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        trajectory_msg.header.stamp = this->now();
        trajectory_msg.joint_names = {"joint1", "joint2"};
        
        // Punkt 1
        trajectory_msgs::msg::JointTrajectoryPoint point1;
        point1.positions = {0.5, 0.3};
        point1.velocities = {0.0, 0.0};
        point1.time_from_start = rclcpp::Duration::from_seconds(2.0);
        
        // Punkt 2
        trajectory_msgs::msg::JointTrajectoryPoint point2;
        point2.positions = {-0.5, -0.3};
        point2.velocities = {0.0, 0.0};
        point2.time_from_start = rclcpp::Duration::from_seconds(4.0);
        
        trajectory_msg.points = {point1, point2};
        trajectory_pub_->publish(trajectory_msg);
        
        RCLCPP_INFO(this->get_logger(), "Trajectory sent to robot");
    }
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_cmd_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> current_positions_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}