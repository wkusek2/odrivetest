#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <thread>
#include <atomic>

class ODriveCANNode : public rclcpp::Node
{
public:
    ODriveCANNode() : Node("odrive_can_node")
    {
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<int>("axis_id", 0);
        can_interface_ = this->get_parameter("can_interface").as_string();
        axis_id_ = this->get_parameter("axis_id").as_int();

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "odrive/joint_state", 10);

        position_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "odrive/command/position", 10,
            std::bind(&ODriveCANNode::positionCallback, this, std::placeholders::_1));
        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "odrive/command/velocity", 10,
            std::bind(&ODriveCANNode::velocityCallback, this, std::placeholders::_1));
        current_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "odrive/command/current", 10,
            std::bind(&ODriveCANNode::currentCallback, this, std::placeholders::_1));

        if (!openSocket()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface %s", can_interface_.c_str());
            return;
        }

        running_ = true;
        read_thread_ = std::thread(&ODriveCANNode::readLoop, this);

        request_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ODriveCANNode::requestData, this));

        RCLCPP_INFO(this->get_logger(), "ODrive CAN node started on %s", can_interface_.c_str());
    }

    ~ODriveCANNode()
    {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

private:
    bool openSocket()
    {
        struct ifreq ifr;
        struct sockaddr_can addr;

        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            return false;
        }

        std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            return false;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            return false;
        }

        return true;
    }

    uint32_t makeId(uint32_t cmd) const
    {
        return (cmd << 5) | static_cast<uint32_t>(axis_id_);
    }

    void requestData()
    {
        sendRemote(makeId(0x09)); // Get_Encoder_Estimates
        sendRemote(makeId(0x10)); // Get_IQ
    }

    void sendRemote(uint32_t id)
    {
        if (socket_fd_ < 0) {
            return;
        }

        struct can_frame frame{};
        frame.can_id = id | CAN_RTR_FLAG;
        frame.can_dlc = 8;
        (void)write(socket_fd_, &frame, sizeof(frame));
    }

    void sendCommand(uint32_t cmd, double value)
    {
        if (socket_fd_ < 0) {
            return;
        }

        struct can_frame frame{};
        frame.can_id = makeId(cmd);
        frame.can_dlc = 8;
        float val = static_cast<float>(value);
        std::memcpy(frame.data, &val, sizeof(float));
        (void)write(socket_fd_, &frame, sizeof(frame));
    }

    void positionCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        sendCommand(0x0C, msg->data); // Set_Input_Position
    }

    void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        sendCommand(0x0D, msg->data); // Set_Input_Velocity
    }

    void currentCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        sendCommand(0x0E, msg->data); // Set_Input_Torque
    }

    void readLoop()
    {
        while (rclcpp::ok() && running_) {
            struct can_frame frame;
            int nbytes = read(socket_fd_, &frame, sizeof(frame));
            if (nbytes > 0) {
                handleFrame(frame);
            }
        }
    }

    void handleFrame(const struct can_frame &frame)
    {
        uint32_t cmd = frame.can_id >> 5;
        uint32_t node_id = frame.can_id & 0x1F;
        if (node_id != static_cast<uint32_t>(axis_id_)) {
            return;
        }

        if (cmd == 0x09 && frame.can_dlc >= 8) {
            float pos, vel;
            std::memcpy(&pos, frame.data, 4);
            std::memcpy(&vel, frame.data + 4, 4);
            last_pos_ = pos;
            last_vel_ = vel;
            publishState();
        } else if (cmd == 0x10 && frame.can_dlc >= 4) {
            std::memcpy(&last_current_, frame.data, 4);
        }
    }

    void publishState()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name.push_back("odrive_joint");
        msg.position.push_back(static_cast<double>(last_pos_));
        msg.velocity.push_back(static_cast<double>(last_vel_));
        msg.effort.push_back(static_cast<double>(last_current_));
        joint_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_sub_;
    rclcpp::TimerBase::SharedPtr request_timer_;

    std::thread read_thread_;
    std::atomic<bool> running_{false};

    int socket_fd_{-1};
    std::string can_interface_;
    int axis_id_{0};

    float last_pos_{0.0f};
    float last_vel_{0.0f};
    float last_current_{0.0f};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ODriveCANNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

