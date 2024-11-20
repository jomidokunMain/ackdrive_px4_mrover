#include "include/ackdrive_px4_mrover/mrover_comms.hpp"

MroverComms::MroverComms()
    : Node("mrover_comms"),
      encoder_val_1_(0.0),
      encoder_val_2_(0.0),
      encoders_updated_(false)
{
    // Declare parameters for topics
    auto encoder_topic = this->declare_parameter<std::string>("encoder_topic", "encoder_values");
    auto pid_topic = this->declare_parameter<std::string>("pid_topic", "pid_values");

    // Initialize publisher for PID values
    pid_pub_ = this->create_publisher<std_msgs::msg::String>(pid_topic, 10);

    // Initialize subscriber for encoder values
    encoder_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        encoder_topic, 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 2) {
                std::lock_guard<std::mutex> lock(encoder_mutex_);
                encoder_val_1_ = msg->data[0];
                encoder_val_2_ = msg->data[1];
                encoders_updated_ = true;
                RCLCPP_INFO(this->get_logger(), "Received encoder values: %f, %f", encoder_val_1_, encoder_val_2_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Encoder topic received insufficient data!");
            }
        });

    // Create a timer to periodically check the encoder values (every 1 second)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { this->periodic_check(); });

    RCLCPP_INFO(this->get_logger(), "MroverComms initialized.");
}

void MroverComms::connect(int32_t timeout_ms)
{
    timeout_ms_ = timeout_ms;
    RCLCPP_INFO(this->get_logger(), "Connected to ROS2 topics for car-like bot control.");
}

void MroverComms::disconnect()
{
    RCLCPP_INFO(this->get_logger(), "Disconnected from ROS2 topics.");
}

bool MroverComms::connected() const
{
    RCLCPP_INFO(this->get_logger(), "ROS 2 communication is always active.");
    return true; // Always connected in ROS 2 context
}

void MroverComms::send_empty_msg()
{
    RCLCPP_WARN(this->get_logger(), "Empty message functionality is deprecated in ROS 2 communication.");
}

void MroverComms::read_encoder_values(double &val_1, double &val_2)
{
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    if (encoders_updated_) {
        val_1 = encoder_val_1_;
        val_2 = encoder_val_2_;
        RCLCPP_INFO(this->get_logger(), "Read encoder values: %f, %f", val_1, val_2);
        encoders_updated_ = false; // Reset flag after reading
    } else {
        val_1 = encoder_val_1_;
        val_2 = encoder_val_1_;
        RCLCPP_WARN(this->get_logger(), "Encoder values have not been updated yet!");
    }
}

void MroverComms::periodic_check()
{
    double val_1, val_2;
    read_encoder_values(val_1, val_2);
    RCLCPP_INFO(this->get_logger(), "Periodic check: %f, %f", val_1, val_2);
}

void MroverComms::set_motor_values(float val_1, float val_2)
{
    std_msgs::msg::Float32MultiArray motor_msg;
    motor_msg.data.push_back(val_1);
    motor_msg.data.push_back(val_2);
    RCLCPP_INFO(this->get_logger(), "Set motor values: %f, %f", val_1, val_2);
}

void MroverComms::set_pid_values(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << k_p << ":" << k_d << ":" << k_i << ":" << k_o;
    auto pid_msg = std_msgs::msg::String();
    pid_msg.data = ss.str();
    pid_pub_->publish(pid_msg);
    RCLCPP_INFO(this->get_logger(), "Set PID values: %s", ss.str().c_str());
}
