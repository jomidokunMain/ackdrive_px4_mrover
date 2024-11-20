#ifndef CARLIKE_ROS_COMMS_HPP
#define CARLIKE_ROS_COMMS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <sstream>

class MroverComms : public rclcpp::Node
{
public:
    MroverComms()
        : Node("mrover_comms"),
          encoder_val_1_(0.0),
          encoder_val_2_(0.0),
          encoders_updated_(false)
    {
        // Declare parameters for topics
        auto encoder_topic = this->declare_parameter<std::string>("encoder_topic", "encoder_values");
        auto pid_topic = this->declare_parameter<std::string>("pid_topic", "pid_values");

        // Initialize publisher for PID values
        pid_pub_ = this->create_publisher<std_msgs::msg::String>(pid_topic, 30);

        // Initialize publisher for motor values
        motor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_values", 30);

        // Initialize subscriber for encoder values
        encoder_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            encoder_topic, 30,
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

        RCLCPP_INFO(this->get_logger(), "MroverComms initialized.");
    }

    void connect(int32_t timeout_ms)
    {  
        timeout_ms_ = timeout_ms;
        RCLCPP_INFO(this->get_logger(), "Connected to ROS2 topics for car-like bot control.");
    }

    void disconnect()
    {
        RCLCPP_INFO(this->get_logger(), "Disconnected from ROS2 topics.");
    }

    bool connected() const
    {
        RCLCPP_INFO(this->get_logger(), "ROS 2 communication is always active.");
        return true; // Always connected in ROS 2 context
    }

    void send_empty_msg()
    {
        RCLCPP_WARN(this->get_logger(), "Empty message functionality is deprecated in ROS 2 communication.");
    }

    void read_encoder_values(double &val_1, double &val_2)
    {
        std::lock_guard<std::mutex> lock(encoder_mutex_);
        if (encoders_updated_) {
            val_1 = encoder_val_1_;
            val_2 = encoder_val_2_;
            RCLCPP_INFO(this->get_logger(), "Read encoder values: %f, %f", val_1, val_2);
            encoders_updated_ = false; // Reset flag after reading
        } else {
            // Provide the last known values
            val_1 = encoder_val_1_;
            val_2 = encoder_val_2_;
            RCLCPP_WARN(this->get_logger(), "Encoder values have not been updated yet!");
        }
    }

    void set_motor_values(double val_1, double val_2)
    {
        std_msgs::msg::Float32MultiArray motor_msg;
        motor_msg.data.push_back(val_1);
        motor_msg.data.push_back(val_2);
        motor_pub_->publish(motor_msg);
        RCLCPP_INFO(this->get_logger(), "Set motor values: %f, %f", val_1, val_2);
    }

    void set_pid_values(float k_p, float k_d, float k_i, float k_o)
    {
        std::stringstream ss;
        ss << k_p << ":" << k_d << ":" << k_i << ":" << k_o;
        auto pid_msg = std_msgs::msg::String();
        pid_msg.data = ss.str();
        pid_pub_->publish(pid_msg);
        RCLCPP_INFO(this->get_logger(), "Set PID values: %s", ss.str().c_str());
    }

private:
    // ROS 2 publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pid_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr encoder_sub_;

    // Encoder values updated from the subscribed topic
    double encoder_val_1_;
    double encoder_val_2_;
    bool encoders_updated_;

    // Mutex to ensure thread-safe access to encoder values
    std::mutex encoder_mutex_;

    // Placeholder timeout variable
    int timeout_ms_;
};

#endif // CARLIKE_ROS_COMMS_HPP