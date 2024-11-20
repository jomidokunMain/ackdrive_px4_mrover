#include "include/ackdrive_px4_mrover/mrover_comms.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MroverComms>();

    node->connect(1000);  // Simulated timeout

    rclcpp::spin(node);

    node->disconnect();

    rclcpp::shutdown();
    return 0;
}
