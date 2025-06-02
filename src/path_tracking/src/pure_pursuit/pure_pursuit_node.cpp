#include "rclcpp/rclcpp.hpp"
#include "path_tracking/pure_pursuit/pure_pursuit.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    const auto pure_pursuit_node = std::make_shared<pure_pursuit::PurePursuit>(rclcpp::NodeOptions());
    exec.add_node(pure_pursuit_node);

    exec.spin();
    rclcpp::shutdown();
}