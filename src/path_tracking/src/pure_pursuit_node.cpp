#include "rclcpp/rclcpp.hpp"
#include "path_tracking/pure_pursuit.hpp" // 修正: "src/path_tracking/include/" プレフィックスを削除

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    const auto pure_pursuit_node = std::make_shared<pure_pursuit::PurePursuit>(rclcpp::NodeOptions());
    exec.add_node(pure_pursuit_node);

    exec.spin();
    rclcpp::shutdown();
    return 0; // Added return 0;
}