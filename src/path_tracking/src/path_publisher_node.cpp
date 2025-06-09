#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class PathPublisherNode : public rclcpp::Node
{
public:
    PathPublisherNode()
        : Node("path_Publisher_node")
    {
        this->declare_parameter<std::string>("csv_file_path",
                                             ament_index_cpp::get_package_share_directory("path_tracking") + "/path/global_path.csv");
        this->declare_parameter<std::string>("frame_id", "map");

        csv_file_path_ = this->get_parameter("csv_file_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 10);

        path_publish_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "path_publish_flag", 10,
            std::bind(&PathPublisherNode::path_publish_flag_callback, this, std::placeholders::_1));

        if (!load_path_from_csv())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path from CSV. Node will not publish.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Path Publisher node started. Publishing path from: %s", csv_file_path_.c_str());
    }

private:
    bool load_path_from_csv()
    {
        std::ifstream file(csv_file_path_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open CSV file: %s", csv_file_path_.c_str());
            return false;
        }

        path_msg_.header.frame_id = frame_id_;
        std::string line;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string x_str, y_str;
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ','))
            {
                try
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = frame_id_;
                    // pose.header.stamp = this->now(); // Will be set at publish time
                    pose.pose.position.x = std::stod(x_str);
                    pose.pose.position.y = std::stod(y_str);
                    pose.pose.position.z = 0.0;    // Assuming 2D path
                    pose.pose.orientation.w = 1.0; // Neutral orientation
                    path_msg_.poses.push_back(pose);
                }
                catch (const std::invalid_argument &ia)
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid number format in CSV line: %s", line.c_str());
                }
                catch (const std::out_of_range &oor)
                {
                    RCLCPP_WARN(this->get_logger(), "Number out of range in CSV line: %s", line.c_str());
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Malformed CSV line: %s", line.c_str());
            }
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", path_msg_.poses.size(), csv_file_path_.c_str());
        return !path_msg_.poses.empty();
    }

    void path_publish_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        publish_path();
        RCLCPP_INFO(this->get_logger(), "Path published with %zu poses.", path_msg_.poses.size());
    }

    void publish_path()
    {
        if (path_msg_.poses.empty())
        {
            return;
        }

        path_msg_.header.stamp = this->now();
        for (auto &pose : path_msg_.poses)
        {
            pose.header.stamp = path_msg_.header.stamp; // Update timestamp for each pose
        }
        path_publisher_->publish(path_msg_);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_publish_flag_subscriber_;
    nav_msgs::msg::Path path_msg_;
    std::string csv_file_path_;
    std::string frame_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
