#ifndef PURE_PURSUIT_HPP_
#define PURE_PURSUIT_HPP_

#include "pure_pursuit_logic.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

#include "path_tracking/visibility_control.h"

namespace pure_pursuit
{

    class PurePursuitController
    {
    public:
        PurePursuitController();

        void setParams(double wb, double k_ld, double min_ld, double max_ld, double target_v, double goal_r);
        void setPath(const nav_msgs::msg::Path &path);
        bool update(const geometry_msgs::msg::Pose &current_pose_msg, double current_velocity,
                    double &out_linear_x_velocity, double &out_linear_y_velocity, double &out_angular_velocity);

        bool isPathSet() const { return algorithm_.isPathSet(); }
        bool hasReachedGoal() const { return algorithm_.hasReachedGoal(); }

    private:
        pure_pursuit_logic::PurePursuitAlgorithm algorithm_;
    };

    class PurePursuit : public rclcpp::Node
    {
    public:
        PATH_TRACKING_PUBLIC
        explicit PurePursuit(const rclcpp::NodeOptions &options);
        virtual ~PurePursuit();
        void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void controlLoop();
        void publishStopCommand();
        void publishVelocityCommand(double linear_x_velocity, double linear_y_velocity, double angular_velocity);

    private:
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        PurePursuitController controller_;

        geometry_msgs::msg::Pose current_pose_;
        double current_velocity_;
        bool path_received_;
        bool odom_received_;

        std::string odom_topic_;
        std::string path_topic_;
        std::string cmd_vel_topic_;
    };

} // namespace pure_pursuit

#endif // PURE_PURSUIT_HPP_
