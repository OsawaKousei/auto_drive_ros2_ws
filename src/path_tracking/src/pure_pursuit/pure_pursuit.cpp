#include <rclcpp_components/register_node_macro.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "path_tracking/pure_pursuit/pure_pursuit_logic.hpp"
#include "path_tracking/pure_pursuit/pure_pursuit.hpp"

namespace pure_pursuit
{

    PurePursuitController::PurePursuitController()
    {
        // Initialization of algorithm_ is handled by its own constructor
    }

    void PurePursuitController::setParams(double k_ld, double min_ld, double max_ld, double target_v, double goal_r, double k_c, double k_v_red)
    {
        algorithm_.setParams(k_ld, min_ld, max_ld, target_v, goal_r, k_c, k_v_red);
    }

    void PurePursuitController::setPath(const nav_msgs::msg::Path &path_msg)
    {
        std::vector<path_tracking::Point> path_points;
        path_points.reserve(path_msg.poses.size());
        for (const auto &pose_stamped : path_msg.poses)
        {
            path_points.push_back({pose_stamped.pose.position.x, pose_stamped.pose.position.y});
        }
        algorithm_.setPath(path_points);
    }

    bool PurePursuitController::update(const geometry_msgs::msg::Pose &current_pose_msg, double current_velocity,
                                       double &out_linear_x_velocity, double &out_linear_y_velocity, double &out_angular_velocity)
    {
        path_tracking::Pose current_pose_logic;
        current_pose_logic.x = current_pose_msg.position.x;
        current_pose_logic.y = current_pose_msg.position.y;
        current_pose_logic.yaw = tf2::getYaw(current_pose_msg.orientation);

        return algorithm_.update(current_pose_logic, current_velocity, out_linear_x_velocity, out_linear_y_velocity, out_angular_velocity);
    }

    PurePursuit::PurePursuit(const rclcpp::NodeOptions &options) : rclcpp::Node("publish", options)
    {
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("path_topic", "/local_path");
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<double>("look_forward_gain", 0.1);
        this->declare_parameter<double>("min_look_ahead_distance", 0.5);
        this->declare_parameter<double>("max_look_ahead_distance", 0.7);
        this->declare_parameter<double>("target_velocity", 1.0);    // m/s
        this->declare_parameter<double>("goal_radius", 0.5);        // Added goal_radius parameter
        this->declare_parameter<double>("k_c", 4.0);                // Control gain for curvature
        this->declare_parameter<double>("k_v_red", 4.0);            // Control gain for velocity reduction
        this->declare_parameter<double>("control_loop_rate", 20.0); // Hz

        odom_topic_ = this->get_parameter("odom_topic").as_string();
        path_topic_ = this->get_parameter("path_topic").as_string();
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        double k_ld = this->get_parameter("look_forward_gain").as_double();
        double min_ld = this->get_parameter("min_look_ahead_distance").as_double();
        double max_ld = this->get_parameter("max_look_ahead_distance").as_double();
        double target_v = this->get_parameter("target_velocity").as_double();
        double goal_r = this->get_parameter("goal_radius").as_double(); // Use goal_radius
        double k_c = this->get_parameter("k_c").as_double();
        double k_v_red = this->get_parameter("k_v_red").as_double();
        double control_loop_rate = this->get_parameter("control_loop_rate").as_double();

        controller_.setParams(k_ld, min_ld, max_ld, target_v, goal_r, k_c, k_v_red);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_, 10, std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&PurePursuit::odomCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / control_loop_rate),
            std::bind(&PurePursuit::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node started.");
    }

    PurePursuit::~PurePursuit()
    {
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node shutting down.");
    }

    void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received an empty path. Ignoring.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu points.", msg->poses.size());
        controller_.setPath(*msg);
        path_received_ = true;
        odom_received_ = false; // Reset odom flag to wait for new odom after new path
    }

    void PurePursuit::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        current_velocity_ = msg->twist.twist.linear.x;
        odom_received_ = true;
    }

    void PurePursuit::controlLoop()
    {
        if (!path_received_ || !odom_received_ || !controller_.isPathSet())
        {
            if (!path_received_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for path...");
            }
            else if (!odom_received_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for odometry...");
            }
            // Publish zero velocity if no path or odom, or path not set in controller
            publishStopCommand();
            return;
        }

        if (controller_.hasReachedGoal())
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Goal reached. Stopping.");
            publishStopCommand();
            return;
        }

        double linear_x_velocity = 0.0;
        double linear_y_velocity = 0.0;
        double angular_velocity = 0.0;

        bool success = controller_.update(current_pose_, current_velocity_, linear_x_velocity, linear_y_velocity, angular_velocity);

        geometry_msgs::msg::Twist cmd_vel_msg;
        if (success)
        {
            cmd_vel_msg.linear.x = linear_x_velocity;
            cmd_vel_msg.linear.y = linear_y_velocity;
            cmd_vel_msg.angular.z = angular_velocity;
        }
        else
        {
            // Controller indicated to stop (e.g., end of path reached or error)
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.linear.y = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            if (!controller_.hasReachedGoal())
            { // Avoid spamming if goal is already logged as reached
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Controller advises to stop or path end.");
            }
        }
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    void PurePursuit::publishStopCommand()
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    // Add the definition for publishVelocityCommand
    void PurePursuit::publishVelocityCommand(double linear_x_velocity, double linear_y_velocity, double angular_velocity)
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_x_velocity;
        cmd_vel_msg.linear.y = linear_y_velocity;
        cmd_vel_msg.angular.z = angular_velocity;
        if (cmd_vel_pub_)
        { // Ensure the publisher is initialized
            cmd_vel_pub_->publish(cmd_vel_msg);
        }
        else
        {
            // It's good practice to log if the publisher isn't ready, though in this class structure,
            // it should be initialized in the constructor.
            RCLCPP_WARN(this->get_logger(), "cmd_vel_pub_ not initialized in publishVelocityCommand.");
        }
    }
} // namespace pure_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(pure_pursuit::PurePursuit)