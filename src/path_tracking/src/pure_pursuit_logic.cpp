#include "path_tracking/pure_pursuit_logic.hpp" // 修正: "path_tracking/" プレフィックスを削除
#include <limits>                               // Not strictly needed now, but good for general math algorithms

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace pure_pursuit_logic
{

    PurePursuitAlgorithm::PurePursuitAlgorithm()
        : wheel_base_(2.0),
          look_forward_gain_(0.3),
          min_look_ahead_distance_(2.0),
          max_look_ahead_distance_(10.0),
          target_velocity_(1.0),
          goal_radius_(0.5),
          last_found_target_idx_(-1),
          current_look_ahead_distance_(2.0),
          has_reached_goal_(false)
    {
    }

    void PurePursuitAlgorithm::setParams(double wb, double k_ld, double min_ld, double max_ld, double target_v, double goal_r)
    {
        wheel_base_ = wb;
        look_forward_gain_ = k_ld;
        min_look_ahead_distance_ = min_ld;
        max_look_ahead_distance_ = max_ld;
        target_velocity_ = target_v;
        goal_radius_ = goal_r;
    }

    void PurePursuitAlgorithm::setPath(const std::vector<Point> &path_points)
    {
        current_path_logic_ = path_points;
        last_found_target_idx_ = -1;
        has_reached_goal_ = false;
        if (!current_path_logic_.empty())
        {
            last_found_target_idx_ = 0;
        }
    }

    double PurePursuitAlgorithm::normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    int PurePursuitAlgorithm::searchTargetIndex(const Pose &current_pose, double current_velocity)
    {
        if (current_path_logic_.empty())
        {
            return -1;
        }

        current_look_ahead_distance_ = look_forward_gain_ * std::abs(current_velocity) + min_look_ahead_distance_;
        current_look_ahead_distance_ = std::max(min_look_ahead_distance_, std::min(current_look_ahead_distance_, max_look_ahead_distance_));

        int N = current_path_logic_.size();
        int search_start_idx = (last_found_target_idx_ >= 0) ? last_found_target_idx_ : 0;

        for (int i = search_start_idx; i < N; ++i)
        {
            double dx = current_path_logic_[i].x - current_pose.x;
            double dy = current_path_logic_[i].y - current_pose.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist >= current_look_ahead_distance_)
            {
                last_found_target_idx_ = i;
                return i;
            }
        }

        if (N > 0)
        {
            last_found_target_idx_ = N - 1;
            return N - 1;
        }

        return -1;
    }

    bool PurePursuitAlgorithm::update(const Pose &current_pose, double current_velocity,
                                      double &out_linear_velocity, double &out_angular_velocity)
    {
        if (current_path_logic_.empty() || has_reached_goal_)
        {
            out_linear_velocity = 0.0;
            out_angular_velocity = 0.0;
            return false;
        }

        int target_idx = searchTargetIndex(current_pose, current_velocity);

        if (target_idx < 0)
        {
            out_linear_velocity = 0.0;
            out_angular_velocity = 0.0;
            return false;
        }

        const auto &target_point = current_path_logic_[target_idx];

        double dx_global = target_point.x - current_pose.x;
        double dy_global = target_point.y - current_pose.y;

        double alpha = normalizeAngle(std::atan2(dy_global, dx_global) - current_pose.yaw);

        double steering_angle = std::atan2(2.0 * wheel_base_ * std::sin(alpha), current_look_ahead_distance_);

        out_angular_velocity = current_velocity * std::tan(steering_angle) / wheel_base_;
        // Ensure angular velocity is not NaN or Inf if current_velocity is very small and steering_angle is large.
        // This can happen if look_ahead_distance is very small.
        // However, current_look_ahead_distance_ has a minimum.
        if (std::isnan(out_angular_velocity) || std::isinf(out_angular_velocity))
        {
            out_angular_velocity = 0.0; // Or some other safe value
        }

        out_linear_velocity = target_velocity_;

        // Check if the vehicle is near the final goal of the path
        // This check should be against the actual last point of the path if target_idx is the last point.
        if (static_cast<size_t>(target_idx) == current_path_logic_.size() - 1)
        {
            double dx_to_actual_end = current_path_logic_.back().x - current_pose.x;
            double dy_to_actual_end = current_path_logic_.back().y - current_pose.y;
            // Using squared distance to avoid sqrt until necessary
            double dist_sq_to_final_goal = dx_to_actual_end * dx_to_actual_end + dy_to_actual_end * dy_to_actual_end;
            if (std::sqrt(dist_sq_to_final_goal) < goal_radius_)
            {
                has_reached_goal_ = true;
                out_linear_velocity = 0.0;
                out_angular_velocity = 0.0;
                return false; // Indicate finished
            }
        }
        return true;
    }

} // namespace pure_pursuit_logic
