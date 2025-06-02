#ifndef PURE_PURSUIT_LOGIC_HPP_
#define PURE_PURSUIT_LOGIC_HPP_

#include <vector>
#include <cmath>
#include <string>
#include <algorithm> // For std::max, std::min

#include "path_tracking/common.hpp"
#include "path_tracking/visibility_control.h"

namespace pure_pursuit_logic
{
    class PurePursuitAlgorithm
    {
    public:
        PurePursuitAlgorithm();

        void setParams(double k_ld, double min_ld, double max_ld, double target_v, double goal_r);
        void setPath(const std::vector<path_tracking::Point> &path_points);
        bool update(const path_tracking::Pose &current_pose, double current_velocity,
                    double &out_linear_x_velocity, double &out_linear_y_velocity, double &out_angular_velocity);
        bool isPathSet() const { return !current_path_logic_.empty(); }
        bool hasReachedGoal() const { return has_reached_goal_; }

    private:
        double look_forward_gain_; // k_ld
        double min_look_ahead_distance_;
        double max_look_ahead_distance_;
        double target_velocity_;
        double goal_radius_;

        std::vector<path_tracking::Point> current_path_logic_;
        int last_found_target_idx_;
        double current_look_ahead_distance_;
        bool has_reached_goal_;

        int searchTargetIndex(const path_tracking::Pose &current_pose, double current_velocity);
        double normalizeAngle(double angle);
    };

} // namespace pure_pursuit_logic

#endif // PURE_PURSUIT_LOGIC_HPP_
