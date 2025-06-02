#ifndef PURE_PURSUIT_LOGIC_HPP_
#define PURE_PURSUIT_LOGIC_HPP_

#include <vector>
#include <cmath>
#include <string>
#include <algorithm> // For std::max, std::min

namespace pure_pursuit_logic
{

    struct Point
    {
        double x;
        double y;
    };

    struct Pose
    {
        double x;
        double y;
        double yaw;
    };

    class PurePursuitAlgorithm
    {
    public:
        PurePursuitAlgorithm();

        void setParams(double wb, double k_ld, double min_ld, double max_ld, double target_v, double goal_r);
        void setPath(const std::vector<Point> &path_points);
        bool update(const Pose &current_pose, double current_velocity,
                    double &out_linear_velocity, double &out_angular_velocity);
        bool isPathSet() const { return !current_path_logic_.empty(); }
        bool hasReachedGoal() const { return has_reached_goal_; }

    private:
        double wheel_base_;
        double look_forward_gain_; // k_ld
        double min_look_ahead_distance_;
        double max_look_ahead_distance_;
        double target_velocity_;
        double goal_radius_;

        std::vector<Point> current_path_logic_;
        int last_found_target_idx_;
        double current_look_ahead_distance_;
        bool has_reached_goal_;

        int searchTargetIndex(const Pose &current_pose, double current_velocity);
        double normalizeAngle(double angle);
    };

} // namespace pure_pursuit_logic

#endif // PURE_PURSUIT_LOGIC_HPP_
