#include <limits>

#include "path_tracking/pure_pursuit/pure_pursuit_logic.hpp"
#include "path_tracking/util/menger_curvature.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace pure_pursuit_logic
{

    PurePursuitAlgorithm::PurePursuitAlgorithm()
        : look_forward_gain_(0.3),
          min_look_ahead_distance_(2.0),
          max_look_ahead_distance_(10.0),
          target_velocity_(1.0),
          goal_radius_(0.5),
          curvature_gain_(1.5),
          velocity_reduction_gain_(1.0),
          last_found_target_idx_(-1),
          current_look_ahead_distance_(2.0),
          has_reached_goal_(false)
    {
    }

    void PurePursuitAlgorithm::setParams(double k_ld, double min_ld, double max_ld, double target_v, double goal_r, double k_c, double k_v_red)
    {
        look_forward_gain_ = k_ld;
        min_look_ahead_distance_ = min_ld;
        max_look_ahead_distance_ = max_ld;
        target_velocity_ = target_v;
        goal_radius_ = goal_r;
        curvature_gain_ = k_c;
        velocity_reduction_gain_ = k_v_red;
    }

    void PurePursuitAlgorithm::setPath(const std::vector<path_tracking::Point> &path_points)
    {
        current_path_logic_ = path_points;
        if (!current_path_logic_.empty())
        {
            // パスが設定されたら曲率を計算
            calculatePathCurvature(current_path_logic_);
            last_found_target_idx_ = 0;
        }
        else
        {
            last_found_target_idx_ = -1;
        }
        has_reached_goal_ = false;
    }

    double PurePursuitAlgorithm::normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // --- ヘルパー関数：最近傍点と目標点の探索 ---
    PurePursuitAlgorithm::ClosestPointResult PurePursuitAlgorithm::findClosestSegment(const path_tracking::Pose &current_pose) const
    {
        ClosestPointResult result;
        if (current_path_logic_.size() < 2)
            return result;

        for (size_t i = 0; i < current_path_logic_.size() - 1; ++i)
        {
            const auto &p1 = current_path_logic_[i];
            const auto &p2 = current_path_logic_[i + 1];
            double dx = p2.x - p1.x, dy = p2.y - p1.y;

            if (dx == 0 && dy == 0)
                continue;

            double len_sq = dx * dx + dy * dy;
            double dot = (current_pose.x - p1.x) * dx + (current_pose.y - p1.y) * dy;
            double t = std::max(0.0, std::min(1.0, dot / len_sq));

            double closest_x = p1.x + t * dx;
            double closest_y = p1.y + t * dy;
            double dist_sq = std::pow(current_pose.x - closest_x, 2) + std::pow(current_pose.y - closest_y, 2);

            if (dist_sq < result.distance_sq)
            {
                result.distance_sq = dist_sq;
                result.segment_idx = i;
                result.closest_t = t;
            }
        }
        return result;
    }

    int PurePursuitAlgorithm::searchTargetIndexByDistanceAlongPath(const ClosestPointResult &closest_result, double look_ahead_distance) const
    {
        double distance_traveled = 0.0;
        const auto &p_start_segment = current_path_logic_[closest_result.segment_idx];
        const auto &p_end_segment = current_path_logic_[closest_result.segment_idx + 1];
        double segment_length = std::hypot(p_end_segment.x - p_start_segment.x, p_end_segment.y - p_start_segment.y);

        distance_traveled = segment_length * (1.0 - closest_result.closest_t);

        if (distance_traveled >= look_ahead_distance)
        {
            return closest_result.segment_idx + 1;
        }

        for (size_t i = closest_result.segment_idx + 1; i < current_path_logic_.size() - 1; ++i)
        {
            const auto &p1 = current_path_logic_[i];
            const auto &p2 = current_path_logic_[i + 1];
            double current_segment_length = std::hypot(p2.x - p1.x, p2.y - p1.y);

            if (distance_traveled + current_segment_length >= look_ahead_distance)
            {
                return i + 1;
            }
            distance_traveled += current_segment_length;
        }

        return current_path_logic_.size() - 1;
    }

    // --- メインのUPDATE関数 ---
    bool PurePursuitAlgorithm::update(const path_tracking::Pose &current_pose, double current_velocity,
                                      double &out_linear_x_velocity, double &out_linear_y_velocity, double &out_angular_velocity)
    {
        if (current_path_logic_.empty() || has_reached_goal_ || current_path_logic_.size() < 2)
        {
            out_linear_x_velocity = 0.0;
            out_linear_y_velocity = 0.0;
            out_angular_velocity = 0.0;
            return false;
        }

        // 1. 経路上でロボットに最も近い点を見つける
        ClosestPointResult closest_result = findClosestSegment(current_pose);
        if (closest_result.segment_idx < 0)
            return false;

        // 2. 曲率に基づいてパラメータを動的に調整
        double path_curvature = current_path_logic_[closest_result.segment_idx].curvature;

        double current_target_velocity = target_velocity_ / (1.0 + velocity_reduction_gain_ * std::abs(path_curvature));

        double base_ld = look_forward_gain_ * std::abs(current_velocity) + min_look_ahead_distance_;
        double adaptive_ld = base_ld / (1.0 + curvature_gain_ * std::abs(path_curvature));
        double look_ahead_distance = std::max(min_look_ahead_distance_, std::min(adaptive_ld, max_look_ahead_distance_));

        // 3. パスに沿って目標点を探索
        int target_idx = searchTargetIndexByDistanceAlongPath(closest_result, look_ahead_distance);
        last_found_target_idx_ = target_idx; // last_found_target_idx_を更新

        // 4. ゴール判定
        const auto &final_goal = current_path_logic_.back();
        double dist_to_final_goal = std::hypot(final_goal.x - current_pose.x, final_goal.y - current_pose.y);

        if (dist_to_final_goal < goal_radius_)
        {
            has_reached_goal_ = true;
            out_linear_x_velocity = 0.0;
            out_linear_y_velocity = 0.0;
            out_angular_velocity = 0.0;
            return false;
        }

        // 5. 速度指令を計算
        const auto &target_point = current_path_logic_[target_idx];
        double dx_global = target_point.x - current_pose.x;
        double dy_global = target_point.y - current_pose.y;
        double target_angle = std::atan2(dy_global, dx_global);
        double alpha = normalizeAngle(target_angle - current_pose.yaw);

        out_linear_x_velocity = current_target_velocity * std::cos(alpha);
        out_linear_y_velocity = current_target_velocity * std::sin(alpha);

        // 6. 角速度指令を計算
        int next_idx_for_yaw = std::min(static_cast<int>(current_path_logic_.size()) - 1, target_idx);
        int prev_idx_for_yaw = std::max(0, next_idx_for_yaw - 1);

        double path_angle = std::atan2(
            current_path_logic_[next_idx_for_yaw].y - current_path_logic_[prev_idx_for_yaw].y,
            current_path_logic_[next_idx_for_yaw].x - current_path_logic_[prev_idx_for_yaw].x);

        // 最終点に近づいたら、その手前のセグメントの向きを目標ヨー角とする
        if (target_idx == current_path_logic_.size() - 1)
        {
            const auto &p_final = current_path_logic_.back();
            const auto &p_before_final = current_path_logic_[current_path_logic_.size() - 2];
            path_angle = std::atan2(p_final.y - p_before_final.y, p_final.x - p_before_final.x);
        }

        double yaw_error = normalizeAngle(path_angle - current_pose.yaw);
        out_angular_velocity = 2.0 * yaw_error; // このゲイン(2.0)も調整可能なパラメータにすると良い

        return true;
    }

} // namespace pure_pursuit_logic
