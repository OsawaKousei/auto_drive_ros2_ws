#include "path_tracking/pure_pursuit/pure_pursuit_logic.hpp"
#include <limits> // Not strictly needed now, but good for general math algorithms

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
          last_found_target_idx_(-1),
          current_look_ahead_distance_(2.0),
          has_reached_goal_(false)
    {
    }

    void PurePursuitAlgorithm::setParams(double k_ld, double min_ld, double max_ld, double target_v, double goal_r)
    {
        look_forward_gain_ = k_ld;
        min_look_ahead_distance_ = min_ld;
        max_look_ahead_distance_ = max_ld;
        target_velocity_ = target_v;
        goal_radius_ = goal_r;
    }

    void PurePursuitAlgorithm::setPath(const std::vector<path_tracking::Point> &path_points)
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

    int PurePursuitAlgorithm::searchTargetIndex(const path_tracking::Pose &current_pose, double current_velocity)
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

    bool PurePursuitAlgorithm::update(const path_tracking::Pose &current_pose, double current_velocity,
                                      double &out_linear_x_velocity, double &out_linear_y_velocity, double &out_angular_velocity)
    {
        if (current_path_logic_.empty() || has_reached_goal_)
        {
            out_linear_x_velocity = 0.0;
            out_linear_y_velocity = 0.0;
            out_angular_velocity = 0.0;
            return false;
        }

        int target_idx = searchTargetIndex(current_pose, current_velocity);

        if (target_idx < 0)
        {
            out_linear_x_velocity = 0.0;
            out_linear_y_velocity = 0.0;
            out_angular_velocity = 0.0;
            return false;
        }

        const auto &target_point = current_path_logic_[target_idx];

        double dx_global = target_point.x - current_pose.x;
        double dy_global = target_point.y - current_pose.y;

        // 目標点への角度（グローバル座標系）
        double target_angle = std::atan2(dy_global, dx_global);

        // ロボットの現在の姿勢からの相対角度
        double alpha = normalizeAngle(target_angle - current_pose.yaw);

        // 全方向移動ロボットの場合、車体の向きに関係なく目標方向に動けるため
        // ローカル座標系でのx,y速度成分を計算
        double distance_to_target = std::sqrt(dx_global * dx_global + dy_global * dy_global);

        // ターゲット点への速度ベクトルをロボット座標系に変換
        double speed_ratio = std::min(1.0, distance_to_target / current_look_ahead_distance_);
        double local_vx = target_velocity_ * speed_ratio * std::cos(alpha);
        double local_vy = target_velocity_ * speed_ratio * std::sin(alpha);

        // グローバル座標系の速度をロボット座標系に変換
        out_linear_x_velocity = local_vx;
        out_linear_y_velocity = local_vy;

        // 角速度計算 - 目標角度へ向かう回転速度
        // 最終ターゲットの場合、最終点での姿勢も考慮する
        if (static_cast<size_t>(target_idx) == current_path_logic_.size() - 1)
        {
            // 最終点に近づいたら姿勢も合わせる
            // 最終点での理想的な姿勢は、最後の2点を結ぶ方向とする
            // ここでは簡単のため、現在のyaw角をそのまま維持
            out_angular_velocity = 0.0;

            // 最終ゴールに十分近づいたかチェック
            double dx_to_actual_end = current_path_logic_.back().x - current_pose.x;
            double dy_to_actual_end = current_path_logic_.back().y - current_pose.y;
            double dist_to_final_goal = std::sqrt(dx_to_actual_end * dx_to_actual_end + dy_to_actual_end * dy_to_actual_end);

            if (dist_to_final_goal < goal_radius_)
            {
                has_reached_goal_ = true;
                out_linear_x_velocity = 0.0;
                out_linear_y_velocity = 0.0;
                out_angular_velocity = 0.0;
                return false; // 終了を示す
            }
        }
        else
        {
            // 経路に沿った方向を向くような角速度
            // 経路の接線方向（次の点への方向）を計算
            int next_idx = std::min(static_cast<int>(current_path_logic_.size()) - 1, target_idx + 1);
            double path_angle = std::atan2(
                current_path_logic_[next_idx].y - current_path_logic_[target_idx].y,
                current_path_logic_[next_idx].x - current_path_logic_[target_idx].x);

            double yaw_error = normalizeAngle(path_angle - current_pose.yaw);
            out_angular_velocity = 2.0 * yaw_error; // 比例係数は調整可能
        }

        return true;
    }

} // namespace pure_pursuit_logic
