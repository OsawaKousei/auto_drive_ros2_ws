#include <cmath>
#include <vector>
#include "path_tracking/common.hpp"
#include "path_tracking/util/menger_curvature.hpp"

// 3点の座標からMenger曲率を計算する関数
double path_tracking::calculateMengerCurvature(const path_tracking::Point &p1, const path_tracking::Point &p2, const path_tracking::Point &p3)
{
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;

    double area_doubled = std::abs(dx1 * dy2 - dy1 * dx2); // 3点で成す三角形の面積の2倍

    double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    double d3 = std::sqrt(std::pow(p3.x - p1.x, 2) + std::pow(p3.y - p1.y, 2));

    if (d1 < 1e-6 || d2 < 1e-6 || d3 < 1e-6)
    {
        return 0.0; // 距離がゼロに近い場合は曲率もゼロ
    }

    return 2.0 * area_doubled / (d1 * d2 * d3);
}

// パス全体の曲率を計算して格納する関数
void path_tracking::calculatePathCurvature(std::vector<path_tracking::Point> &path)
{
    if (path.size() < 3)
    {
        return;
    }

    // 最初と最後の点の曲率は0とする
    path[0].curvature = 0.0;
    path[path.size() - 1].curvature = 0.0;

    for (size_t i = 1; i < path.size() - 1; ++i)
    {
        path[i].curvature = path_tracking::calculateMengerCurvature(path[i - 1], path[i], path[i + 1]);
    }
}