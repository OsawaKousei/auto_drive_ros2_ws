#ifndef PATH_TRACKING_MENGER_CURVATURE_HPP
#define PATH_TRACKING_MENGER_CURVATURE_HPP

#include <vector>
#include "path_tracking/common.hpp"

namespace path_tracking
{
    /**
     * @brief 3点の座標からMenger曲率を計算する関数
     * @param p1 1番目の点
     * @param p2 2番目の点
     * @param p3 3番目の点
     * @return 計算された曲率
     */
    double calculateMengerCurvature(const Point &p1, const Point &p2, const Point &p3);

    /**
     * @brief パス全体の曲率を計算して格納する関数
     * @param path 曲率を計算するパス（値は更新される）
     */
    void calculatePathCurvature(std::vector<Point> &path);
}

#endif // PATH_TRACKING_MENGER_CURVATURE_HPP
