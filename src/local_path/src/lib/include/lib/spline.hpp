#ifndef LOCAL_PATH_LIB_SPLINE_HPP
#define LOCAL_PATH_LIB_SPLINE_HPP

#include <vector>

namespace path_planning
{
    std::pair<std::vector<double>, std::vector<double>> spline_by_num(std::vector<double> xs, std::vector<double> ys, const int &num_points);
    std::pair<std::vector<double>, std::vector<double>> spline_by_min_max(std::vector<double> xs, std::vector<double> ys,
                                                                          double l_min, double l_max, double d_max);
}

#endif // LOCAL_PATH_LIB_SPLINE_HPP
