#ifndef PATH_TRACKING_COMMON_HPP
#define PATH_TRACKING_COMMON_HPP

namespace path_tracking
{
    struct Point
    {
        double x;
        double y;
        double curvature = 0.0;
    };

    struct Pose
    {
        double x;
        double y;
        double yaw;
    };
}

#endif // PATH_TRACKING_COMMON_HPP