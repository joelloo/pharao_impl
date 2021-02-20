#ifndef INCLUDE_ANGLE_UTILITIES_H
#define INCLUDE_ANGLE_UTILITIES_H

#include <cmath>
#include <Eigen/Dense>

/// ** Angle utilities
///    Authored by: Joel

namespace Util
{

namespace Angle
{

static const double M_TWO_PI = M_PI * 2;
static const double DEG2RAD_FACTOR = M_PI / 180.f;
static const double RAD2DEG_FACTOR = 180.f / M_PI;

inline double deg2Rad(const double angle)
{
    return angle * DEG2RAD_FACTOR;
}

inline double rad2Deg(const double angle)
{
    return angle * RAD2DEG_FACTOR;
}

/// \brief Computes the yaw for both 2D and 3D rotation matrices
/// (for 2D matrices it accepts a 2D-H matrix but does not utilise 
/// translation)
inline double getYaw(const Eigen::Matrix3d& aRotationMatrix)
{
    return std::atan2(aRotationMatrix(1, 0), aRotationMatrix(0, 0));
}

/// \brief Wraps angles into the range [-pi, pi)
inline double wrap(const double angle)
{
    double wrapped = angle;

    while (wrapped < -M_PI)
    {
        wrapped += M_TWO_PI;
    }

    while (!(wrapped < M_PI))
    {
        wrapped -= M_TWO_PI;
    }

    return wrapped;
}

/// \brief Wraps angles into the range [-180, 180) degrees
inline double wrapDeg(const double angle)
{
    double wrapped = angle;

    while (wrapped < -180.f)
    {
        wrapped += 360.f;
    }

    while (!(wrapped < 180.f))
    {
        wrapped -= 360.f;
    }

    return wrapped;
}

/// \brief Returns minimum signed difference between two angles in the range [-pi, pi)
///        while accounting for wraparound at the -pi, pi boundary
inline double difference(const double a1, const double a2)
{
    double diff = a1 - a2;
    diff += (diff >= M_PI) ? -M_TWO_PI : ((diff < -M_PI) ? M_TWO_PI : 0);
    return diff;
}

} // Angle

} // Util

#endif // INCLUDE_ANGLE_UTILITIES_H