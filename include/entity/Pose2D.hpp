#ifndef INCLUDE_POSE2D_HPP
#define INCLUDE_POSE2D_HPP

#include <cmath>
#include <chrono>
#include <iostream>
#include <Eigen/Dense>

#include "utils/Angle.hpp"

namespace Pharao
{

struct Pose2D
{
    Pose2D()
        : x(0)
        , y(0)
        , yaw(0)
    {
    }

    Pose2D(const double ax, const double ay, const double aYaw)
        : x(ax)
        , y(ay)
        , yaw(aYaw)
        , xVar(1)
        , yVar(1)
        , yawVar(1)
    {
    }

    Pose2D(const double ax, const double ay, const double aYaw,
        const std::chrono::nanoseconds& aTimestamp)
        : x(ax)
        , y(ay)
        , yaw(aYaw)
        , timestamp(aTimestamp)
        , xVar(1)
        , yVar(1)
        , yawVar(1)
    {
    }

    Pose2D(const Eigen::Matrix3d& aH, const std::chrono::nanoseconds& aTimestamp)
        : xVar(1)
        , yVar(1)
        , yawVar(1)
    {
        x = aH(0, 2);
        y = aH(1, 2);
        yaw = Util::Angle::getYaw(aH);
        timestamp = aTimestamp;
    }

    Eigen::Matrix3d toMatrix() const
    {
        Eigen::Matrix3d H;
        double cy = std::cos(yaw);
        double sy = std::sin(yaw);

        H << cy, -sy, x,
             sy,  cy, y,
             0, 0, 1;
        
        return H;
    }

    void fromMatrix(const Eigen::Matrix3d& aH)
    {
        x = aH(0, 2);
        y = aH(1, 2);
        yaw = Util::Angle::getYaw(aH);
    }

    friend std::ostream& operator<<(std::ostream& os, const Pose2D& pose)
    {
        os << "(" << pose.x << "," << pose.y << "," << pose.yaw << ")";
        return os;
    }

    std::chrono::nanoseconds timestamp;

    double x;
    double y;
    double yaw;

    double xVar;
    double yVar;
    double yawVar;
};

}

#endif