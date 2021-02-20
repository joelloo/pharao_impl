#ifndef INCLUDE_POSE_GRAPH_RESIDUAL_HPP
#define INCLUDE_POSE_GRAPH_RESIDUAL_HPP

#include <ceres/ceres.h>

#include <iostream>
#include <exception>
#include <vector>
#include <cmath>

#include "utils/Angle.hpp"

namespace Pharao
{

class SE2OdometryResidual
{
public:
    SE2OdometryResidual(const Eigen::Matrix<double, 3, 3>& aMeasurement)
        : mMeasurement(aMeasurement)
    {
        mSqrtInformation = Eigen::Matrix<double, 3, 3>::Identity();
    }

    SE2OdometryResidual(const Eigen::Matrix<double, 3, 3>& aMeasurement,
                        const Eigen::Matrix<double, 3, 3>& aSqrtInformation)
        : mMeasurement(aMeasurement)
        , mSqrtInformation(aSqrtInformation)
    {
    }

    template <typename T>
    bool operator()(const T* const x0, const T* const y0, const T* const yaw0,
                    const T* const x1, const T* const y1, const T* const yaw1,
                    T* residuals) const
    {
        Eigen::Matrix<T, 3, 3> H_meas = mMeasurement.cast<T>();
        Eigen::Matrix<T, 3, 3> sqrtInformation = mSqrtInformation.cast<T>();

        T r0 = yaw0[0];
        T r1 = yaw1[0];
        T cr0 = ceres::cos(r0);
        T sr0 = ceres::sin(r0);
        T cr1 = ceres::cos(r1);
        T sr1 = ceres::sin(r1);

        T Zero(0.0);
        T One(1.0);
        Eigen::Matrix<T, 3, 3> H_0_world, H_1_world;
        H_0_world << cr0, -sr0, x0[0],
                     sr0, cr0, y0[0],
                     Zero, Zero, One;
        H_1_world << cr1, -sr1, x1[0],
                     sr1, cr1, y1[0],
                     Zero, Zero, One;
        
        Eigen::Matrix<T, 3, 3> H_1_0 = H_0_world.inverse() * H_1_world;
        Eigen::Matrix<T, 3, 3> H_error = H_meas * H_1_0.inverse();

        Eigen::Matrix<T, 3, 1> error;
        error << H_error(0, 2), H_error(1, 2), atan2(H_error(1, 0), H_error(0, 0));
        error = sqrtInformation * error;

        residuals[0] = error(0);
        residuals[1] = error(1);
        residuals[2] = error(2);
        return true;
    }

private:
    Eigen::Matrix<double, 3, 3> mMeasurement;
    Eigen::Matrix<double, 3, 3> mSqrtInformation;
};

class HeadingCorrectionResidual
{
public:
    HeadingCorrectionResidual(const double aYaw)
        : mYaw(aYaw)
        {
            mWeight = 1.0f;
        }
    
    HeadingCorrectionResidual(const double aYaw, const double aWeight)
        : mYaw(aYaw)
        , mWeight(aWeight)
    {
    }

    template <typename T>
    bool operator()(const T* const p,
                    T* residuals) const
    {
        T diff = angleDifference(p[0], static_cast<T>(mYaw));
        residuals[0] = static_cast<T>(mWeight) * diff * diff;
        return true;
    }

private:
    template <typename T>
    T angleDifference(const T a1, const T a2) const
    {
        T Pi(M_PI);
        T TwoPi(Util::Angle::M_TWO_PI);
        T Zero(0);

        T diff = a1 - a2;
        diff += (diff >= Pi) ? -TwoPi : ((diff < -Pi) ? TwoPi : Zero);
        return diff;
    }

    double mYaw;
    double mWeight;
};

}

#endif