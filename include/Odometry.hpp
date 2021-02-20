#ifndef INCLUDE_RADAR_ODOMETRY_HPP
#define INCLUDE_RADAR_ODOEMTRY_HPP

#include <chrono>
#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include "entity/Pose2D.hpp"
#include "entity/Frame.hpp"
#include "entity/FrameBuffer.hpp"
#include "utils/Angle.hpp"
#include "utils/ParameterList.h"

#include "registration/logpolar_fft/LogPolarFFT.hpp"
#include "registration/imreg_dft/image_registration.h"

#include "PoseGraphResidual.hpp"

namespace Pharao
{

class Odometry
{
public:
    Odometry(const Util::ParameterList& aParams);
    ~Odometry();

    bool update(const std::chrono::nanoseconds& aTimestamp, const cv::Mat& aImage, Pose2D& aUpdatedPose, Pose2D& aDeltaPose);

    void setInitialPose(const Eigen::Matrix3d& aInitialPose);

private:
    void initialiseFrame(const cv::Mat& aImage, Frame& aPrevFrame);

    bool checkFrameValidity(const std::chrono::nanoseconds& aPrevTs, const Pose2D& aDeltaPose, double& aScore);

    void performKeyframing(Frame& aPrevFrame, Frame& aCurrFrame);

    bool smoothPoses();

    inline double computeConfidence(const Pose2D& aDeltaPose);

    cv::Size mCoarsePolarSize;
    cv::Size mCoarseCartesianSize;
    cv::Size mCoarseLogPolarSize;
    cv::Size mFinePolarSize;
    cv::Size mFineCartesianSize;

    cv::Point2f mCoarsePolarTransformCentre;
    cv::Point2f mFinePolarTransformCentre;
    cv::Rect mFineCropROI;

    cv::Mat mCoarsePolarImage;
    cv::Mat mFinePolarImage;

    Eigen::Matrix3d mPose;

    std::shared_ptr<ScanMatcher> mpMatcher;
    FrameBuffer mFrameBuffer;

    double mPlatformMaxVelocity;
    double mPlatformMaxAngularVelocity;
    double mRadarMaxRangeSquared;

    // Keyframing parameters
    double mConfidenceThreshold;
    int mSlidingWindowSize;

    // Pose graph optimisation parameters
    bool mRunPoseGraphOpt;
    bool mUseRobustLoss;
    double mRobustLossParam;
    double mHeadingCorrectionFactorScale;
};

}

#endif