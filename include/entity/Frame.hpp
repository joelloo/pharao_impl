#ifndef INCLUDE_SCAN_FRAME_HPP
#define INCLUDE_SCAN_FRAME_HPP

#include <chrono>
#include "opencv2/opencv.hpp"

#include <Eigen/Dense>

#include "entity/Pose2D.hpp"

namespace Pharao
{

enum class FrameType
{
    KEYFRAME,
    FRAME,
    INVALID,
};

struct Frame
{
    std::chrono::nanoseconds mTimestamp;
    FrameType mType;

    cv::Mat mCoarseCartesianImage;
    cv::Mat mCoarseLogPolarImage;
    cv::Mat mFineCartesianImage;

    double mConfPrevFrame;
    double mConfKeyframe;

    // Used only in regular frames, to specify measurements and 
    // pose relative to previous keyframe
    Pose2D mPoseFromPrevFrame;
    Pose2D mPoseFromKeyframe;
    double mHeadingOdometryFactor;
    Eigen::Matrix<double, 3, 3> mOdomResidualInvSqrtInfo;
    double mHeadingResidualInvSqrtInfo;

    // Used only in keyframes to specify pose from origin
    Pose2D mWorldPose;
};

}

#endif