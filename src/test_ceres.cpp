#include <ceres/ceres.h>
#include <glog/logging.h>

#include <cmath>

#include "utils/Stopwatch.h"
#include "PoseGraphResidual.hpp"
#include "entity/FrameBuffer.hpp"

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    Pharao::FrameBuffer buffer;
    buffer.set_capacity(7, cv::Size(1, 1), cv::Size(1, 1), cv::Size(1, 1));

    buffer.push_back();
    buffer.back().mType = Pharao::FrameType::KEYFRAME;
    buffer.back().mPoseFromPrevFrame = Pharao::Pose2D(0, 0, 0);
    buffer.back().mPoseFromKeyframe = Pharao::Pose2D(0, 0, 0);
    buffer.back().mHeadingOdometryFactor = 0.0;
    
    buffer.push_back();
    buffer.back().mType = Pharao::FrameType::FRAME;
    buffer.back().mPoseFromPrevFrame = Pharao::Pose2D(5, 2, M_PI / 2);
    buffer.back().mPoseFromKeyframe = Pharao::Pose2D(0, 0, 0);
    // buffer.back().mPoseFromKeyframe = Pharao::Pose2D(0, 5, 0);
    buffer.back().mHeadingOdometryFactor = M_PI / 2 + 0.1;

    buffer.push_back();
    buffer.back().mType = Pharao::FrameType::FRAME;
    buffer.back().mPoseFromPrevFrame = Pharao::Pose2D(0, 4, M_PI / 4);
    buffer.back().mPoseFromKeyframe = Pharao::Pose2D(0, 0, 0);
    // buffer.back().mPoseFromKeyframe = Pharao::Pose2D(0, 9, 0);
    buffer.back().mHeadingOdometryFactor = 3 * M_PI / 4 - 0.2;

    buffer.push_back();
    buffer.back().mType = Pharao::FrameType::FRAME;
    buffer.back().mPoseFromPrevFrame = Pharao::Pose2D(3, 0, -0.1);
    buffer.back().mPoseFromKeyframe = Pharao::Pose2D(0, 0, 0);
    buffer.back().mHeadingOdometryFactor = 3 * M_PI / 4 + 0.1;
    
    std::vector<Eigen::Matrix3d> odometryMeasurements;
    std::vector<double> headingMeasurements;

    std::cout << "Initial poses: " << std::endl;
    for (auto it = buffer.begin(); it != buffer.end(); ++it)
    {
        std::cout << it->mPoseFromKeyframe.x << ", " << it->mPoseFromKeyframe.y << ", " << it->mPoseFromKeyframe.yaw << std::endl;
        if (it->mType == Pharao::FrameType::KEYFRAME) continue;
        odometryMeasurements.push_back(it->mPoseFromPrevFrame.toMatrix());
        headingMeasurements.push_back(it->mHeadingOdometryFactor);
    }

    std::cout << "Odometry measurements (" << odometryMeasurements.size() << ")" << std::endl;
    for (auto& measurement : odometryMeasurements)
    {
        std::cout << measurement << std::endl;
        std::cout << "-----" << std::endl;
    }

    Util::Stopwatch::tic();
    ceres::Problem problem;

    for (int i = 0; i < odometryMeasurements.size(); ++i)
    {
        ceres::CostFunction* costFunction = 
            new ceres::AutoDiffCostFunction<Pharao::SE2OdometryResidual, 3, 1, 1, 1, 1, 1, 1>(
                new Pharao::SE2OdometryResidual(odometryMeasurements[i]));

        auto& frame0 = buffer[i];
        auto& frame1 = buffer[i + 1];

        problem.AddResidualBlock(costFunction, new ceres::HuberLoss(1.0),
            &frame0.mPoseFromKeyframe.x, &frame0.mPoseFromKeyframe.y, &frame0.mPoseFromKeyframe.yaw,
            &frame1.mPoseFromKeyframe.x, &frame1.mPoseFromKeyframe.y, &frame1.mPoseFromKeyframe.yaw);

        if (i == 0)
        {
            problem.SetParameterBlockConstant(&frame0.mPoseFromKeyframe.x);
            problem.SetParameterBlockConstant(&frame0.mPoseFromKeyframe.y);
            problem.SetParameterBlockConstant(&frame0.mPoseFromKeyframe.yaw);
        }
    }

    for (int i = 0; i < headingMeasurements.size(); ++i)
    {
        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<Pharao::HeadingCorrectionResidual, 1, 1>(
                new Pharao::HeadingCorrectionResidual(headingMeasurements[i]));
        
        problem.AddResidualBlock(costFunction, new ceres::HuberLoss(1.0), 
            &buffer[i + 1].mPoseFromKeyframe.yaw);
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "Took: " << Util::Stopwatch::toc() << std::endl;

    std::cout << summary.BriefReport() << "\n" << std::endl;
    std::cout << "Optimised poses: " << std::endl;
    for (auto it = buffer.begin(); it != buffer.end(); ++it)
    {
        std::cout << it->mPoseFromKeyframe.x << ", " << it->mPoseFromKeyframe.y << ", " << it->mPoseFromKeyframe.yaw << std::endl;
    }
    return 0;
}