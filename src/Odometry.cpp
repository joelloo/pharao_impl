#include "Odometry.hpp"

namespace Pharao
{

Odometry::Odometry(const Util::ParameterList& aParams)
    : mpMatcher(nullptr)
{
    // Odometry expects radar range images of the form
    // min_range ... ... ... ... max_range
    mCoarsePolarSize.width = static_cast<int>(aParams["coarse_polar_width"]);
    mCoarsePolarSize.height = static_cast<int>(aParams["coarse_polar_height"]);
    mCoarseCartesianSize.width = static_cast<int>(aParams["coarse_cartesian_width"]);
    mCoarseCartesianSize.height = static_cast<int>(aParams["coarse_cartesian_height"]);
    mCoarseLogPolarSize.width = cv::getOptimalDFTSize(mCoarseCartesianSize.width) & -2;
    mCoarseLogPolarSize.height = cv::getOptimalDFTSize(mCoarseCartesianSize.height) & -2;
    mFinePolarSize.width = static_cast<int>(aParams["fine_polar_width"]);
    mFinePolarSize.height = static_cast<int>(aParams["fine_polar_height"]);
    mFineCartesianSize.width = static_cast<int>(aParams["fine_cartesian_width"]);
    mFineCartesianSize.height = static_cast<int>(aParams["fine_cartesian_height"]);

    double radarMaxRange = static_cast<float>(aParams["radar_max_range_metres"]);
    mRadarMaxRangeSquared = radarMaxRange * radarMaxRange;

    // Set image centres and ROI for image preprocessing step
    mCoarsePolarTransformCentre = cv::Point2f(static_cast<float>(mCoarseCartesianSize.width) / 2.f, 
        static_cast<float>(mCoarseCartesianSize.height) / 2.f);
    mFinePolarTransformCentre = cv::Point2f(static_cast<float>(mFineCartesianSize.width) / 2.f,
        static_cast<float>(mFineCartesianSize.height) / 2.f);
    mFineCropROI = cv::Rect(0, 0, mFinePolarSize.width, mFinePolarSize.height);

    mCoarsePolarImage = cv::Mat(mCoarsePolarSize, CV_32F);
    mFinePolarImage = cv::Mat(mFinePolarSize, CV_32F);

    std::string matchingAlgorithm = static_cast<std::string>(aParams["image_registration_algorithm"]);
    if (matchingAlgorithm.compare("lpfft") == 0)
    {
        mpMatcher = std::make_shared<LogPolarFFTMatcher>(aParams);
    }
    else if (matchingAlgorithm.compare("imreg") == 0)
    {
        mpMatcher = std::make_shared<ImageRegistration>(aParams);
    }
    else
    {
        std::cout << "Unknown image registration algorithm" << std::endl;
        throw std::exception("Unknown image registration algorithm");
    }

    mSlidingWindowSize = static_cast<int>(aParams["frame_sliding_window_size"]);
    mFrameBuffer.set_capacity(mSlidingWindowSize + 2, mCoarseCartesianSize, mCoarseLogPolarSize, mFineCartesianSize);

    mPlatformMaxVelocity = static_cast<float>(aParams["platform_max_velocity_m_s"]);
    mPlatformMaxAngularVelocity = static_cast<float>(aParams["platform_max_angular_velocity_rad_s"]);
    mConfidenceThreshold = static_cast<float>(aParams["confidence_alpha"]);

    mRunPoseGraphOpt = static_cast<bool>(aParams["use_pgo"]);
    mUseRobustLoss = static_cast<bool>(aParams["use_robust_loss"]);
    mRobustLossParam = static_cast<float>(aParams["robust_loss_param"]);
    mHeadingCorrectionFactorScale = static_cast<float>(aParams["heading_correction_factor_scale"]);
}

Odometry::~Odometry()
{
}

void
Odometry::initialiseFrame(const cv::Mat& aImage, Frame& aPrevFrame)
{
    cv::resize(aImage, mCoarsePolarImage, mCoarsePolarSize, 0, 0, cv::INTER_LINEAR);

    int unwarpFlags = cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS + cv::WARP_INVERSE_MAP;
    double maxCoarseRadius = static_cast<double>(mCoarsePolarSize.width);
    cv::Mat cartesianImage;
    cv::warpPolar(mCoarsePolarImage, cartesianImage, mCoarseCartesianSize, 
        mCoarsePolarTransformCentre, maxCoarseRadius, unwarpFlags);
    
    mpMatcher->initialise(cartesianImage, aPrevFrame);

    mFinePolarImage = aImage(mFineCropROI);
    double maxFineRadius = static_cast<double>(mFinePolarSize.width);
    cv::warpPolar(mFinePolarImage, aPrevFrame.mFineCartesianImage, mFineCartesianSize, 
        mFinePolarTransformCentre, maxFineRadius, unwarpFlags);
}

bool
Odometry::update(const std::chrono::nanoseconds& aTimestamp, const cv::Mat& aImage, Pose2D& aUpdatedPose, Pose2D& aDeltaPose)
{
    if (mFrameBuffer.empty()) 
    {
        std::cout << "Frame buffer empty... initialising frame" << std::endl;
        mFrameBuffer.push_back();
        Frame& first = mFrameBuffer.back();

        first.mType = FrameType::KEYFRAME;
        first.mTimestamp = aTimestamp;
        first.mPoseFromPrevFrame = Pose2D(0, 0, 0, aTimestamp);
        first.mPoseFromKeyframe = Pose2D(0, 0, 0, aTimestamp);
        first.mWorldPose = Pose2D(0, 0, 0, aTimestamp);
        first.mHeadingOdometryFactor = 0.0;
        initialiseFrame(aImage, first);
        return false;
    }

    cv::imshow("Raw data", aImage);
    

    // Grab reference to previous frame
    Frame& prevFrame = mFrameBuffer.back();

    // Initialise current frame
    mFrameBuffer.push_back();
    Frame& currFrame = mFrameBuffer.back();
    currFrame.mTimestamp = aTimestamp;
    currFrame.mType = FrameType::FRAME;
    currFrame.mPoseFromPrevFrame = Pose2D(0, 0, 0, aTimestamp);
    currFrame.mPoseFromKeyframe = Pose2D(0, 0, 0, aTimestamp);
    currFrame.mHeadingOdometryFactor = 0.0;

    // Downsample to create coarse images
    cv::resize(aImage, mCoarsePolarImage, mCoarsePolarSize, 0, 0, cv::INTER_LINEAR);

    // Generate coarse images
    int unwarpFlags = cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS + cv::WARP_INVERSE_MAP;
    double maxCoarseRadius = static_cast<double>(mCoarsePolarSize.width);
    cv::warpPolar(mCoarsePolarImage, currFrame.mCoarseCartesianImage, mCoarseCartesianSize, 
       mCoarsePolarTransformCentre, maxCoarseRadius, unwarpFlags);

    // Perform coarse pose estimation
    Pose2D estimatedPose;
    estimatedPose.timestamp = aTimestamp;
    estimatedPose = mpMatcher->estimateCoarse(prevFrame, currFrame);
    
    //! TODO: Perform fine translation estimation

    // Check frame validity and compute confidence (wrt previous frame and previous keyframe)
    if (!checkFrameValidity(prevFrame.mTimestamp, estimatedPose, currFrame.mConfPrevFrame))
    {
        mFrameBuffer.pop_back();
        std::cout << "Current frame is invalid (" << currFrame.mConfPrevFrame << ")" << std::endl;
        return false;
    }

    // Update frame with pose and necessary information
    const Eigen::Matrix3d estimatedPoseMat = estimatedPose.toMatrix();
    currFrame.mPoseFromKeyframe = Pose2D(prevFrame.mPoseFromKeyframe.toMatrix() * estimatedPoseMat, currFrame.mTimestamp);
    currFrame.mConfKeyframe = computeConfidence(currFrame.mPoseFromKeyframe);

    currFrame.mPoseFromPrevFrame = estimatedPose;
    currFrame.mOdomResidualInvSqrtInfo << currFrame.mConfKeyframe / estimatedPose.xVar, 0, 0,
                                          0, currFrame.mConfKeyframe / estimatedPose.yVar, 0,
                                          0, 0, currFrame.mConfKeyframe / estimatedPose.yawVar;

    // Compute heading odometry factor (against previous keyframe)
    if (prevFrame.mType == FrameType::KEYFRAME)
    {
        currFrame.mHeadingOdometryFactor = currFrame.mPoseFromPrevFrame.yaw;
        currFrame.mHeadingResidualInvSqrtInfo = 0.0;
    }
    else
    {
        Frame& prevKeyframe = mFrameBuffer[0];
        Pose2D estimatedYaw = mpMatcher->estimateCoarseRotation(prevKeyframe, currFrame);
        currFrame.mHeadingOdometryFactor = estimatedYaw.yaw;
        currFrame.mHeadingResidualInvSqrtInfo = mHeadingCorrectionFactorScale * currFrame.mConfKeyframe / estimatedYaw.yawVar;

        //! NOTE: TESTING!
        // currFrame.mPoseFromKeyframe.yaw = estimatedYaw.yaw;
    }
    
    // Apply fixed-lag smoothing by optimising pose graph of recent poses
    if (mRunPoseGraphOpt && !smoothPoses())
    {
        std::cout << "Warning: Pose graph optimisation failed to converge! Reverting to unsmoothed poses..." << std::endl;
        for (int i = 1; i < mFrameBuffer.size(); ++i)
        {
            auto& frame = mFrameBuffer[i];
            frame.mPoseFromKeyframe = Pose2D(
                mFrameBuffer[i - 1].mPoseFromKeyframe.toMatrix() * frame.mPoseFromPrevFrame.toMatrix(),
                frame.mTimestamp);
        }
    }

    // Keyframing and frame buffer management
    std::cout << "Frame buffer size: " << mFrameBuffer.size() << std::endl;
    performKeyframing(prevFrame, currFrame);

    std::cout << "### Confidence: " << currFrame.mConfPrevFrame << std::endl;


    Frame& lastKeyframe = mFrameBuffer.front();
    aDeltaPose = estimatedPose; /// TODO: Does not reflect smoothed pose. Remove?
    aUpdatedPose = Pose2D(
        lastKeyframe.mWorldPose.toMatrix() * currFrame.mPoseFromKeyframe.toMatrix(), 
        currFrame.mTimestamp);

    return true;
}

void
Odometry::performKeyframing(Frame& aPrevFrame, Frame& aCurrFrame)
{
    // Conditions required for new keyframe to be selected
    Pose2D& poseFromKF = aCurrFrame.mPoseFromKeyframe;

    if (mFrameBuffer.full()
        || aPrevFrame.mConfKeyframe > aCurrFrame.mConfKeyframe
        || poseFromKF.x * poseFromKF.x + poseFromKF.y * poseFromKF.y > mRadarMaxRangeSquared)
    {
        std::cout << "----- Selecting new keyframe!" << std::endl;
        std::cout << aPrevFrame.mConfKeyframe << "," << aCurrFrame.mConfKeyframe << "," << mFrameBuffer.full() << std::endl;
        if (mFrameBuffer.size() <= 1)
        {
            std::cout << "Warning: Frame buffer expected to have >= 2 elements!" << std::endl;
            throw std::exception("Out of bounds access of frame buffer");
        }
        else if (mFrameBuffer.size() == 2)
        {
            std::cout << "Warning: Selecting new keyframe from buffer of size 2!" << std::endl;
        }

        // Select new keyframe, by iterating through keyframe candidates
        double feasibleConf = -DBL_MAX;
        int lastIndex = std::min(mSlidingWindowSize, static_cast<int>(mFrameBuffer.size() - 1));
        auto beginIt = mFrameBuffer.iterator_at(1);
        auto endIt = mFrameBuffer.iterator_at(lastIndex);
        ++endIt; // we need the end iterator, i.e. one past the last index

        for (auto it = beginIt; it != endIt; ++it) { feasibleConf = std::max(feasibleConf, it->mConfKeyframe); }
        feasibleConf *= 0.9;
        
        double deltaYawKeyframe = -DBL_MAX;
        int nextKeyframeIndex = -1;
        int index = 1;
        for (auto it = beginIt; it != endIt; ++index, ++it)
        {
            if (it->mConfKeyframe > feasibleConf 
                && fabs(it->mPoseFromKeyframe.yaw) > deltaYawKeyframe)
            {
                nextKeyframeIndex = index;
                deltaYawKeyframe = fabs(it->mPoseFromKeyframe.yaw);
            }
        }

        int i = 0;
        for (auto it = mFrameBuffer.begin(); it != mFrameBuffer.end(); ++it, ++i)
        {
            std::cout << i << ": [" << static_cast<int>(it->mType) << "," << it->mConfKeyframe << "," << it->mPoseFromKeyframe << "]   ";
        }
        std::cout << std::endl;
        std::cout << "New keyframe selected at: " << nextKeyframeIndex << std::endl;

        // Set new keyframe and recompute confidence/poses for remaining frames
        Frame& nextKeyframe = mFrameBuffer[nextKeyframeIndex];
        Frame& lastKeyframe = mFrameBuffer.front();
        nextKeyframe.mType = FrameType::KEYFRAME;
        nextKeyframe.mConfPrevFrame = 0.0;
        nextKeyframe.mConfKeyframe = 0.0;
        nextKeyframe.mWorldPose = Pose2D(
            lastKeyframe.mWorldPose.toMatrix() * nextKeyframe.mPoseFromKeyframe.toMatrix(),
            nextKeyframe.mTimestamp);
        nextKeyframe.mPoseFromKeyframe = Pose2D(0, 0, 0, nextKeyframe.mTimestamp);
        nextKeyframe.mPoseFromPrevFrame = Pose2D(0, 0, 0, nextKeyframe.mTimestamp);

        mFrameBuffer.clear_until_next_keyframe();

        if (mFrameBuffer.size() > 1)
        {
            auto prev = mFrameBuffer.iterator_at(0);
            for (auto it = mFrameBuffer.iterator_at(1); it != mFrameBuffer.end(); ++it)

            for (int i = 1; i < mFrameBuffer.size(); ++i)
            {
                auto& frame = mFrameBuffer[i];
                frame.mPoseFromKeyframe = Pose2D(
                    mFrameBuffer[i - 1].mPoseFromKeyframe.toMatrix() * frame.mPoseFromPrevFrame.toMatrix(),
                    frame.mTimestamp);
                frame.mConfKeyframe = computeConfidence(frame.mPoseFromKeyframe);
            }
        }
    }
}

bool
Odometry::checkFrameValidity(const std::chrono::nanoseconds& aPrevTs,
    const Pose2D& aDeltaPose, double& aScore)
{
    // Check if delta pose violates our motion constraints
    std::chrono::duration<double> diff = aDeltaPose.timestamp - aPrevTs;
    double secsInv = 1.f / diff.count();
    if (sqrtf(aDeltaPose.x * aDeltaPose.x + aDeltaPose.y * aDeltaPose.y) * secsInv > mPlatformMaxVelocity
        && fabs(aDeltaPose.yaw) * secsInv > mPlatformMaxAngularVelocity)
    {
        std::cout << "Failed velocity check" << std::endl;
        return false;
    }

    // Compute confidence score and see if it passes our threshold
    aScore = computeConfidence(aDeltaPose);

    return aScore > mConfidenceThreshold;
}

bool
Odometry::smoothPoses()
{
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Matrix3d>> odometryMeasurements;
    std::vector<std::pair<double, double>> headingMeasurements;

    for (auto it = mFrameBuffer.begin(); it != mFrameBuffer.end(); ++it)
    {
        if (it->mType == FrameType::KEYFRAME) { continue; }

        std::cout << it->mOdomResidualInvSqrtInfo << std::endl;
        std::cout << it->mHeadingResidualInvSqrtInfo << std::endl;

        // odometryMeasurements.push_back(std::make_pair(it->mPoseFromPrevFrame.toMatrix(),
        //     Eigen::Matrix<double, 3, 3>::Identity()));
        // headingMeasurements.push_back(std::make_pair(it->mHeadingOdometryFactor, 1.0f));

        odometryMeasurements.push_back(std::make_pair(it->mPoseFromPrevFrame.toMatrix(), 
            it->mOdomResidualInvSqrtInfo));
        headingMeasurements.push_back(std::make_pair(it->mHeadingOdometryFactor, 
            it->mHeadingResidualInvSqrtInfo));
    }

    ceres::Problem problem;
    for (int i = 0; i < odometryMeasurements.size(); ++i)
    {
        ceres::LossFunction* loss = 0;
        if (mUseRobustLoss)
        {
            loss = new ceres::HuberLoss(mRobustLossParam);
        }

        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<SE2OdometryResidual, 3, 1, 1, 1, 1, 1, 1>(
                new SE2OdometryResidual(odometryMeasurements[i].first, odometryMeasurements[i].second));

        auto& frame0 = mFrameBuffer[i];
        auto& frame1 = mFrameBuffer[i + 1];

        problem.AddResidualBlock(costFunction, loss,
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
        ceres::LossFunction* loss = 0;
        if (mUseRobustLoss)
        {
            loss = new ceres::HuberLoss(mRobustLossParam);
        }

        ceres::CostFunction* costFunction = 
            new ceres::AutoDiffCostFunction<HeadingCorrectionResidual, 1, 1>(
                new HeadingCorrectionResidual(headingMeasurements[i].first, headingMeasurements[i].second));

        problem.AddResidualBlock(costFunction, loss, &mFrameBuffer[i + 1].mPoseFromKeyframe.yaw);
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    bool ret = (summary.termination_type == ceres::CONVERGENCE);
    if (ret) std::cout << "Optimisation converged! Iters (" << summary.iterations.size() << ") with cost (" << summary.initial_cost << " -> " << summary.final_cost << ")" << std::endl;

    return ret;
}

inline double
Odometry::computeConfidence(const Pose2D& aDeltaPose)
{
    return exp(-fabs(Util::Angle::difference(std::atan2(aDeltaPose.y, aDeltaPose.x), aDeltaPose.yaw)));
}

void
Odometry::setInitialPose(const Eigen::Matrix3d& aInitialPose)
{
    if (mFrameBuffer.empty())
    {
        std::cout << "Warning: Frame buffer empty, cannot set initial pose!" << std::endl;
        return;
    }
    
    if (mFrameBuffer.front().mType != FrameType::KEYFRAME)
    {
        std::cout << "Warning: First frame not a keyframe! Reassigning as keyframe..." << std::endl;
        mFrameBuffer.front().mType = FrameType::KEYFRAME;
    }

    mFrameBuffer.front().mWorldPose.fromMatrix(aInitialPose);
}

}