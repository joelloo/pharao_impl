#include "registration/logpolar_fft/LogPolarFFT.hpp"

using namespace std;
using namespace cv;

namespace Pharao
{

LogPolarFFTMatcher::LogPolarFFTMatcher(const Util::ParameterList& aParams)
    : ScanMatcher(aParams)
{
    mUseOriginal = static_cast<bool>(aParams["lpfft_use_original"]);

    mCoarseCartesianSize.width = static_cast<int>(aParams["coarse_cartesian_width"]);
    mCoarseCartesianSize.height = static_cast<int>(aParams["coarse_cartesian_height"]);
    mCoarseCartesianCentre = cv::Point2d(static_cast<double>(mCoarseCartesianSize.width) / 2, 
        static_cast<double>(mCoarseCartesianSize.height) / 2);
    mFineCartesianSize.width = static_cast<int>(aParams["fine_cartesian_width"]);
    mFineCartesianSize.height = static_cast<int>(aParams["fine_cartesian_height"]);

    mScaleThreshold = static_cast<float>(aParams["lpfft_estimated_scale_threshold"]);
    mRadarRangeDiameter = 2.f * static_cast<float>(aParams["radar_max_range_metres"]);
    
    mDFTSize.height = getOptimalDFTSize(mCoarseCartesianSize.height);
    mDFTSize.width = getOptimalDFTSize(mCoarseCartesianSize.width);
    mPaddedEvenSize.height = getOptimalDFTSize(mCoarseCartesianSize.height) & -2; 
    mPaddedEvenSize.width = getOptimalDFTSize(mCoarseCartesianSize.width) & -2;
    highpass(mPaddedEvenSize, mHighpassFilter);

    mCoarseCartesianFFT = cv::Mat(mPaddedEvenSize, CV_32F);
    mCoarseCartesianRotated = cv::Mat(mCoarseCartesianSize, CV_32F);

    mPhaseCorrPower = cv::Mat(mDFTSize, CV_32F);
    mPhaseCorrNormalised = cv::Mat(mDFTSize, CV_32F);

    cv::createHanningWindow(mCoarseCartesianHanningWindow, mCoarseCartesianSize, CV_32F);
    cv::createHanningWindow(mFineCartesianHanningWindow, mFineCartesianSize, CV_32F);
    if (mUseOriginal)
    {
        cv::createHanningWindow(mCoarseLogPolarHanningWindow, mPaddedEvenSize, CV_32F);
    }
    else
    {
        cv::createHanningWindow(mCoarseLogPolarHanningWindow, mCoarseCartesianSize, CV_32F);
    }
}

LogPolarFFTMatcher::~LogPolarFFTMatcher()
{
}

void
LogPolarFFTMatcher::initialise(cv::Mat& aImage, Frame& aFrame)
{
    aFrame.mCoarseCartesianImage = aImage;
    if (mUseOriginal) { convertFourierLogPolar(aFrame); }
    else { convertLogPolar(aFrame); }
}

Pose2D
LogPolarFFTMatcher::estimateCoarse(const Frame& aPrevFrame, Frame& aCurrFrame)
{
    if (mUseOriginal)
    {
        return estimateCoarseOriginal(aPrevFrame, aCurrFrame);
    }
    else
    {
        return estimateCoarseModified(aPrevFrame, aCurrFrame);
    }
    
}

Pose2D 
LogPolarFFTMatcher::estimateCoarseRotation(const Frame& aPrevFrame, const Frame& aCurrFrame)
{
    // Estimate rotation and scale
    // Point2d rotationAndScale = cv::phaseCorrelate(aCurrFrame.mCoarseLogPolarImage, aPrevFrame.mCoarseLogPolarImage, mCoarseLogPolarHanningWindow);
    cv::Point rsPeakLoc;
    Point2d rotationAndScale = PhaseCorrelation::phaseCorrelate(aCurrFrame.mCoarseLogPolarImage,
        aPrevFrame.mCoarseLogPolarImage, mCoarseLogPolarHanningWindow, mPhaseCorrPower, rsPeakLoc);
    double angleDeg = 180.f * (rotationAndScale.y - 0.5) / aPrevFrame.mCoarseLogPolarImage.rows;

    auto rsVariance = computeCorrelationVariance(rsPeakLoc);
    const double yawVar = 180.f * rsVariance.second / aPrevFrame.mCoarseLogPolarImage.rows;

    //! TODO: Store log_base somewhere so we can compute the scale?

    std::cout << "*** Rotation/scale: " << rotationAndScale.y << ", " << rotationAndScale.x << ", " << angleDeg << std::endl;

    //! TODO: Strict clamping to [-pi/2, pi/2). Should we expand this to [-pi, pi)?
    if (angleDeg < -90.0)
    {
        angleDeg += 180.0;
    }
    else if (angleDeg > 90.0)
    {
        angleDeg -= 180.0;
    }

    Pose2D tmp(0, 0, -Util::Angle::deg2Rad(angleDeg), aCurrFrame.mTimestamp);
    tmp.yawVar = yawVar;

    return tmp;
}

Pose2D
LogPolarFFTMatcher::estimateFineTranslation(const Frame& aPrevFrame, Frame& aCurrFrame)
{
    Point2d tr = cv::phaseCorrelate(aCurrFrame.mFineCartesianImage, 
        aPrevFrame.mFineCartesianImage, mFineCartesianHanningWindow);

    std::cout << "$$$ Translation: " << tr.x << "," << tr.y << std::endl;

    Pose2D tmp;
    tmp.x = (tr.x - 0.5) * mRadarRangeDiameter / static_cast<double>(mFineCartesianSize.width);
    tmp.y = (tr.y - 0.5) * mRadarRangeDiameter / static_cast<double>(mFineCartesianSize.height);
    tmp.yaw = 0;

    return tmp;
}

Pose2D
LogPolarFFTMatcher::estimateCoarseOriginal(const Frame& aPrevFrame, Frame& aCurrFrame)
{
    Mat F1[2];
    forwardFFT(aCurrFrame.mCoarseCartesianImage, F1);
    magnitude(F1[0], F1[1], mCoarseCartesianFFT);

    // Apply highpass filter in freq domain
    mCoarseCartesianFFT = mCoarseCartesianFFT.mul(mHighpassFilter);

    // Log-polar conversion
    float log_base = logpolar(mCoarseCartesianFFT, aCurrFrame.mCoarseLogPolarImage);

    // Estimate rotation and scale
    cv::Point rsPeakLoc;
    Point2d rotationAndScale = PhaseCorrelation::phaseCorrelate(aCurrFrame.mCoarseLogPolarImage, 
        aPrevFrame.mCoarseLogPolarImage, mCoarseLogPolarHanningWindow, mPhaseCorrPower, rsPeakLoc);
    double angleDeg = 180.f * (rotationAndScale.y - 0.5) / aPrevFrame.mCoarseLogPolarImage.rows;
    double scale = pow(log_base, rotationAndScale.x - 0.5);

    std::cout << "*** Rotation/scale: " << rotationAndScale.y << ", " << rotationAndScale.x << ", " << angleDeg << ", " << scale << std::endl;

    //const auto rsVariance = computeCorrelationVariance(rsPeakLoc);
    std::pair<double, double> rsVariance;
    const double yawVar = 180.f * rsVariance.second / aPrevFrame.mCoarseLogPolarImage.rows;

    //! TODO: Possibly use the scale to identify situations where rotation estimation fails?
    if (scale > mScaleThreshold)
    {
        std::cout << "Unexpectedly large scale in rotation estimation! Rotation estimate will be poor...";
    }

    //! TODO: Strict clamping to [-pi/2, pi/2). Should we expand this to [-pi, pi)?
    if (angleDeg < -90.0)
    {
        angleDeg += 180.0;
    }
    else if (angleDeg > 90.0)
    {
        angleDeg -= 180.0;
    }

    // Account for rotation in image
    Mat R = getRotationMatrix2D(mCoarseCartesianCentre, angleDeg, 1.0);
    warpAffine(aCurrFrame.mCoarseCartesianImage, mCoarseCartesianRotated, R, mCoarseCartesianSize);

    // Estimate translation
    cv::Point trPeakLoc;
    Point2d tr = PhaseCorrelation::phaseCorrelate(mCoarseCartesianRotated, 
        aPrevFrame.mCoarseCartesianImage, mCoarseCartesianHanningWindow, mPhaseCorrPower, trPeakLoc);
    
    std::cout << "*** Translation: " << tr.x << "," << tr.y << std::endl;

    const auto trVariance = computeCorrelationVariance(trPeakLoc);
    //std::pair<double, double> trVariance;
    const double xVar = trVariance.first * mRadarRangeDiameter / static_cast<double>(mCoarseCartesianSize.width);
    const double yVar = trVariance.second * mRadarRangeDiameter / static_cast<double>(mCoarseCartesianSize.height);

    std::cout << "Variance: " << xVar << "," << yVar << "," << Util::Angle::deg2Rad(yawVar) << std::endl;

    Pose2D tmp;
    tmp.x = (tr.x - 0.5) * mRadarRangeDiameter / static_cast<double>(mCoarseCartesianSize.width);
    tmp.y = (tr.y - 0.5) * mRadarRangeDiameter / static_cast<double>(mCoarseCartesianSize.height);
    tmp.yaw = -Util::Angle::deg2Rad(angleDeg);

    tmp.xVar = xVar;
    tmp.yVar = yVar;
    tmp.yawVar = Util::Angle::deg2Rad(yawVar);

    tmp.timestamp = aCurrFrame.mTimestamp;

    return tmp;
}

Pose2D
LogPolarFFTMatcher::estimateCoarseModified(const Frame& aPrevFrame, Frame& aCurrFrame)
{
    int warpFlags = cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS + cv::WARP_POLAR_LOG;
    cv::Point2d centre(static_cast<double>(mCoarseCartesianSize.width) / 2.f, 
        static_cast<double>(mCoarseCartesianSize.height) / 2.f);
    cv::warpPolar(aCurrFrame.mCoarseCartesianImage, aCurrFrame.mCoarseLogPolarImage, 
        mCoarseCartesianSize, centre, static_cast<double>(mCoarseCartesianSize.width) / 2.f, warpFlags);

    // float log_base = logpolar(aCurrFrame.mCoarseCartesianImage, aCurrFrame.mCoarseLogPolarImage);

    std::cout << "phase correlate" << std::endl;
    // Point2d rotationAndScale = PhaseCorrelation::phaseCorrelate(aCurrFrame.mCoarseLogPolarImage, aPrevFrame.mCoarseLogPolarImage, mCoarseLogPolarHanningWindow);
    Point2d rotationAndScale = cv::phaseCorrelate(aCurrFrame.mCoarseLogPolarImage, 
        aPrevFrame.mCoarseLogPolarImage, mCoarseLogPolarHanningWindow);
    double angleDeg = 180.f * (rotationAndScale.y) / aPrevFrame.mCoarseLogPolarImage.rows;
    
    std::cout << "*** Rotation/scale: " << rotationAndScale.y << ", " << rotationAndScale.x << ", " << angleDeg << ", " << std::endl;

    if (angleDeg < -90.0)
    {
        angleDeg += 180.0;
    }
    else if (angleDeg > 90.0)
    {
        angleDeg -= 180.0;
    }

    
    // Account for rotation in image
    Mat R = getRotationMatrix2D(mCoarseCartesianCentre, angleDeg, 1.0);
    warpAffine(aCurrFrame.mCoarseCartesianImage, mCoarseCartesianRotated, R, mCoarseCartesianSize);

    // Estimate translation
    Point2d tr = cv::phaseCorrelate(mCoarseCartesianRotated, 
        aPrevFrame.mCoarseCartesianImage, mCoarseCartesianHanningWindow);
    // Point2d tr = PhaseCorrelation::phaseCorrelate(mCoarseCartesianRotated,
    //     aPrevFrame.mCoarseCartesianImage, mCoarseCartesianHanningWindow);
    
    std::cout << "*** Translation: " << tr.x << "," << tr.y << std::endl;

    Pose2D tmp;
    tmp.x = (tr.x - 0.5) * mRadarRangeDiameter / static_cast<double>(mCoarseCartesianSize.width);
    tmp.y = (tr.y - 0.5) * mRadarRangeDiameter / static_cast<double>(mCoarseCartesianSize.height);
    tmp.yaw = Util::Angle::deg2Rad(angleDeg);

    return tmp;
}

std::pair<double, double>
LogPolarFFTMatcher::computeCorrelationVariance(const cv::Point& aPeakLocation)
{
    // Threshold at half max power according to sonar mosaicing paper
    float maxPower = mPhaseCorrPower.at<float>(aPeakLocation);
    float powerThreshold = 0.5 * maxPower;
    double varx, vary;
    int countNonZero = 0;

    if (maxPower < DBL_EPSILON)
    {
        std::cout << "Warning: Max power of phase correlation matrix close to 0!" << std::endl;
        return std::make_pair(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    }

    float invMaxPower = 1.f / maxPower;
    cv::multiply(mPhaseCorrPower, cv::Scalar(invMaxPower), mPhaseCorrNormalised);

    for (int y = 0; y < mPhaseCorrPower.rows; ++y)
    {
        for (int x = 0; x < mPhaseCorrPower.cols; ++x)
        {
            cv::Point pixel(x, y);
            const float power = mPhaseCorrPower.at<float>(pixel);
            if (power > powerThreshold)
            {
                const float pmf = mPhaseCorrNormalised.at<float>(pixel);
                const cv::Point diff = pixel - aPeakLocation;
                varx = pmf * diff.x * diff.x;
                vary = pmf * diff.y * diff.y;
                ++countNonZero;
            }
        }
    }

    if (countNonZero > 1)
    {
        varx /= static_cast<double>(countNonZero - 1);
        vary /= static_cast<double>(countNonZero - 1);

        std::cout << "Nonzero variance: " << varx << "," << vary << std::endl;
        std::cout << "Count nonzero: " << countNonZero << std::endl;
    }
    else
    {
        varx = vary = 0.0;
    }

     std::cout << "PC pixel shift: " << varx << "," << vary << std::endl;
     cv::imshow("Power", mPhaseCorrNormalised);

     cv::Mat tmp;
     cv::threshold(mPhaseCorrPower, tmp, powerThreshold, 1.0, cv::THRESH_BINARY);
     cv::cvtColor(tmp, tmp, cv::COLOR_GRAY2RGB);
     cv::circle(tmp, aPeakLocation, 3, cv::Scalar(0, 0, 255), cv::FILLED);
     cv::imshow("Thresholded", tmp);

    return std::make_pair(varx, vary);
}

void
LogPolarFFTMatcher::convertFourierLogPolar(Frame& aFrame)
{
    Mat F1[2];
    forwardFFT(aFrame.mCoarseCartesianImage, F1);
    magnitude(F1[0], F1[1], mCoarseCartesianFFT);

    // Apply highpass filter in freq domain
    mCoarseCartesianFFT = mCoarseCartesianFFT.mul(mHighpassFilter);

    // Log-polar conversion
    float log_base = logpolar(mCoarseCartesianFFT, aFrame.mCoarseLogPolarImage); 
}

void
LogPolarFFTMatcher::convertLogPolar(Frame& aFrame)
{
    int warpFlags = cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS + cv::WARP_POLAR_LOG;
    cv::Point2d centre(static_cast<double>(mCoarseCartesianSize.width) / 2.f, 
        static_cast<double>(mCoarseCartesianSize.height) / 2.f);
    cv::warpPolar(aFrame.mCoarseCartesianImage, aFrame.mCoarseLogPolarImage, 
        mCoarseCartesianSize, centre, static_cast<double>(mCoarseCartesianSize.width) / 2.f, warpFlags);
    // float log_base = logpolar(aFrame.mCoarseCartesianImage, aFrame.mCoarseLogPolarImage);
}

// Recombine image quarters
void 
LogPolarFFTMatcher::recomb(Mat &src, Mat &dst)
{
    int cx = src.cols >> 1;
    int cy = src.rows >> 1;
    Mat tmp;
    tmp.create(src.size(), src.type());
    src(Rect(0, 0, cx, cy)).copyTo(tmp(Rect(cx, cy, cx, cy)));
    src(Rect(cx, cy, cx, cy)).copyTo(tmp(Rect(0, 0, cx, cy)));
    src(Rect(cx, 0, cx, cy)).copyTo(tmp(Rect(0, cy, cx, cy)));
    src(Rect(0, cy, cx, cy)).copyTo(tmp(Rect(cx, 0, cx, cy)));
    dst = tmp;
}

// 2D Forward FFT
void 
LogPolarFFTMatcher::forwardFFT(Mat &Src, Mat *FImg, bool do_recomb)
{
    int M = getOptimalDFTSize(Src.rows);
    int N = getOptimalDFTSize(Src.cols);
    Mat padded;
    copyMakeBorder(Src, padded, 0, M - Src.rows, 0, N - Src.cols, BORDER_CONSTANT, Scalar::all(0));
    Mat planes[] = { Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F) };
    Mat complexImg;
    merge(planes, 2, complexImg);
    dft(complexImg, complexImg);
    split(complexImg, planes);
    planes[0] = planes[0](Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
    planes[1] = planes[1](Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));
    if (do_recomb)
    {
        recomb(planes[0], planes[0]);
        recomb(planes[1], planes[1]);
    }
    planes[0] /= float(M*N);
    planes[1] /= float(M*N);
    FImg[0] = planes[0].clone();
    FImg[1] = planes[1].clone();
}

// 2D inverse FFT
void 
LogPolarFFTMatcher::inverseFFT(Mat *FImg, Mat &Dst, bool do_recomb)
{
    if (do_recomb)
    {
        recomb(FImg[0], FImg[0]);
        recomb(FImg[1], FImg[1]);
    }
    Mat complexImg;
    merge(FImg, 2, complexImg);
    idft(complexImg, complexImg);
    split(complexImg, FImg);
    Dst = FImg[0].clone();
}

// Generate highpass filter
void 
LogPolarFFTMatcher::highpass(Size sz, Mat& dst)
{
    Mat a = Mat(sz.height, 1, CV_32FC1);
    Mat b = Mat(1, sz.width, CV_32FC1);

    float step_y = CV_PI / sz.height;
    float val = -CV_PI*0.5;

    for (int i = 0; i < sz.height; ++i)
    {
        a.at<float>(i) = cos(val);
        val += step_y;
    }

    val = -CV_PI*0.5;
    float step_x = CV_PI / sz.width;
    for (int i = 0; i < sz.width; ++i)
    {
        b.at<float>(i) = cos(val);
        val += step_x;
    }

    Mat tmp = a*b;
    dst = (1.0 - tmp).mul(2.0 - tmp);
}

// Log-polar conversion
float 
LogPolarFFTMatcher::logpolar(Mat& src, Mat& dst)
{
    float radii = src.cols;
    float angles = src.rows;
    Point2f center(src.cols / 2, src.rows / 2);
    float d = norm(Vec2f(src.cols - center.x, src.rows - center.y));
    float log_base = pow(10.0, log10(d) / radii);
    float d_theta = CV_PI / (float)angles;
    float theta = CV_PI / 2.0;
    float radius = 0;
    Mat map_x(src.size(), CV_32FC1);
    Mat map_y(src.size(), CV_32FC1);
    for (int i = 0; i < angles; ++i)
    {
        for (int j = 0; j < radii; ++j)
        {
            radius = pow(log_base, float(j));
            float x = radius * sin(theta) + center.x;
            float y = radius * cos(theta) + center.y;
            map_x.at<float>(i, j) = x;
            map_y.at<float>(i, j) = y;
        }
        theta += d_theta;
    }
    remap(src, dst, map_x, map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
    return log_base;
}

}