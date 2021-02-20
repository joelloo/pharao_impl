#ifndef INCLUDE_LOG_POLAR_FFT_HPP
#define INCLUDE_LOG_POLAR_FFT_HPP

#include "opencv2/opencv.hpp"
#include "entity/Pose2D.hpp"
#include "entity/Frame.hpp"
#include "utils/Angle.hpp"
#include "utils/ParameterList.h"

#include "registration/ScanMatching.hpp"
#include "registration/logpolar_fft/PhaseCorrelation.hpp"

namespace Pharao
{

class LogPolarFFTMatcher : public ScanMatcher
{
public:
    explicit LogPolarFFTMatcher(const Util::ParameterList& aParams);
    ~LogPolarFFTMatcher();

    Pose2D estimateCoarse(const Frame& aPrevFrame, Frame& aCurrFrame) override;
    Pose2D estimateCoarseRotation(const Frame& aPrevFrame, const Frame& aCurrFrame) override;
    Pose2D estimateFineTranslation(const Frame& aPrevFrame, Frame& aCurrFrame) override;

    void initialise(cv::Mat& aImage, Frame& aFrame) override;
    
private:
    Pose2D estimateCoarseOriginal(const Frame& aPrevFrame, Frame& aCurrFrame);
    Pose2D estimateCoarseModified(const Frame& aPrevFrame, Frame& aCurrFrame);

    std::pair<double, double> computeCorrelationVariance(const cv::Point& aPeakLocation);

    void convertFourierLogPolar(Frame& aFrame);
    void convertLogPolar(Frame& aFrame);

    void recomb(cv::Mat& src, cv::Mat& dst);
    void forwardFFT(cv::Mat& src, cv::Mat* FImg, bool doRecomb = true);
    void inverseFFT(cv::Mat* FImg, cv::Mat& dst, bool doRecomb = true);
    void highpass(cv::Size sz, cv::Mat& dst);
    float logpolar(cv::Mat& src, cv::Mat& dst);

    cv::Mat mCoarseLogPolarHanningWindow;
    cv::Mat mCoarseCartesianHanningWindow;
    cv::Mat mFineCartesianHanningWindow;

    cv::Mat mHighpassFilter;
    cv::Mat mCoarseCartesianFFT;
    cv::Mat mCoarseCartesianRotated;

    cv::Mat mPhaseCorrPower;
    cv::Mat mPhaseCorrNormalised;

    cv::Size mFineCartesianSize;
    cv::Size mCoarseCartesianSize;
    cv::Size mPaddedEvenSize;
    cv::Size mDFTSize;
    cv::Point2d mCoarseCartesianCentre;

    double mScaleThreshold;
    double mRadarRangeDiameter;

    bool mUseOriginal{ true };
};

}

#endif