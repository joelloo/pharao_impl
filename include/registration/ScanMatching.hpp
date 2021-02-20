#ifndef INCLUDE_SCAN_MATCHING_HPP
#define INCLUDE_SCAN_MATCHING_HPP

#include "entity/Frame.hpp"
#include "entity/Pose2D.hpp"
#include "utils/ParameterList.h"

#include "opencv2/opencv.hpp"

namespace Pharao
{

class ScanMatcher
{
public:
    ScanMatcher();
    ScanMatcher(const Util::ParameterList& aParams);
    ~ScanMatcher();

    virtual Pose2D estimateCoarse(const Frame& aPrevFrame, Frame& aCurrFrame) = 0;
    virtual Pose2D estimateCoarseRotation(const Frame& aPrevFrame, const Frame& aCurrFRame) = 0;
    virtual Pose2D estimateFineTranslation(const Frame& aPrevFrame, Frame& aCurrFrame) = 0;

    virtual void initialise(cv::Mat& aImage, Frame& aFrame) = 0;
};

}

#endif