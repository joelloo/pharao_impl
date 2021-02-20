#ifndef INCLUDE_PHASE_CORRELATION_HPP
#define INCLUDE_PHASE_CORRELATION_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace Pharao
{

class PhaseCorrelation
{
public:
    // PhaseCorrelation();
    // ~PhaseCorrelation();

    static cv::Point2d phaseCorrelate(cv::InputArray& _src1, cv::InputArray& _src2, cv::InputArray& _window = cv::noArray(), 
        cv::InputOutputArray& _pcMatrix = cv::noArray(), cv::Point& peakLocation = cv::Point(), double* response = nullptr);

private:
    static void magSpectrums( cv::InputArray& _src, cv::OutputArray& _dst);
    static void divSpectrums( cv::InputArray _srcA, cv::InputArray _srcB, cv::OutputArray _dst, int flags, bool conjB);
    static void fftShift(cv::InputOutputArray _out);
    static cv::Point2d weightedCentroid(cv::InputArray _src, cv::Point peakLocation, cv::Size weightBoxSize, double* response);

    static void highpass(cv::Size sz, cv::Mat& dst);

};

}

#endif