#include "registration/imreg_dft/image_registration.h"
#include <opencv2/core/eigen.hpp>

namespace Pharao
{

ImageRegistration::ImageRegistration(const cv::Mat &im) 
    : rows_(im.rows)
    , cols_(im.cols)
    , pix2dist_x_(1.f)
    , pix2dist_y_(1.f)
    , log_polar_size_(std::max(rows_, cols_))
    , imdft_(rows_, cols_)
    , imdft_logpolar_(log_polar_size_, log_polar_size_)
    , image_transforms_(rows_, cols_, log_polar_size_, log_polar_size_)
    , ScanMatcher()
{
    high_pass_filter_ = imdft_.getHighPassFilter();
    initialise(im);
}

ImageRegistration::ImageRegistration(const Util::ParameterList& aParams)
    : rows_(static_cast<int>(aParams["coarse_cartesian_height"]))
    , cols_(static_cast<int>(aParams["coarse_cartesian_width"]))
    , log_polar_size_(std::max(rows_, cols_))
    , imdft_(rows_, cols_)
    , imdft_logpolar_(log_polar_size_, log_polar_size_)
    , image_transforms_(rows_, cols_, log_polar_size_, log_polar_size_)
    , ScanMatcher(aParams)
{
    high_pass_filter_ = imdft_.getHighPassFilter();

    double range = static_cast<float>(aParams["radar_max_range_metres"]);
    pix2dist_x_ = 2.f * range / cols_;
    pix2dist_y_ = 2.f * range / rows_;
}

ImageRegistration::~ImageRegistration()
{
}

void 
ImageRegistration::initialise(const cv::Mat &im)
{
    processImage(im, im0_gray_, im0_logpolar_);
}

void 
ImageRegistration::initialise(cv::Mat& aImage, Frame& aFrame)
{
    processImage(aImage, im0_gray_, im0_logpolar_);
    aFrame.mCoarseCartesianImage = im0_gray_;
    aFrame.mCoarseLogPolarImage = im0_logpolar_;
}

Pose2D
ImageRegistration::estimateCoarse(const Frame& aPrevFrame, Frame& aCurrFrame)
{
    double rs_row, rs_col;
    double t_row, t_col;
    double scale, rotation;

    processImage(aCurrFrame.mCoarseCartesianImage, aCurrFrame.mCoarseLogPolarImage);

    imdft_logpolar_.phaseCorrelate(aCurrFrame.mCoarseLogPolarImage, 
        aPrevFrame.mCoarseLogPolarImage, rs_row, rs_col);
    image_transforms_.getScaleRotation(rs_row, rs_col, scale, rotation);
    image_transforms_.rotateAndScale(aPrevFrame.mCoarseCartesianImage, im0_rotated_, scale, rotation);

    imdft_.phaseCorrelate(aCurrFrame.mCoarseCartesianImage, im0_rotated_, t_row, t_col);

    Pose2D tmp;
    tmp.x = -t_col * pix2dist_x_;
    tmp.y = -t_row * pix2dist_y_;
    tmp.yaw = Util::Angle::deg2Rad(rotation);
    tmp.timestamp = aCurrFrame.mTimestamp;

    return tmp;
}

Pose2D 
ImageRegistration::estimateCoarseRotation(const Frame& aPrevFrame, const Frame& aCurrFrame)
{
    double rs_row, rs_col;
    double scale, rotation;

    imdft_logpolar_.phaseCorrelate(aCurrFrame.mCoarseLogPolarImage, 
        aPrevFrame.mCoarseLogPolarImage, rs_row, rs_col);
    image_transforms_.getScaleRotation(rs_row, rs_col, scale, rotation);

    return Pose2D(0, 0, Util::Angle::deg2Rad(rotation), aCurrFrame.mTimestamp);
}

Pose2D
ImageRegistration::estimateFineTranslation(const Frame& aPrevFrame, Frame& aCurrFrame)
{
    return Pose2D();
}

void ImageRegistration::registerImage(const cv::Mat &im, cv::Mat &registered_image, std::vector<double> &transform_params, bool display_images)
{
    double rs_row, rs_col;
    double t_row, t_col;
    double scale, rotation;

    processImage(im, im1_gray_,im1_logpolar_);

    imdft_logpolar_.phaseCorrelate(im1_logpolar_, im0_logpolar_, rs_row, rs_col);
    image_transforms_.getScaleRotation(rs_row, rs_col, scale, rotation);
    image_transforms_.rotateAndScale(im0_gray_, im0_rotated_, scale, rotation);

    transform_params[2] = rotation;
    transform_params[3] = scale;

    if (display_images)
    {
        cv::imshow("im0_rotated", im0_rotated_);
    }

    imdft_.phaseCorrelate(im1_gray_, im0_rotated_, t_row, t_col);
    image_transforms_.translate(im0_rotated_, registered_image, t_col, t_row); // x, y

    transform_params[0] = t_col;
    transform_params[1] = t_row;

    if (display_images)
    {
        cv::imshow("im0_registered", registered_image);
    }
}

void ImageRegistration::next()
{
    im1_logpolar_.copyTo(im0_logpolar_);
    im1_gray_.copyTo(im0_gray_);
}

cv::Mat ImageRegistration::getBorderMask()
{
    return image_transforms_.getBorderMask();
}

void ImageRegistration::processImage(const cv::Mat &im, cv::Mat &gray, cv::Mat &log_polar)
{
    im.convertTo(gray, CV_32F, 1.0/255.0);

    cv::Mat apodized;
    cv::Mat im_dft_cv;

    image_transforms_.apodize(gray, apodized);
    Eigen::MatrixXf im_dft = (imdft_.fftShift(imdft_.fft(apodized)).cwiseProduct(high_pass_filter_).cwiseAbs()).cast<float>();
    cv::eigen2cv(im_dft, im_dft_cv);
    image_transforms_.remapLogPolar(im_dft_cv, log_polar);
}

void
ImageRegistration::processImage(const cv::Mat& gray, cv::Mat& log_polar)
{
    cv::Mat apodized;
    cv::Mat im_dft_cv;

    image_transforms_.apodize(gray, apodized);
    Eigen::MatrixXf im_dft = (imdft_.fftShift(imdft_.fft(apodized)).cwiseProduct(high_pass_filter_).cwiseAbs()).cast<float>();
    cv::eigen2cv(im_dft, im_dft_cv);
    image_transforms_.remapLogPolar(im_dft_cv, log_polar);
}

cv::Mat 
ImageRegistration::getPreviousImage()
{
    return im0_gray_;
}

cv::Mat
ImageRegistration::getCurrentImage()
{
    return im1_gray_;
}

}