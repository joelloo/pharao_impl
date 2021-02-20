#ifndef INCLUDE_POSE_VISUALISER_HPP
#define INCLUDE_POSE_VISUALISER_HPP

#include <cmath>
#include <opencv2/opencv.hpp>

#include "entity/Pose2D.hpp"
#include "utils/parsing/PoseDataParser.hpp"

namespace Pharao
{

class PoseVisualiser
{
public:
    enum PoseType
    {
        GT,
        PHASE_CORR,
    };

    PoseVisualiser()
    {        
    }

    ~PoseVisualiser()
    {
    }

    void init(const double aExtentX, const double aExtentY, const Eigen::Matrix3d& aInitTransform = Eigen::Matrix3d::Identity(),
        const int aDimX = 640, const int aDimY = 640, const int aMarkerSize = 2)
    {
        mMarkerSize = aMarkerSize;
        mDimX = aDimX;
        mDimY = aDimY;
        mExtentX = aExtentX;
        mExtentY = aExtentY;
        mPixelsPerMetreX = static_cast<double>(mDimX) / mExtentX;
        mPixelsPerMetreY = static_cast<double>(mDimY) / mExtentY;
        mMetresPerPixelX = 1.f / mPixelsPerMetreX;
        mMetresPerPixelY = 1.f / mPixelsPerMetreY;
        mCoordsBottomLeft = Eigen::Vector2d(-aExtentX / 2.f, -aExtentY / 2.f);
        mInitTransform = aInitTransform;

        mCanvas = cv::Mat(mDimX, mDimY, CV_8UC3, cv::Scalar(20, 20, 20));
        mCanvasTitle = "Pose Visualiser";

        cv::namedWindow(mCanvasTitle);
        cv::imshow(mCanvasTitle, mCanvas);
    }

    void draw(const Pose2D& aPose, const PoseType aType)
    {
        draw(aPose.toMatrix(), aType);
    }

    void draw(const Eigen::Matrix3d& aPose, const PoseType aType)
    {
        // std::cout << "Drawing..." << std::endl;
        Eigen::Matrix3d poseWorld = mInitTransform * aPose;
        cv::Scalar colour;
        switch (aType)
        {
        case PoseType::GT:
            colour = cv::Scalar(0, 255, 0);
            break;
        case PoseType::PHASE_CORR:
            colour = cv::Scalar(0, 255, 255);
            break;
        default:
            std::cout << "Unknown pose type in visualiser" << std::endl;
            break;
        }

        // Compute "pose" relative to bottom left corner of image
        Eigen::Vector2d relative = poseWorld.col(2).head(2) - mCoordsBottomLeft;

        // New pose lies outside current image, so we have to shift the image (our boundaries are
        // a bit smaller than the current image to prevent the drawn points from being cut up)
        double offsetX = 2 * mMetresPerPixelX;
        double offsetY = 2 * mMetresPerPixelY;

        if (relative.x() < offsetX|| relative.x() > mExtentX - offsetX || relative.y() < offsetY || relative.y() > mExtentY - offsetY)
        {
            // Pose lies outside current image, so make latest pose the new centre
            if (relative.x() < -mExtentX || relative.x() > 2 * mExtentX || relative.y() < -mExtentY || relative.y() > 2 * mExtentY)
            {
                // std::cout << "Making new centre" << std::endl;
                mCanvas = cv::Mat(mDimX, mDimY, CV_8UC3, cv::Scalar(20, 20, 20));
                cv::circle(mCanvas, cv::Point2d(static_cast<double>(mDimX) / 2.f, static_cast<double>(mDimY) / 2.f), mMarkerSize, colour, cv::FILLED);
                mCoordsBottomLeft = poseWorld.col(2).head(2) - Eigen::Vector2d(mExtentX * 0.5, mExtentY * 0.5);
            }
            // Otherwise, we need to copy some portion of the current image
            else
            {
                // std::cout << "Copying" << std::endl;

                // Compute the overlap ROI (shift the ROI a bit in the direction of the new 
                // point to prevent the drawn point from being cut off by the edge of the image)
                double xminOverlap = mCoordsBottomLeft.x();
                double xmaxOverlap = xminOverlap + mExtentX;
                double yminOverlap = mCoordsBottomLeft.y();
                double ymaxOverlap = yminOverlap + mExtentY;
                double xminNewImg = mCoordsBottomLeft.x();
                double yminNewImg = mCoordsBottomLeft.y();
                if (relative.x() < offsetX)
                {
                    xminOverlap = mCoordsBottomLeft.x();
                    xminNewImg = poseWorld(0, 2) - offsetX;
                    xmaxOverlap = xminNewImg + mExtentX;
                }
                else if (relative.x() > mExtentX - offsetX)
                {
                    xmaxOverlap = mCoordsBottomLeft.x() + mExtentX;
                    xminOverlap = poseWorld(0, 2) + offsetX - mExtentX;
                    xminNewImg = xminOverlap;
                }

                if (relative.y() < offsetY)
                {
                    yminOverlap = mCoordsBottomLeft.y();
                    yminNewImg = poseWorld(1, 2) - offsetY;
                    ymaxOverlap = yminNewImg + mExtentY;
                }
                else if (relative.y() > mExtentY - offsetY)
                {
                    ymaxOverlap = mCoordsBottomLeft.y() + mExtentY;
                    yminOverlap = poseWorld(1, 2) + offsetY - mExtentY;
                    yminNewImg = yminOverlap;
                }
                
                Eigen::Vector2d newImgCoordsBottomLeft(xminNewImg, yminNewImg);
                cv::Rect canvasRect = getRectWorld2Mat(xminOverlap, yminOverlap, xmaxOverlap, ymaxOverlap, mCoordsBottomLeft);
                cv::Rect newImgRect = getRectWorld2Mat(xminOverlap, yminOverlap, xmaxOverlap, ymaxOverlap, newImgCoordsBottomLeft);
                cv::Mat tmp(mDimX, mDimY, CV_8UC3, cv::Scalar(20, 20, 20));
                mCanvas(canvasRect).copyTo(tmp(newImgRect));

                // Update image and corner coordinates
                mCanvas = tmp;
                mCoordsBottomLeft = newImgCoordsBottomLeft;

                // Draw
                relative = poseWorld.col(2).head(2) - mCoordsBottomLeft;
                cv::circle(mCanvas, convertPointWorld2Mat(relative), mMarkerSize, colour, cv::FILLED);
            }
        }
        else
        {
            // std::cout << "Plotting point as usual" << std::endl;
            cv::circle(mCanvas, convertPointWorld2Mat(relative), mMarkerSize, colour, cv::FILLED);
        }

        cv::imshow(mCanvasTitle, mCanvas);
        // std::cout << "Done!" << std::endl;
    }

private:
    //  World coordinates           Mat coordinates
    //       ______ xmax, ymax            0,0 ______ 
    //      |      |                         |      |
    //      |      |                         |      |
    //      |______|                         |______|
    //  xmin, ymin                                 mDimX, mDimY

    // Converts a point in world coordinates into mat coordinates
    inline cv::Point2d convertPointWorld2Mat(const cv::Point2d& aCoords, const Eigen::Vector2d& aWorldBottomLeft)
    {
        return convertPointWorld2Mat(Eigen::Vector2d(aCoords.x, aCoords.y) - aWorldBottomLeft);
    }

    inline cv::Point2d convertPointWorld2Mat(const Eigen::Vector2d& aRelative)
    {
        return cv::Point2d(aRelative.x() * mPixelsPerMetreX,
                           static_cast<double>(mDimY) - aRelative.y() * mPixelsPerMetreY);
    }

    // Given the extent of a rectangle in world space, returns the ROI (clamped
    // within the dimensions of the current image)
    inline cv::Rect getRectWorld2Mat(const double aMinX, const double aMinY, 
        const double aMaxX, const double aMaxY, const Eigen::Vector2d& aWorldBottomLeft)
    {
        const double xmin = std::max(aMinX, aWorldBottomLeft.x());
        const double ymin = std::max(aMinY, aWorldBottomLeft.y());
        const double xmax = std::min(aMaxX, aWorldBottomLeft.x() + mExtentX);
        const double ymax = std::min(aMaxY, aWorldBottomLeft.y() + mExtentY);

        cv::Point2d topLeft = convertPointWorld2Mat(cv::Point2d(xmin, ymax), aWorldBottomLeft);
        cv::Point2d bottomRight = convertPointWorld2Mat(cv::Point2d(xmax, ymin), aWorldBottomLeft);
        return cv::Rect(topLeft, bottomRight);
    }

    cv::Mat mCanvas;
    std::string mCanvasTitle;

    int mDimX;
    int mDimY;
    int mMarkerSize;
    double mPixelsPerMetreX;
    double mPixelsPerMetreY;
    double mMetresPerPixelX;
    double mMetresPerPixelY;
    double mExtentX;
    double mExtentY;
    Eigen::Vector2d mCoordsBottomLeft;

    Eigen::Matrix3d mInitTransform;
};

}

#endif