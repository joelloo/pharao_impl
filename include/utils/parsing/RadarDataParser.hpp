#ifndef INCLUDE_RADAR_DATA_PARSER_HPP
#define INCLUDE_RADAR_DATA_PARSER_HPP

#include <chrono>
#include <string>
#include <fstream>
#include <sstream>
#include <utility>
#include <exception>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

namespace Pharao
{

namespace MulRan
{

class RadarDataParser
{
public:
    RadarDataParser(const std::string& aLogDirectory = "")
        : mHasValidPaths(false)
    {
        if (!getFiles(aLogDirectory, ".png"))
        {
            std::cout << "Unable to get files from " << aLogDirectory << std::endl;
            throw std::exception("Bad log directory");
        }

        mImageScaleFactor = 1 / 255.f;
    }

    ~RadarDataParser()
    {
    }

    bool getNextImage(std::chrono::nanoseconds& aTimestamp, cv::Mat& aImage)
    {
        if (mHasValidPaths && mImagePathIter != mImagePaths.end())
        {
            aTimestamp = mImagePathIter->first;
            mRawImage = cv::imread(mImagePathIter->second, cv::IMREAD_GRAYSCALE);
            cv::rotate(mRawImage, mRawImage, cv::ROTATE_90_COUNTERCLOCKWISE);
            mRawImage.convertTo(aImage, CV_32F, mImageScaleFactor);
            ++mImagePathIter;       
            return true;
        }
        else
        {
            return false;
        }
    }

    void fastForward(std::chrono::nanoseconds& aStartTimestamp)
    {
        std::chrono::nanoseconds curr;
        while (next(curr) && curr < aStartTimestamp) ;
    }

    void reset()
    {
        mImagePathIter = mImagePaths.begin();
    }

private:
    bool next(std::chrono::nanoseconds& aTimestamp)
    {
        if (mHasValidPaths && mImagePathIter != mImagePaths.end())
        {
            aTimestamp = mImagePathIter->first;
            ++mImagePathIter;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool getFiles(const boost::filesystem::path& aRoot, const std::string& aExt)
    {
        mImagePaths.clear();
        const int extLen = aExt.size();

        if (!boost::filesystem::exists(aRoot) || !boost::filesystem::is_directory(aRoot)) { return false; }

        // Assumes the filesystem orders files in directory...
        boost::filesystem::recursive_directory_iterator it(aRoot);
        boost::filesystem::recursive_directory_iterator endit;

        while (it != endit)
        {
            if (boost::filesystem::is_regular_file(*it) && it->path().extension() == aExt) 
            {
                std::string pathStr = it->path().string();
                auto timestamp = std::chrono::nanoseconds(std::stoull(it->path().filename().string().substr(0, pathStr.size() - extLen)));
                mImagePaths.push_back(std::make_pair(timestamp, pathStr));
            }
            ++it;
        }

        if (mImagePaths.size() > 0)
        {
            mHasValidPaths = true;
            mImagePathIter = mImagePaths.begin();
        }

        return true;
    }

    typedef std::pair<std::chrono::nanoseconds, std::string> PathPair;

    bool mHasValidPaths;
    std::vector<PathPair> mImagePaths;
    std::vector<PathPair>::iterator mImagePathIter;
    
    cv::Mat mRawImage;
    double mImageScaleFactor;
};

}

}

#endif