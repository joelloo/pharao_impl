#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED

#pragma comment(lib, "user32.lib")

#include <mutex>
#include <future>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <windows.h>

#include <Eigen/Dense>

#include "Odometry.hpp"
#include "entity/Pose2D.hpp"
#include "utils/ParameterList.h"
#include "utils/Stopwatch.h"
#include "utils/Visualiser.hpp"
#include "utils/parsing/PoseDataParser.hpp"
#include "utils/parsing/RadarDataParser.hpp"

/// \brief Returns an iterator pointing to the pose nearest to the given timestamp,
/// under the condition that the timestamp lies between the start and end of the pose
/// buffer. Returns poseBuffer.end() otherwise.
Pharao::PoseBuffer::iterator getNearestStamp(Pharao::PoseBuffer::iterator& aStart,
    Pharao::PoseBuffer& aPoseBuffer, const std::chrono::nanoseconds& aTimestamp)
{
    Pharao::PoseStamped tmp = std::make_pair(aTimestamp, Eigen::Matrix4d());
    auto it = std::lower_bound(aStart, aPoseBuffer.end(), tmp,
        [](const Pharao::PoseStamped& lhs, const Pharao::PoseStamped& rhs)
        {
            return lhs.first < rhs.first;
        });

    if (it == aPoseBuffer.end() || it == (aPoseBuffer.end() - 1))
    {
        // If given timestamp does not lie within the buffer, return poseBuffer.end()
        return aPoseBuffer.end();
    }
    else
    {
        // Look at enclosing timestamps on both sides to determine closer pose
        auto prevIt = it;
        it++;

        auto prevDur = aTimestamp - prevIt->first;
        auto nextDur = it->first - aTimestamp;

        if (nextDur < prevDur)
        {
            return it;
        }
        else
        {
            return prevIt;
        }
    }
}

int main(int argc, char** argv)
{
    std::string configFile;
    if (argc == 2)
    {
        configFile = argv[1];
    }
    else
    {
        configFile = "C:/autovision_ws/src/so_pharao/configuration_files/config.xml";
    }

    Util::ParameterList params;
    if (!Util::parseXmlFile(configFile, params, true))
    {
        std::cout << "Error parsing configuration file." << std::endl;
        return 1;
    }

    std::cout << ">>> Parameters" << std::endl;
    for (const auto& param : params) { std::cout << param.toString() << std::endl; }
    std::cout << ">>>" << std::endl;

    if (!params.hasParam("log_directory"))
    {
        std::cout << "Error: no log directory specified" << std::endl;
        return 1;
    }

    std::ofstream ofs("C:/Users/Joel/Desktop/odom_error.txt");

    std::string logDirectory = static_cast<std::string>(params["log_directory"]);
    std::cout << logDirectory << std::endl;
    std::string poseFile = logDirectory + "/global_pose.csv";
    std::string radarDirectory = logDirectory + "/polar";
    std::string playbackType = static_cast<std::string>(params["playback_type"]);
    const bool stepPlayback = (playbackType.compare("step") == 0);
    const int numSkip = static_cast<int>(params["num_skip_frames"]);

    Pharao::PoseBuffer poseBuffer;
    Pharao::MulRan::PoseDataParser::readPoseFileIntoBuffer(poseFile, poseBuffer);
    Pharao::MulRan::RadarDataParser radarDataReader(radarDirectory);

    if (poseBuffer.empty())
    {
        std::cout << "Warn: No poses buffered!" << std::endl;
    }
    else
    {
        radarDataReader.fastForward(poseBuffer.front().first);
        // radarDataReader.fastForward(std::chrono::nanoseconds(1567496070673831766));
    }

    Pharao::Odometry odom(params);

    std::chrono::nanoseconds currTs;
    cv::Mat image;
    Pharao::Pose2D latestPose, deltaPose;
    Pharao::Pose2D prevPose;
    Pharao::PoseBuffer::iterator lower = poseBuffer.begin();

    Pharao::PoseVisualiser vis;
    Eigen::Matrix3d gtPoseInitial;
    bool initialised = false;
    int skipCount = 0;

    std::cout << "Starting playback..." << std::endl;
    while (radarDataReader.getNextImage(currTs, image))
    {       
        if (!initialised)
        {
            std::cout << "Initialising... (" << currTs.count() << ")" << std::endl;

            auto it = getNearestStamp(lower, poseBuffer, currTs);
            if (it != poseBuffer.end())
            {
                // Initialise visualiser
                Eigen::Matrix3d R = it->second.block(0, 0, 3, 3);
                const double yaw = Util::Angle::getYaw(R);
                Eigen::Affine2d transform = Eigen::Translation2d(it->second(0, 3), it->second(1, 3))
                                            * Eigen::Rotation2D<double>(yaw);
                gtPoseInitial = transform.matrix();
                vis.init(150, 150, gtPoseInitial);

                // Initialise odometry with initial pose and initial radar image
                odom.update(currTs, image, latestPose, deltaPose);
                odom.setInitialPose(gtPoseInitial);

                initialised = true;
            }

            continue;
        }

        if (skipCount < numSkip)
        {
            ++skipCount;
            continue;
        }
        else
        {
            skipCount = 0;
        }

        Util::Stopwatch::tic();
        bool estimated = odom.update(currTs, image, latestPose, deltaPose);
        std::cout << "Took: " << Util::Stopwatch::toc() << std::endl;

        if (estimated)
        {
            auto it = getNearestStamp(lower, poseBuffer, currTs);

            if (it != poseBuffer.end())
            {
                std::cout << "Found matching INS " << it->first.count() << "(" << currTs.count() << ")" << std::endl;
                Eigen::Matrix3d R = it->second.block(0, 0, 3, 3);
                const double currYaw = Util::Angle::getYaw(R);

                const Pharao::Pose2D nextPose(it->second(0, 3), it->second(1, 3), currYaw);
                const double gtDeltaX = it->second(0, 3) - prevPose.x;
                const double gtDeltaY = it->second(1, 3) - prevPose.y;
                const double gtDeltaYaw = Util::Angle::difference(currYaw, prevPose.yaw);
                std::cout << "[Delta] GT: (" << gtDeltaX << "," << gtDeltaY << "," << gtDeltaYaw << ")  Est: " << deltaPose << std::endl;
                std::cout << "[Accum] GT: " << nextPose << "  Est: " << latestPose << std::endl;
                const double angleErr = Util::Angle::difference(deltaPose.yaw, gtDeltaYaw);
                // std::cout << "[Error] " << deltaPose.x - gtDeltaX << ", " << deltaPose.y - gtDeltaY << ", " << angleErr << "(" << Util::Angle::rad2Deg(angleErr) << ")" << std::endl;
                 std::cout << std::endl;

                ofs << gtDeltaX << "," << gtDeltaY << "," << gtDeltaYaw << "," << deltaPose.yaw << "," << angleErr << "," << Util::Angle::rad2Deg(angleErr) << "\n";

                vis.draw(latestPose.toMatrix(), Pharao::PoseVisualiser::PHASE_CORR);
                vis.draw(nextPose.toMatrix(), Pharao::PoseVisualiser::GT);

                prevPose = nextPose;
                lower = it;
            }
            else
            {
                std::cout << "No matching INS data found! " << currTs.count() << std::endl;
                vis.draw(latestPose.toMatrix(), Pharao::PoseVisualiser::PHASE_CORR);
            }
        }

        if (stepPlayback)
        {
            cv::waitKey(0);
            while (!GetAsyncKeyState(VK_RIGHT)) ;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else
        {
            cv::waitKey(1);
        }
    }

    ofs.close();

    return 0;
}