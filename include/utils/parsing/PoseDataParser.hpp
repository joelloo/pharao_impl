#ifndef INCLUDE_POSE_DATA_PARSER_HPP
#define INCLUDE_POSE_DATA_PARSER_HPP

#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <utility>
#include <cstdlib>
#include <Eigen/Dense>

namespace Pharao
{

typedef std::pair<std::chrono::nanoseconds, Eigen::Matrix4d> PoseStamped;
typedef std::vector<PoseStamped> PoseBuffer;

namespace MulRan
{

static const int MULRAN_POSE_NUM_TOKENS = 13;

class PoseDataParser
{
public:
    static void readPoseFileIntoBuffer(const std::string& aPoseFile, PoseBuffer& aPoseBuffer)
    {
        std::ifstream ifs(aPoseFile);
        std::string line;

        while (std::getline(ifs, line))
        {
            std::istringstream iss(line);
            std::string val;
            std::vector<std::string> tmp;
            tmp.reserve(MULRAN_POSE_NUM_TOKENS);

            while (std::getline(iss, val, ','))
            {
                tmp.push_back(val);
            }

            if (tmp.size() != MULRAN_POSE_NUM_TOKENS)
            {
                std::cout << "Unknown number of tokens (" << tmp.size() << ") detected! Skipping line..." << std::endl;
                continue;
            }

            aPoseBuffer.push_back(PoseStamped());
            auto& pose = aPoseBuffer.back();

            pose.first = std::chrono::nanoseconds(std::stoull(tmp.at(0)));
            
            pose.second(0, 0) = std::atof(tmp.at(1).c_str());
            pose.second(0, 1) = std::atof(tmp.at(2).c_str());
            pose.second(0, 2) = std::atof(tmp.at(3).c_str());
            pose.second(0, 3) = std::atof(tmp.at(4).c_str());
            pose.second(1, 0) = std::atof(tmp.at(5).c_str());
            pose.second(1, 1) = std::atof(tmp.at(6).c_str());
            pose.second(1, 2) = std::atof(tmp.at(7).c_str());
            pose.second(1, 3) = std::atof(tmp.at(8).c_str());
            pose.second(2, 0) = std::atof(tmp.at(9).c_str());
            pose.second(2, 1) = std::atof(tmp.at(10).c_str());
            pose.second(2, 2) = std::atof(tmp.at(11).c_str());
            pose.second(2, 3) = std::atof(tmp.at(12).c_str());
            pose.second(3, 0) = 0;
            pose.second(3, 1) = 0;
            pose.second(3, 2) = 0;
            pose.second(3, 3) = 1;
        }
    }

};

}

}

#endif