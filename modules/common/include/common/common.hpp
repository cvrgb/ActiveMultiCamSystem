#ifndef COMMON_COMMON_HPP
#define COMMON_COMMON_HPP
#include <opencv2/opencv.hpp>
#include <string.h>
#include "json.hpp"

#define NUM_OF_CAMERAS 4
#define MAX_NUM_OF_JOINTS 3

namespace calibration_toolkit
{
    class MultiCamRig
    {
    public:
        cv::Mat K[NUM_OF_CAMERAS];
        cv::Mat D[NUM_OF_CAMERAS];
        cv::Mat Rt[NUM_OF_CAMERAS][NUM_OF_CAMERAS];
        cv::Mat H[NUM_OF_CAMERAS][NUM_OF_CAMERAS];
        cv::Mat X[NUM_OF_CAMERAS];
        MultiCamRig(){};
        ~MultiCamRig(){};
        bool readJSON(std::string path2JSON, std::string device_name);
        // bool readJSON(std::string path2JSON, std::string device_name);
        bool writeJSON(std::string path2JSON, std::string flow);

    private:
        nlohmann::ordered_json json;
        bool read(const nlohmann::ordered_json &json,
                  std::string device_name, std::string mat_name,
                  cv::Mat *M, size_t row, size_t col);
        bool write(nlohmann::ordered_json &json,
                   std::string device_name, std::string mat_name,
                   cv::Mat *M, size_t row, size_t col);
        // bool write(nlohmann::ordered_json &json,
        //            std::string device_name, std::string mat_name,
        //            cv::Mat *M, size_t row, size_t col);
    };

    // class Gimbal
    // {
    // public:
    //     Eigen::Matrix4d X[MAX_NUM_OF_JOINTS];

    //     Eigen::Matrix4d X[MAX_NUM_OF_JOINTS];
    // };

}
#endif