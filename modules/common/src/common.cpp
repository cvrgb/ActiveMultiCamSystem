#include "common/common.hpp"
#include <fstream>

std::vector<std::string> splitString(std::string text, char delim)
{
    std::string line;
    std::vector<std::string> vec;
    std::stringstream ss(text);
    while (std::getline(ss, line, delim))
        vec.push_back(line);
    return vec;
}
namespace calibration_toolkit
{

    bool MultiCamRig::writeJSON(std::string path2JSON, std::string flow)
    {
        nlohmann::ordered_json json_out;
        // std::vector<std::string> flow_split = splitString(flow, '>');
        // // json
        // // json_["BE2"]["G"] = "1";
        // // nlohmann::json JSON_OUT;
        // // if (json_out[flow_split[0]].empty())

        // // json_out[flow_split[0]] = nullptr;
        // nlohmann::ordered_json json_tmp;
        // json_tmp[flow_split.back()] = 1;

        // json_out[flow_split[flow_split.size() - 2]] = json_tmp;

        write(json_out, flow, "K", K, 3, 3);
        write(json_out, flow, "D", D, 8, 1);
        write(json_out, flow, "X", X, 4, 4);
        //  std::cout << "abc\n";
        std::ofstream ofile(path2JSON);
        std::string JSON_dump = json_out.dump(4);
        ofile << JSON_dump << std::endl;
        return true;
    };

    bool MultiCamRig::readJSON(std::string path2JSON, std::string device_name)
    {
        // nlohmann::json JSON;
        std::ifstream file(path2JSON);
        file >> json;
        read(json, device_name, "K", &(K[0]), 3, 3);
        read(json, device_name, "D", &(D[0]), 8, 1);
        read(json, device_name, "X", &(X[0]), 4, 4);

        return true;
    };
    bool MultiCamRig::write(nlohmann::ordered_json &json,
                            std::string device_name, std::string mat_name,
                            cv::Mat *M, size_t row, size_t col)
    {
        for (int k = 0; k < NUM_OF_CAMERAS; k++)
        {
            // static const int arr[] = {16, 2, 77, 29};
            //  MM[0].data;
            // pgm_double.ptr<double>(0);
            if (!M[k].empty())
            {
                double *g = M[k].ptr<double>(0);
                std::vector<double> vec(g, g + M[k].rows * M[k].cols);
                json[device_name][mat_name][std::to_string(k)] = vec;
            }
            //  std::vector<double>
            //  mat_data()
        }
        // auto it = json[device_name][mat_name].begin();
        // while (it != json[device_name][mat_name].end())
        // {
        //     // std::cout << it.key() << std::endl;
        //     std::vector<double> mat_data;
        //     it.value().get_to(mat_data);
        // }
        return false;
    };
    // bool MultiCamRig::writeJSON(std::string path2JSON){};
    bool MultiCamRig::read(const nlohmann::ordered_json &json,
                           std::string device_name, std::string mat_name,
                           cv::Mat *M, size_t row, size_t col)
    {
        bool found = false;

        if (json.contains(device_name))
            if (json.at(device_name).contains(mat_name))
            {

                auto it = json[device_name][mat_name].begin();
                while (it != json[device_name][mat_name].end())
                {
                    // std::cout << it.key() << std::endl;
                    std::vector<double> mat_data;
                    it.value().get_to(mat_data);
                    if (row * col != mat_data.size() && col == 1)
                    {
                        cv::Mat mat(std::min(row, mat_data.size()), col, CV_64FC1, mat_data.data());
                        mat.copyTo(M[std::stoi(it.key())]);
                        mat.~Mat();
                    }
                    else
                    {
                        cv::Mat mat(row, col, CV_64FC1, mat_data.data());
                        mat.copyTo(M[std::stoi(it.key())]);
                        mat.~Mat();
                    }
                    found = true;
                    it++;
                } //  it.va
                //      // auto it_K = it.value().begin();
                //      while (it_K != it.value().end())
                //  {
                //  }
                //  if (json.at(device_name).at(mat_name).at(std::to_string(index))
                //      //     if (!json.at(device_name).at(mat_name).at(std::to_string(index)).empty())
                //      {
                //          json.at(device_name).at(mat_name).at(std::to_string(index)).get_to(abc);
                //          cv::Mat tmp(row, col, CV_64FC1, abc.data());
                //          tmp.copyTo(M);
                //          tmp.~Mat();
                //          found = true;
                //      }
                //  if (!json.at(device_name).empty())
                //      if (!json.at(device_name).empty())
            }
        return found;
    };

}