/*
 * ActiveBinocularVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Chengzhe Zhou
 * Copyright (C) 2021 ActiveBionicVision Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef COMMON_IO_HPP
#define COMMON_IO_HPP
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
std::string getStrBetweenTwoStr(const std::string &s, const std::string &start_delim, const std::string &stop_delim)
{
    unsigned first_delim_pos = s.find_last_of(start_delim);
    unsigned end_pos_of_first_delim = first_delim_pos + start_delim.length();
    unsigned last_delim_pos = s.find(stop_delim);
    return s.substr(end_pos_of_first_delim, last_delim_pos - end_pos_of_first_delim);
};

namespace calibration_toolkit
{

    namespace io
    {

        int loadFiles(std::string path, std::string ext_, std::vector<std::string> &imgFiles_sort)
        {
            std::vector<cv::String> files_cv;
            std::vector<std::string> files, imgFiles;
            if (!cv::utils::fs::exists(path))
            {
                std::cout << "file " << path << " not exits" << std::endl;

                return -1;
            }

            cv::utils::fs::glob(path, "*" + ext_, files_cv, false);
            if (files_cv.size() == 0)
            {
                std::cout << "file " << path << " not exits" << std::endl;
                return -1;
            }
            else
                for (size_t i = 0; i < files_cv.size(); ++i)
                {
                    // std::cout << static_cast<std::string>(files_cv[i]) << std::endl;
                    files.push_back(static_cast<std::string>(files_cv[i]));
                }

            imgFiles.reserve(files.size());

            int maxId = -1;
            int minId = 10000000;
            for (size_t i = 0; i < files.size(); ++i)
            {
                std::string::size_type dotPos = files[i].find_last_of('.');
                std::string ext = files[i].substr(dotPos, files[i].length() - dotPos);
                if (ext == ext_)
                {
                    imgFiles.push_back(files[i]);

                    int id = std::stoi(getStrBetweenTwoStr(files[i], "/", ext));
                    if (id > maxId)
                        maxId = id;
                    if (id < minId)
                        minId = id;
                }
            }
            std::cout << minId << "\t" << maxId << std::endl;

            std::vector<int> aux;
            aux.resize(maxId - minId + 1, -1);
            for (size_t i = 0; i < imgFiles.size(); i++)
            {
                int id = std::stoi(getStrBetweenTwoStr(imgFiles[i], "/", ext_));
                // std::cout << id << std::endl;
                aux[id - minId] = i;
            }

            imgFiles_sort.clear();
            for (size_t i = 0; i < aux.size(); i++)
            {
                if (aux[i] >= 0)
                    imgFiles_sort.push_back(imgFiles[aux[i]]);
            }
            // for (int i = 0; i < imgFiles_sort.size(); ++i)
            //     std::cout << static_cast<std::string>(imgFiles_sort[i]) << std::endl;

            return 1;

            // imgFiles_sort.resize(imgFiles.size());
            // for (int i = 0; i < imgFiles.size(); i++)
            // {
            //     int id = std::stoi(getStrBetweenTwoStr(imgFiles[i], "/", ext));
            //     imgFiles_sort[id] = imgFiles[i];
            // }
            // for (int i = 0; i < imgFiles_sort.size(); i++)
            // std::cout << imgFiles_sort[i] << "\n";
        };
        int loadFiles(std::string path, std::string ext, std::vector<cv::Mat> &imgs, int interval = 1)
        {
            std::vector<std::string> imgFiles_sort;
            // std::cout << "################################################################################" << std::endl;
            if (loadFiles(path, ext, imgFiles_sort) < 0)
                return -1;
            // for (int i = 0; i < imgFiles_sort.size(); ++i)
            //     std::cout << static_cast<std::string>(imgFiles_sort[i]) << std::endl;
            imgs.clear();
            imgs.reserve(imgFiles_sort.size());

            for (size_t i = 0; i < imgFiles_sort.size(); i = i + interval)
            {
                // i = i + 3;
                cv::Mat show = cv::imread(imgFiles_sort[i]);
                if (!show.empty())
                    imgs.push_back(show.clone());
                // cv::imshow("aaa", show);
                // cv::waitKey(10);
            }

            // std::cout << imgs.size() << std::endl;

            return 1;
        }
    };

}

#endif