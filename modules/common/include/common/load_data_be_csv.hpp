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

#ifndef ABV_CORE_UTILS_LOAD_DATA_BE_CSV_HPP
#define ABV_CORE_UTILS_LOAD_DATA_BE_CSV_HPP

#include <functional>
#include <random>
#include <opencv2/opencv.hpp>
#include "rapidcsv.h"
//#include "infraData.hpp"
#include <Eigen/Core>

static Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
static Eigen::IOFormat ExportFmt(Eigen::FullPrecision, 0, ", ", "\n", "", "", "", "");

namespace calibration_toolkit
{

    enum class RecordDataHeader_BECSV // "enum class" defines this as a scoped enumeration instead of a standard enumeration
    {
        id,
        timeStamp,
        motorData,
        imuData,
        jpg,
        png,
        bmp,
        ENCODER_RAD,
        ENCODER_DEG
    };

    struct Frame_BE
    {
        long id;
        long timeStamp;
        Eigen::VectorXd motorEncoder = Eigen::VectorXd::Constant(6, 0);
        std::string imageFile = "";
        cv::Mat image;
        void print()
        {
            std::cout << id << "\t";
            std::cout << timeStamp << "\t";
            std::cout << motorEncoder.transpose().format(HeavyFmt) << "\t";
            std::cout << imageFile << "\n";
        };
        cv::Mat imageL()
        {
            assert(!image.empty());
            return image.colRange(0, image.cols * 0.5);
        }
        cv::Mat imageR()
        {
            assert(!image.empty());
            return image.colRange(image.cols * 0.5, image.cols);
        }
        Eigen::VectorXd encoderR() { return Eigen::VectorXd(Eigen::Vector3d(motorEncoder(2), motorEncoder(0), motorEncoder(1))); };
        Eigen::VectorXd encoderL() { return Eigen::VectorXd(Eigen::Vector3d(motorEncoder(2 + 3), motorEncoder(0 + 3), motorEncoder(1 + 3))); };
    };

    /**
     * @brief For loading "0_data.csv" in recorded data
     */
    class RecordData_BECSV
    {
    public:
        RecordData_BECSV(const std::string &path, RecordDataHeader_BECSV imageExt = RecordDataHeader_BECSV::jpg)
        {
            _dataDir = path;
            _csvFile = _dataDir + "/0_data.csv";

            _csv.Load(_csvFile, rapidcsv::LabelParams(1, 0));
            _csvRaw.Load(_csvFile, rapidcsv::LabelParams(1, -1));

            _id = _csvRaw.GetColumn<long>(0);
            _idMax = *std::max_element(_id.begin(), _id.end());
            _idMin = *std::min_element(_id.begin(), _id.end());

            _ext = imageExt;
        };

        std::vector<size_t> random_ids(int len)
        {
            std::random_device r;
            std::seed_seq seed{r(), r(), r(), r(), r(), r(), r(), r()};
            std::mt19937 eng(seed);

            std::vector<size_t> random_ids_;
            random_ids_.reserve(idMax() - idMin() + 1);
            for (size_t i = idMin(); i <= idMax(); i++)
                if (idExists(i))
                    random_ids_.push_back(i);

            std::shuffle(random_ids_.begin(), random_ids_.end(), eng);
            auto start = random_ids_.begin() + 0;
            auto end = random_ids_.begin() + len;

            std::vector<size_t> result(len);
            std::copy(start, end, result.begin());
            return result;
        }

        bool idExists(long id) const
        {

            if (std::binary_search(_id.begin(), _id.end(), id))
                return true;
            else
                return false;
        }
        bool grabFrame(long id, int id_cam, Frame_BE &frame, RecordDataHeader_BECSV encoderUnit = RecordDataHeader_BECSV::ENCODER_DEG, bool loadImage = false) const
        {

            if (idExists(id))
            {
                frame.id = id;
                frame.timeStamp = timeStamp(id);
                frame.motorEncoder = motorEncoder(id, encoderUnit);
                frame.imageFile = imagePath(id, id_cam, _ext);
                if (loadImage)
                    frame.image = cv::imread(frame.imageFile);

                // std::cout << "       id " << i << "\t";
                // std::cout << "timeStamp " << data.timeStamp(i) << "\t";
                // std::cout << "    motor " << data.motorEncoder(i).transpose() << "\t";
                // std::cout << "    image " << frame.imageFile << "\n";
                return true;
            }
            return false;
        }
        // bool grabFrame(long id, int id_cam, Frame_BE &frame, RecordDataHeader_BECSV encoderUnit = RecordDataHeader_BECSV::ENCODER_DEG, bool loadImage = false,
        //                float minT = -10, float maxT = 10) const
        // {
        //     // std::cout << "       id " << frame.id << "\n";

        //     if (idExists(id))
        //     {
        //         frame.id = id;
        //         frame.timeStamp = timeStamp(id);
        //         frame.motorEncoder = motorEncoder(id, encoderUnit);
        //         frame.imageFile = imagePath(id, id_cam, _ext);

        //         if (loadImage)
        //         {

        //             if (id_cam != 3)
        //             {
        //                 frame.image = cv::imread(frame.imageFile);
        //                 if (frame.image.empty())
        //                     return false;
        //                 // std::cout << "       id " << frame.imageFile << "\t";
        //             }
        //             else
        //             {

        //                 cv::Mat infraImage, tempImage;
        //                 uchar *imageData = InfraData::readImage(frame.imageFile, 745039);

        //                 cv::Mat infCamTemp = cv::Mat(cv::Size(1280, 512), CV_8UC2, imageData);

        //                 InfraData::infraImageTrans(infCamTemp, infraImage, tempImage);
        //                 InfraData::T2BGR(tempImage, frame.image, minT, maxT);
        //                 // cv::Mat t_image;
        //                 // infCamTemp.copyTo(frame.image);
        //                 delete[] imageData;
        //             }
        //         }

        //         // std::cout << "timeStamp " << frame.timeStamp << "\t";
        //         // std::cout << "    motor " << frame.motorEncoder.transpose() << "\t";
        //         // std::cout << "    image " << frame.imageFile << "\n";
        //         return true;
        //     }
        //     return false;
        // }

        const std::vector<long> &id() const
        {
            return _id;
        }

        const long &idMax() const
        {
            return _idMax;
        }
        const long &idMin() const
        {
            return _idMin;
        }

        long timeStamp(long id) const
        {
            return _csv.GetCell<long>("timeStamp", std::to_string(id));
        }

        std::string imagePath(long id, RecordDataHeader_BECSV ext = RecordDataHeader_BECSV::jpg) const
        {

            switch (ext)
            {
            case RecordDataHeader_BECSV::png:
                return _dataDir + "/" + std::to_string(id) + ".png";
                break;
            case RecordDataHeader_BECSV::bmp:
                return _dataDir + "/" + std::to_string(id) + ".bmp";

            default:
                return _dataDir + "/" + std::to_string(id) + ".jpg";
                break;
            }
        }

        std::string imagePath(long id, int id_cam, RecordDataHeader_BECSV ext = RecordDataHeader_BECSV::jpg) const
        {

            std::string midfix = "/Cam_" + std::to_string(id_cam);

            if (id_cam != 3)
            {

                switch (ext)
                {
                case RecordDataHeader_BECSV::png:

                    return _dataDir + midfix + "/" + std::to_string(id) + ".png";
                    break;
                case RecordDataHeader_BECSV::bmp:
                    return _dataDir + midfix + "/" + std::to_string(id) + ".bmp";

                default:
                    //                    std::cout << _dataDir + midfix + "/" + std::to_string(id) + ".jpg\n";
                    return _dataDir + midfix + "/" + std::to_string(id) + ".jpg";
                    break;
                }
            }
            else
            {
                return _dataDir + midfix + "/" + std::to_string(id) + ".beInfra";
            }
        }

        Eigen::VectorXd motorEncoder(long id, RecordDataHeader_BECSV type = RecordDataHeader_BECSV::ENCODER_DEG) const
        {
            Eigen::VectorXd m = Eigen::VectorXd::Constant(6, 0);

            m(0) = _csv.GetCell<double>("motorData(0-RightPitch)", std::to_string(id));
            m(1) = _csv.GetCell<double>("motorData(1-RightRoll)", std::to_string(id));
            m(2) = _csv.GetCell<double>("motorData(2-RightYaw)", std::to_string(id));
            m(3) = _csv.GetCell<double>("motorData(3-LeftPitch)", std::to_string(id));
            m(4) = _csv.GetCell<double>("motorData(4-LeftRoll)", std::to_string(id));
            m(5) = _csv.GetCell<double>("motorData(5-LeftYaw)", std::to_string(id));

            if (type == RecordDataHeader_BECSV::ENCODER_RAD)
                m = m / 180.0 * M_PI;

            return m;
        }

        // void make_csv(std::string path2CSVTemplate, std::string path2Left, std::string path2Right)
        // {
        //     rapidcsv::Document tmpCSV;
        //     tmpCSV.Load(path2CSVTemplate, rapidcsv::LabelParams(1, 0));
        //     tmpCSV.InsertColumn()
        // }

    private:
        rapidcsv::Document _csv;
        rapidcsv::Document _csvRaw;
        std::vector<long> _id;
        std::string _csvFile;
        std::string _dataDir;
        long _idMax;
        long _idMin;
        RecordDataHeader_BECSV _ext = RecordDataHeader_BECSV::jpg;
    };

} /* namespace abv_core */

#endif
