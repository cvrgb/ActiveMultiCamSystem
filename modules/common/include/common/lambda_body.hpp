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

#ifndef COMMON_LAMBDA_BODY_HPP
#define COMMON_LAMBDA_BODY_HPP

#include <functional>
#include <opencv2/opencv.hpp>

namespace abv_core
{
    /**
     * https://answers.opencv.org/question/65800/how-to-use-lambda-as-a-parameter-to-parallel_for_/?answer=130691#post-id-130691
     */
    class LambdaBody : public cv::ParallelLoopBody
    {
    public:
        explicit LambdaBody(const std::function<void(const cv::Range &)> &body) { _body = body; }
        void operator()(const cv::Range &range) const override { _body(range); }

    private:
        std::function<void(const cv::Range &)> _body;
    };

}

#endif
