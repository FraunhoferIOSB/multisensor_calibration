// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef MULTISENSORCALIBRATION_CAMERAREFERENCEREGISTRATIONPARAMETERS_HPP
#define MULTISENSORCALIBRATION_CAMERAREFERENCEREGISTRATIONPARAMETERS_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// multisensor_calibration
#include "DynamicParameter.hpp"

namespace multisensor_calibration
{

struct CameraReferenceRegistrationParameters
{

    // limit board reprojection error in pnp
    DynamicParameter<bool>
      limit_single_board_rpj_error = DynamicParameter<bool>(
        false,
        "Use max maximum reprojection error to accept during calibration of a single target "
        "pose. If false, 'board_max_rpj_error' is ignored.");

    // maximum rpj error
    DynamicParameter<double>
      single_board_max_rpj_error = DynamicParameter<double>(
        5.0,
        "Limit for maximum reprojection error to accept during calibration of a single target "
        "pose. All calibrated poses, that exceed this limit are rejected.",
        0.001, 5.0);

    // minimum number of inlies
    DynamicParameter<int>
      single_board_min_inliers = DynamicParameter<int>(
        10,
        "Threshold for minimum number of inliers to accept during calibration of a single "
        "target pose. All calibrated poses, that do not reach this threshold are rejected.",
        4, 16);

    // eprojection upper bound for inliers
    DynamicParameter<double>
      pnp_inlier_rpj_error_limit = DynamicParameter<double>(
        8.0,
        "Limit for maximum reprojection error for which points are considered as RANSAC "
        "inliers during PnP.",
        0.00001, 10.0);

    /**
     * @brief Declare parameters as dynamic to be adjusted during runtime.
     *
     * @param[in] ipNode Pointer to node for which the parameters are to be declared.
     */
    void declareDynamic(rclcpp::Node* ipNode) const
    {
        DECLARE_PARAMETER(limit_single_board_rpj_error, ipNode)
        DECLARE_PARAMETER(single_board_max_rpj_error, ipNode)
        DECLARE_PARAMETER(single_board_min_inliers, ipNode)
        DECLARE_PARAMETER(pnp_inlier_rpj_error_limit, ipNode)
    }

    /**
     * @brief Try to set available parameter from given rclcpp::Parameter.
     *
     * This will try through all parameters in list and try to set value.
     *
     * @return True, if successful, i.e., if a parameter from the list has been updated.
     * False otherwise.
     */
    bool tryToSetParameter(const rclcpp::Parameter& iParameter)
    {
        SET_PARAMETER_FROM_RCLCPP_PARAM(limit_single_board_rpj_error, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(single_board_max_rpj_error, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(single_board_min_inliers, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(pnp_inlier_rpj_error_limit, iParameter)

        return false;
    }
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CAMERAREFERENCEREGISTRATIONPARAMETERS_HPP