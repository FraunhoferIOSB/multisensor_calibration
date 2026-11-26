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

#ifndef MULTISENSORCALIBRATION_LIDARTARGETDETECTIONPARAMETERS_HPP
#define MULTISENSORCALIBRATION_LIDARTARGETDETECTIONPARAMETERS_HPP

// ROS
#include <rclcpp/rclcpp.hpp>

// multisensor_calibration
#include "DynamicParameter.hpp"

namespace multisensor_calibration
{

struct LidarTargetDetectionParameters
{
    //--- ENUM DECLARATION ---//

    enum ENormalEstSearchMethod
    {
        RADIUS_SEARCH = 0,
        NEAREST_NEIGHBOR_SEARCH
    };
    enum EIcpMethod
    {
        ICP = 0,
        PlaneICP,
        GICP
    };

    //--- MEMBER DECLARATION ---//

    DynamicParameter<double>
      max_range = DynamicParameter<double>(
        11.0,
        "Maximum range at which to filter the incomming point cloud prior to any processing. "
        "Any point with a range (absolute distance from sensor) larger than max_range will be "
        "discarded. Turn to 0 to switch off.",
        0.0, 20.0);

    // normal estimation search method
    DynamicParameter<int>
      normal_estimation_search_method = DynamicParameter<int>(
        1,
        "Select method to use for neighbor search."
        "\n\t0 = RADIUS_SEARCH,"
        "\n\t1 = NEAREST_NEIGHBOR_SEARCH",
        0, 1, 1);

    // normal estimation search radius
    DynamicParameter<double>
      normal_estimation_search_radius = DynamicParameter<double>(
        100.0,
        "Radius in which to search for neighbors.\n"
        "In case of 'RadiusSearch', this is a spatial extend.\n"
        "In case of 'NearestNeighborSearch', this represents the number of nearest "
        "neighbors (truncated to int).",
        0.001, 500.0);

    // region growing minimum cluster size
    DynamicParameter<int>
      region_growing_cluster_size_min = DynamicParameter<int>(
        130,
        "Minimum number of points a cluster needs to contain in order to be considered as "
        "valid inside the region growing.",
        10, 999);

    // region growing maximum cluster size
    DynamicParameter<int>
      region_growing_cluster_size_max = DynamicParameter<int>(
        30000,
        "Maximum number of points a cluster needs to contain in order to be considered as "
        "valid inside the region growing.",
        1000, 999999);

    // region growing number neighbors
    DynamicParameter<int>
      region_growing_number_neighbors = DynamicParameter<int>(
        30,
        "Number of neighbor points to consider during region growing.",
        1, 100);

    // region growing angle threshold
    DynamicParameter<double>
      region_growing_angle_thresh = DynamicParameter<double>(
        3.0,
        "Angle in degrees used as the allowable range for the normals deviation. "
        "If the deviation between points normals is less than the smoothness threshold "
        "then they are suggested to be in the same cluster.",
        0.1, 10.0);

    // region growing curvature threshold
    DynamicParameter<double>
      region_growing_curvature_thresh = DynamicParameter<double>(
        0.8,
        "Second criteria for the region growing.  If two points have a small normals deviation "
        "then the disparity between their curvatures is tested.",
        0.1, 10.0);

    // size filter minimum tolerance for width
    DynamicParameter<double>
      size_filter_width_min_tolerance = DynamicParameter<double>(
        0.05,
        "Tolerance (in m) of the minimum board width when filtering the clusters based on "
        "their size.",
        0.01, 0.5);

    // size filter maximum tolerance for width
    DynamicParameter<double>
      size_filter_width_max_tolerance = DynamicParameter<double>(
        0.1,
        "Tolerance (in m) of the maximum board width when filtering the clusters based on "
        "their size.",
        0.01, 0.5);

    // size filter minimum tolerance for height
    DynamicParameter<double>
      size_filter_height_min_tolerance = DynamicParameter<double>(
        0.05,
        "Tolerance (in m) of the minimum board height when filtering the clusters based on "
        "their size.",
        0.01, 0.5);

    // size filter maximum tolerance for height
    DynamicParameter<double>
      size_filter_height_max_tolerance = DynamicParameter<double>(
        0.5,
        "Tolerance (in m) of the maximum board height when filtering the clusters based on "
        "their size.",
        0.01, 0.5);

    // ransac distance threshold
    DynamicParameter<double>
      ransac_distance_thresh = DynamicParameter<double>(
        0.05,
        "Distance threshold (in m) from model for points to count as inliers during RANSAC.",
        0.01, 0.5);

    // ransac rotation variance
    DynamicParameter<double>
      ransac_rotation_variance = DynamicParameter<double>(
        1.0,
        "Maximum angle in rotation (in degrees) to be sampled when computing the new "
        "coefficients within RANSAC.",
        0.01, 180.0);

    // ransac translation variance
    DynamicParameter<double>
      ransac_translation_variance = DynamicParameter<double>(
        0.08,
        "Maximum distance in translation (in m) to be sampled when computing the new "
        "coefficients within RANSAC.",
        0.01, 1.0);

    // ransac maximum iterations
    DynamicParameter<int>
      ransac_max_iterations = DynamicParameter<int>(
        200,
        "Maximum number of iterations of the RANSAC detecting the calibration target.",
        1, 1000);

    // ransac optimize coefficients
    DynamicParameter<bool>
      ransac_optimize_coefficients = DynamicParameter<bool>(
        true,
        "Option to activate the optimization of the coefficients by means of ICP.");

    // target ICP variant
    DynamicParameter<int>
      target_icp_variant = DynamicParameter<int>(
        2,
        "Select ICP variant to use to optimize coefficients."
        "\n\t0 = ICP,"
        "\n\t1 = PlaneICP,"
        "\n\t2 = GICP",
        0, 2, 1);

    // target ICP maximum correspondacne distance
    DynamicParameter<double>
      target_icp_max_correspondence_distance = DynamicParameter<double>(
        0.1,
        "Maximum distance for ICP to search for point correspondences. "
        "Given as ratio with respect to shorter side of calibration target.",
        0.001, 1.0);

    // target ICP rotation tolerance
    DynamicParameter<double>
      target_icp_rotation_tolerance = DynamicParameter<double>(
        0.5,
        "Rotation tolerance for convergence check. Given in degrees.",
        0.001, 10.0);

    // target ICP translation tolerance
    DynamicParameter<double>
      target_icp_translation_tolerance = DynamicParameter<double>(
        0.001,
        "Translation tolerance for convergence check. Given in unit of the"
        "LiDAR point cloud, typically meters.",
        0.000001, 1.0);

    /**
     * @brief Declare parameters as dynamic to be adjusted during runtime.
     *
     * @param[in] ipNode Pointer to node for which the parameters are to be declared.
     */
    void declareDynamic(rclcpp::Node* ipNode) const
    {
        DECLARE_PARAMETER(max_range, ipNode)

        DECLARE_PARAMETER(normal_estimation_search_method, ipNode)
        DECLARE_PARAMETER(normal_estimation_search_radius, ipNode)

        DECLARE_PARAMETER(region_growing_cluster_size_min, ipNode)
        DECLARE_PARAMETER(region_growing_cluster_size_max, ipNode)
        DECLARE_PARAMETER(region_growing_number_neighbors, ipNode)
        DECLARE_PARAMETER(region_growing_angle_thresh, ipNode)
        DECLARE_PARAMETER(region_growing_curvature_thresh, ipNode)

        DECLARE_PARAMETER(size_filter_width_min_tolerance, ipNode)
        DECLARE_PARAMETER(size_filter_width_max_tolerance, ipNode)
        DECLARE_PARAMETER(size_filter_height_min_tolerance, ipNode)
        DECLARE_PARAMETER(size_filter_height_max_tolerance, ipNode)

        DECLARE_PARAMETER(ransac_distance_thresh, ipNode)
        DECLARE_PARAMETER(ransac_rotation_variance, ipNode)
        DECLARE_PARAMETER(ransac_translation_variance, ipNode)
        DECLARE_PARAMETER(ransac_max_iterations, ipNode)
        DECLARE_PARAMETER(ransac_optimize_coefficients, ipNode)

        DECLARE_PARAMETER(target_icp_variant, ipNode)
        DECLARE_PARAMETER(target_icp_max_correspondence_distance, ipNode)
        DECLARE_PARAMETER(target_icp_rotation_tolerance, ipNode)
        DECLARE_PARAMETER(target_icp_translation_tolerance, ipNode)
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
        SET_PARAMETER_FROM_RCLCPP_PARAM(max_range, iParameter)

        SET_PARAMETER_FROM_RCLCPP_PARAM(normal_estimation_search_method, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(normal_estimation_search_radius, iParameter)

        SET_PARAMETER_FROM_RCLCPP_PARAM(region_growing_cluster_size_min, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(region_growing_cluster_size_max, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(region_growing_number_neighbors, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(region_growing_angle_thresh, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(region_growing_curvature_thresh, iParameter)

        SET_PARAMETER_FROM_RCLCPP_PARAM(size_filter_width_min_tolerance, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(size_filter_width_max_tolerance, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(size_filter_height_min_tolerance, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(size_filter_height_max_tolerance, iParameter)

        SET_PARAMETER_FROM_RCLCPP_PARAM(ransac_distance_thresh, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(ransac_rotation_variance, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(ransac_translation_variance, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(ransac_max_iterations, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(ransac_optimize_coefficients, iParameter)

        SET_PARAMETER_FROM_RCLCPP_PARAM(target_icp_variant, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(target_icp_max_correspondence_distance, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(target_icp_rotation_tolerance, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(target_icp_translation_tolerance, iParameter)

        return false;
    }
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_LIDARTARGETDETECTIONPARAMETERS_HPP