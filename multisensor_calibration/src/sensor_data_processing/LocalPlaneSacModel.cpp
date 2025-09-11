/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/sensor_data_processing/LocalPlaneSacModel.h"

// OpenCV
#include <opencv2/core/eigen.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
template <typename PointT>
LocalPlaneSacModel<PointT>::LocalPlaneSacModel(const PointCloudConstPtr& cloud,
                                               bool random) :
  pcl::SampleConsensusModelPlane<PointT>(cloud, random),
  pPlaneCenter_(new Eigen::Vector3f(0.f, 0.f, 0.f)),
  planeRadius_(0.f),
  isCenterUpdateSet_(true),
  pWtaInlierCount_(new std::size_t),
  translationVarianceMeters_(0.08)
{
    model_name_  = "SampleConsensusModelLocalPlane";
    sample_size_ = 3;
    model_size_  = 8;

    *pWtaInlierCount_ = 0;
}

//==================================================================================================
template <typename PointT>
LocalPlaneSacModel<PointT>::LocalPlaneSacModel(const PointCloudConstPtr& cloud,
                                               const std::vector<int>& indices,
                                               bool random) :
  pcl::SampleConsensusModelPlane<PointT>(cloud, indices, random),
  pPlaneCenter_(new Eigen::Vector3f(0.f, 0.f, 0.f)),
  planeRadius_(0.f),
  isCenterUpdateSet_(true),
  pWtaInlierCount_(new std::size_t),
  translationVarianceMeters_(0.08)
{
    model_name_  = "SampleConsensusModelLocalPlane";
    sample_size_ = 3;
    model_size_  = 8;

    *pWtaInlierCount_ = 0;
}

//==================================================================================================
template <typename PointT>
LocalPlaneSacModel<PointT>::~LocalPlaneSacModel()
{
}

//==================================================================================================
template <typename PointT>
void LocalPlaneSacModel<PointT>::clearIndices()
{
    if (indices_ != nullptr)
        indices_->clear();
}

//==================================================================================================
template <typename PointT>
void LocalPlaneSacModel<PointT>::setCenter(const Eigen::Vector3f& iCenter)
{
    *pPlaneCenter_    = iCenter;
    *pWtaInlierCount_ = 0;
}

//==================================================================================================
template <typename PointT>
void LocalPlaneSacModel<PointT>::setRadius(const float& iRadius)
{
    planeRadius_      = iRadius;
    *pWtaInlierCount_ = 0;
}

//==================================================================================================
template <typename PointT>
void LocalPlaneSacModel<PointT>::setIncrementalCenterUpdate(const bool iValue)
{
    isCenterUpdateSet_ = iValue;
}

//==================================================================================================
template <typename PointT>
void LocalPlaneSacModel<PointT>::setTranslationVariance(const double iValue)
{
    translationVarianceMeters_ = std::max(0.0, iValue);
}

//==================================================================================================
template <typename PointT>
bool LocalPlaneSacModel<PointT>::computeModelCoefficients(
  const std::vector<int>& samples,
  Eigen::VectorXf& model_coefficients) const
{
    //--- check if correct number of sample points are available
    if (samples.size() != sample_size_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::LocalPlaneSacModel"),
                     "%s: Invalid set of samples given (%lu)!", __PRETTY_FUNCTION__, samples.size());
        return (false);
    }

    model_coefficients.setZero(model_size_);

    //--- PLANE PARAMETERIZATION

    pcl::Array4fMapConst p0 = input_->points[samples[0]].getArray4fMap();
    pcl::Array4fMapConst p1 = input_->points[samples[1]].getArray4fMap();
    pcl::Array4fMapConst p2 = input_->points[samples[2]].getArray4fMap();

    Eigen::Array4f p1p0 = p1 - p0;
    Eigen::Array4f p2p0 = p2 - p0;

    //--- Avoid some crashes by checking for colinearity here
    Eigen::Array4f dy1dy2 = p1p0 / p2p0;
    if ((dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1])) // Check for collinearity
        return (false);

    //--- Compute the plane coefficients from the 3 given points in a straightforward manner
    //--- calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
    model_coefficients[0] = p1p0[1] * p2p0[2] - p1p0[2] * p2p0[1];
    model_coefficients[1] = p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2];
    model_coefficients[2] = p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0];

    //--- Normalize plane normal
    model_coefficients.normalize();

    // ... + d = 0
    model_coefficients[3] = -1 * (model_coefficients.template head<4>().dot(p0.matrix()));

    //--- if center update is set, use it to resample center
    if (isCenterUpdateSet_)
    {
        // Direction vector in which to update center. Is perpendicular to normal vector by computing
        // the cross product between the normal vector and any other vector.
        Eigen::Vector3f centerUpdateVec =
          Eigen::Vector3f(model_coefficients[0], model_coefficients[1], model_coefficients[2])
            .cross(Eigen::Vector3f(model_coefficients[2], model_coefficients[1], model_coefficients[0]));

        //--- compute center point on plane in the radius of translationVarianceMeters_
        float rand_distance =
          static_cast<float>(utils::drawRandomFloat() * translationVarianceMeters_);
        float rand_angle_radian =
          static_cast<float>(utils::drawRandomFloat() * 2 * M_PI);
        Eigen::Matrix3f rotMatrix = Eigen::AngleAxisf(rand_angle_radian,
                                                      model_coefficients.template head<3>())
                                      .toRotationMatrix();
        model_coefficients.block<3, 1>(4, 0) = *pPlaneCenter_ +
                                               (rotMatrix * centerUpdateVec) * rand_distance;
    }
    else
    {
        model_coefficients.block<3, 1>(4, 0) = *pPlaneCenter_;
    }

    //--- set radius
    model_coefficients[7] = planeRadius_;

    return (true);
}

//==================================================================================================
template <typename PointT>
void LocalPlaneSacModel<PointT>::selectWithinDistance(
  const Eigen::VectorXf& model_coefficients,
  const double threshold,
  std::vector<int>& inliers)
{
    //--- Check if the model is valid given the user constraints
    if (!isModelValid(model_coefficients))
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::LocalPlaneSacModel"),
                     "%s: Model coefficients do not satisfy the model!", __PRETTY_FUNCTION__);
        inliers.clear();
        return;
    }

    int nr_p = 0;
    inliers.resize(indices_->size());
    error_sqr_dists_.resize(indices_->size());

    //--- Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < indices_->size(); ++i)
    {
        // point in reference coordinate system, i.e. w.r.t. origin of sensor
        Eigen::Vector4f pnt(input_->points[(*indices_)[i]].x,
                            input_->points[(*indices_)[i]].y,
                            input_->points[(*indices_)[i]].z,
                            1);

        //--- Calculate the orthogonal distance from the point to the plane as the dot product
        //--- D = (P-A).N/|N|
        float orthoDistance = std::abs(model_coefficients.block<4, 1>(0, 0).dot(pnt));

        //--- Calculate the euclidean distance between point and center of local plane
        float euclDistance = Eigen::Vector3f(
                               pnt.block<3, 1>(0, 0) - model_coefficients.block<3, 1>(4, 0))
                               .norm();

        //--- if distance of point is within threshold and euclidean distance is within radius,
        //--- add as inlier
        if (orthoDistance <= threshold && euclDistance <= model_coefficients[7])
        {
            // Returns the indices of the points whose distances are smaller than the threshold
            inliers[nr_p]          = (*indices_)[i];
            error_sqr_dists_[nr_p] = static_cast<double>(orthoDistance);
            ++nr_p;
        }
    }
    inliers.resize(nr_p);
    error_sqr_dists_.resize(nr_p);
}

//==================================================================================================
template <typename PointT>
std::size_t LocalPlaneSacModel<PointT>::countWithinDistance(
  const Eigen::VectorXf& model_coefficients,
  const double threshold) const
{
    //--- Check if the model is valid given the user constraints
    if (!isModelValid(model_coefficients))
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::LocalPlaneSacModel"),
                     "%s: Model coefficients do not satisfy the model!", __PRETTY_FUNCTION__);
        return 0;
    }

    float nr_p = 0.f;

    //--- Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < indices_->size(); ++i)
    {
        // point in reference coordinate system, i.e. w.r.t. origin of sensor
        Eigen::Vector4f pnt(input_->points[(*indices_)[i]].x,
                            input_->points[(*indices_)[i]].y,
                            input_->points[(*indices_)[i]].z,
                            1);

        //--- Calculate the orthogonal distance from the point to the plane as the dot product
        //--- D = (P-A).N/|N|
        float orthoDistance = std::abs(model_coefficients.block<4, 1>(0, 0).dot(pnt));

        //--- Calculate the euclidean distance between point and center of local plane
        float euclDistance = Eigen::Vector3f(
                               pnt.block<3, 1>(0, 0) - model_coefficients.block<3, 1>(4, 0))
                               .norm();

        //--- if distance of point is within threshold and euclidean distance is within radius,
        //--- add as inlier
        if (orthoDistance <= threshold && euclDistance <= model_coefficients[7])
            ++nr_p;
    }

    //--- round to nearest integer and clamp at 0
    std::size_t nInlier = std::max(0, static_cast<int>(std::round(nr_p)));

    //--- if incremental pose guess update is activated, check for wta solution and update
    //--- accordingly
    if (isCenterUpdateSet_ && nInlier > *pWtaInlierCount_)
    {
        //--- only if wtaInlierCount_ is greater than 0, i.e. if it has already been initialized,
        //--- update the pose guess
        if (*pWtaInlierCount_ > 0)
        {
            *pPlaneCenter_ = model_coefficients.block<3, 1>(4, 0);
        }

        *pWtaInlierCount_ = nInlier;
    }

    return nInlier;
}

//==================================================================================================
template <typename PointT>
void LocalPlaneSacModel<PointT>::optimizeModelCoefficients(
  const std::vector<int>& inliers,
  const Eigen::VectorXf& model_coefficients,
  Eigen::VectorXf& optimized_coefficients) const
{
    //--- Check if the model is valid given the user constraints
    if (!isModelValid(model_coefficients))
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::LocalPlaneSacModel"),
                     "%s: Model coefficients do not satisfy the model! "
                     "Returning the same coefficients.",
                     __PRETTY_FUNCTION__);
        return;
    }

    //--- Need more than the minimum sample size to make a difference
    if (inliers.size() <= sample_size_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::LocalPlaneSacModel"),
                     "%s: Not enough inliers! Returning the same coefficients.",
                     __PRETTY_FUNCTION__);
        return;
    }

    optimized_coefficients = model_coefficients;

    //--- Optimize only plane coefficients, copied from pcl::SampleConsensusModelPlane

    // Use Least-Squares to fit the plane through all the given sample points and find out its coefficients
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f xyz_centroid;

    computeMeanAndCovarianceMatrix(*input_, inliers, covariance_matrix, xyz_centroid);

    // Compute the model coefficients
    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
    pcl::eigen33(covariance_matrix, eigen_value, eigen_vector);

    // Hessian form (D = nc . p_plane (centroid here) + p)
    optimized_coefficients[0] = eigen_vector[0];
    optimized_coefficients[1] = eigen_vector[1];
    optimized_coefficients[2] = eigen_vector[2];
    optimized_coefficients[3] = 0;
    optimized_coefficients[3] = -1 * optimized_coefficients.block<4, 1>(0, 0).dot(xyz_centroid);

    //--- Make sure it results in a valid model
    if (!isModelValid(optimized_coefficients))
    {
        optimized_coefficients = model_coefficients;
    }
}

//==================================================================================================
template <typename PointT>
void LocalPlaneSacModel<PointT>::setInputCloud(const PointCloudConstPtr& cloud)
{
    this->clearIndices();
    pcl::SampleConsensusModel<PointT>::setInputCloud(cloud);
}

//==================================================================================================
template <typename PointT>
bool LocalPlaneSacModel<PointT>::isModelValid(
  const Eigen::VectorXf& model_coefficients) const
{
    //--- check that model_coefficients is of correct size
    if (!pcl::SampleConsensusModel<PointT>::isModelValid(model_coefficients))
        return (false);

    //--- check if radius are real
    if (model_coefficients[7] <= 0)
        return false;

    return (true);
}

} // namespace multisensor_calibration

template class multisensor_calibration::LocalPlaneSacModel<multisensor_calibration::InputPointType>;