/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/sensor_data_processing/CalibrationTargetSacModel.h"

// Std
#include <thread>

// PCL
#include <pcl/PolygonMesh.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/transforms.hpp>

// OpenCV
#include <opencv2/core/eigen.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/common.h"
#include "multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
template <typename PointT>
CalibrationTargetSacModel<PointT>::CalibrationTargetSacModel(const CalibrationTarget& target,
                                                             const PointCloudConstPtr& cloud,
                                                             bool random) :
  pcl::SampleConsensusModelPerpendicularPlane<PointT>(cloud, random),
  pWtaInlierCount_(new std::size_t)
{
    init(target);
}

//==================================================================================================
template <typename PointT>
CalibrationTargetSacModel<PointT>::CalibrationTargetSacModel(const CalibrationTarget& target,
                                                             const PointCloudConstPtr& cloud,
                                                             const std::vector<int>& indices,
                                                             bool random) :
  pcl::SampleConsensusModelPerpendicularPlane<PointT>(cloud, indices, random),
  pWtaInlierCount_(new std::size_t)
{
    init(target);
}

//==================================================================================================
template <typename PointT>
CalibrationTargetSacModel<PointT>::~CalibrationTargetSacModel()
{
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::clearIndices()
{
    if (indices_ != nullptr)
        indices_->clear();
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::setPoseGuess(const Eigen::Vector3f& iCenter,
                                                     const Eigen::Vector3f& iUp,
                                                     const Eigen::Vector3f& iNormal)
{
    pPoseGuess_.reset(new lib3d::Extrinsics(lib3d::Extrinsics::LOCAL_2_REF));
    CalibrationTarget::computePose(iCenter, iUp, iNormal, *pPoseGuess_);
    *pWtaInlierCount_ = 0;
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::setIncrementalPoseGuessUpdate(const bool iValue)
{
    isPoseGuessUpdateSet_ = iValue;
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::setRotationVariance(const double iValue)
{
    rotationVarianceAngle_ = std::max(0.0, std::min(iValue, 180.0));
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::setTranslationVariance(const double iValue)
{
    translationVarianceMeters_ = std::max(0.0, iValue);
}

//==================================================================================================
template <typename PointT>
bool CalibrationTargetSacModel<PointT>::computeModelCoefficients(
  const std::vector<int>& samples,
  Eigen::VectorXf& model_coefficients) const
{
    //--- check if correct number of sample points are available
    if (samples.size() != sample_size_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::CalibrationTargetSacModel"),
                     "%s: Invalid set of samples given (%lu)!", __PRETTY_FUNCTION__,
                     samples.size());
        return (false);
    }

    model_coefficients.setZero(model_size_);

    //--- PLANE PARAMETERIZATION

    pcl::Array4fMapConst p0 = input_->points[samples[0]].getArray4fMap();
    pcl::Array4fMapConst p1 = input_->points[samples[1]].getArray4fMap();
    pcl::Array4fMapConst p2 = input_->points[samples[2]].getArray4fMap();

    Eigen::Array4f p1p0 = p1 - p0;
    Eigen::Array4f p2p0 = p2 - p0;

    //--- Avoid some crashes by checking for collinearity here
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

    //--- if pose guess is set, use it to resample location and rotation
    if (pPoseGuess_ != nullptr)
    {
        // pose vectors decomposed from pose guess
        Eigen::Vector3f centerGuess, upVecGuess, normalVecGuess;
        CalibrationTarget::decomposePose(*pPoseGuess_, centerGuess, upVecGuess, normalVecGuess);

        //--- compute up-vector based on seed value rotated by max +- rotationVarianceAngle_
        float rand_angle_radian =
          static_cast<float>(
            ((utils::drawRandomFloat() * 2 * rotationVarianceAngle_) -
             rotationVarianceAngle_) *
            M_PI / 180.f);
        Eigen::Matrix3f rotMatrix = Eigen::AngleAxisf(rand_angle_radian,
                                                      model_coefficients.template head<3>())
                                      .toRotationMatrix();
        model_coefficients.block<3, 1>(4, 0) = rotMatrix * upVecGuess;

        //--- compute center point on plane in the radius of translationVarianceMeters_
        float rand_distance =
          static_cast<float>(utils::drawRandomFloat() * translationVarianceMeters_);
        rand_angle_radian =
          static_cast<float>(utils::drawRandomFloat() * 2 * M_PI);
        rotMatrix = Eigen::AngleAxisf(rand_angle_radian,
                                      model_coefficients.template head<3>())
                      .toRotationMatrix();
        model_coefficients.block<3, 1>(7, 0) = centerGuess +
                                               (rotMatrix * upVecGuess) * rand_distance;
    }

    //--- copy height and with from calibration target definition
    model_coefficients[10] = calibrationTarget_.boardSize.width;
    model_coefficients[11] = calibrationTarget_.boardSize.height;

    return (true);
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::selectWithinDistance(
  const Eigen::VectorXf& model_coefficients,
  const double threshold,
  std::vector<int>& inliers)
{
    //--- Check if the model is valid given the user constraints
    if (!isModelValid(model_coefficients))
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::CalibrationTargetSacModel"),
                     "%s: Model coefficients do not satisfy the model!", __PRETTY_FUNCTION__);
        inliers.clear();
        return;
    }

    int nr_p = 0;
    inliers.resize(indices_->size());
    error_sqr_dists_.resize(indices_->size());

    //--- compute pose based from model_coefficients
    lib3d::Extrinsics tmpPose;
    CalibrationTarget::computePose(model_coefficients.block<3, 1>(7, 0),
                                   model_coefficients.block<3, 1>(4, 0),
                                   model_coefficients.block<3, 1>(0, 0),
                                   tmpPose);

    //--- Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < indices_->size(); ++i)
    {
        // point in reference coordinate system, i.e. w.r.t. origin of sensor
        Eigen::Vector4f pnt(input_->points[(*indices_)[i]].x,
                            input_->points[(*indices_)[i]].y,
                            input_->points[(*indices_)[i]].z,
                            1);

        //--- Calculate the distance from the point to the plane normal as the dot product
        //--- D = (P-A).N/|N|
        float distance = std::abs(model_coefficients.block<4, 1>(0, 0).dot(pnt));

        //--- if distance of point is within threshold, check if it is an inlier
        if (distance <= threshold)
        {
            // local 2D point on target
            cv::Vec4d localPnt = tmpPose.getRTMatrix(lib3d::Extrinsics::REF_2_LOCAL) *
                                 cv::Vec4d(pnt(0),
                                           pnt(1),
                                           pnt(2),
                                           1.0);

            //--- check if point is inlier
            bool isInlier = calibrationTarget_.isLocalPointInlier(static_cast<float>(localPnt(0)),
                                                                  static_cast<float>(localPnt(1)),
                                                                  &distance);
            if (isInlier)
            {
                // Returns the indices of the points whose distances are smaller than the threshold
                inliers[nr_p]          = (*indices_)[i];
                error_sqr_dists_[nr_p] = static_cast<double>(distance);
                ++nr_p;
            }
        }
    }
    inliers.resize(nr_p);
    error_sqr_dists_.resize(nr_p);
}

//==================================================================================================
template <typename PointT>
std::size_t CalibrationTargetSacModel<PointT>::countWithinDistance(
  const Eigen::VectorXf& model_coefficients,
  const double threshold) const
{
    //--- Check if the model is valid given the user constraints
    if (!isModelValid(model_coefficients))
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::CalibrationTargetSacModel"),
                     "%s: Model coefficients do not satisfy the model!", __PRETTY_FUNCTION__);
        return 0;
    }

    float nr_p = 0.f;

    //--- compute pose based from model_coefficients
    lib3d::Extrinsics tmpPose;
    CalibrationTarget::computePose(model_coefficients.block<3, 1>(7, 0),
                                   model_coefficients.block<3, 1>(4, 0),
                                   model_coefficients.block<3, 1>(0, 0),
                                   tmpPose);

    //--- Iterate through the 3d points and calculate the distances from them to the plane
    for (std::size_t i = 0; i < indices_->size(); ++i)
    {
        // point in reference coordinate system, i.e. w.r.t. origin of sensor
        Eigen::Vector4f pnt(input_->points[(*indices_)[i]].x,
                            input_->points[(*indices_)[i]].y,
                            input_->points[(*indices_)[i]].z,
                            1);

        //--- Calculate the distance from the point to the plane normal as the dot product
        //--- D = (P-A).N/|N|
        float distance = std::abs(model_coefficients.block<4, 1>(0, 0).dot(pnt));

        //--- if distance of point is within threshold, check if it is an inlier
        if (distance <= threshold)
        {
            // local 2D point on target
            cv::Vec4d localPnt = tmpPose.getRTMatrix(lib3d::Extrinsics::REF_2_LOCAL) *
                                 cv::Vec4d(pnt(0),
                                           pnt(1),
                                           pnt(2),
                                           1.0);

            //--- check if point is inlier
            float penalty;
            bool isInlier = calibrationTarget_.isLocalPointInlier(static_cast<float>(localPnt(0)),
                                                                  static_cast<float>(localPnt(1)),
                                                                  &distance, &penalty);
            if (isInlier)
                nr_p += 1;
            else
                nr_p -= penalty;
        }
    }

    //--- round to nearest integer and clamp at 0
    std::size_t nInlier = std::max(0, static_cast<int>(std::round(nr_p)));

    //--- if incremental pose guess update is activated, check for wta solution and update
    //--- accordingly
    if (isPoseGuessUpdateSet_ && nInlier > *pWtaInlierCount_)
    {
        //--- only if wtaInlierCount_ is greater than 0, i.e. if it has already been initialized,
        //--- update the pose guess
        if (*pWtaInlierCount_ > 0)
        {
            *pPoseGuess_ = tmpPose;
        }

        *pWtaInlierCount_ = nInlier;
    }

    return nInlier;
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::optimizeModelCoefficients(
  const Eigen::VectorXf& model_coefficients,
  Eigen::VectorXf& optimized_coefficients) const
{
    std::vector<int> inliers(input_->size());
    std::iota(std::begin(inliers), std::end(inliers), 0);
    optimizeModelCoefficients(inliers, model_coefficients, optimized_coefficients);
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::optimizeModelCoefficients(
  const std::vector<int>& inliers,
  const Eigen::VectorXf& model_coefficients,
  Eigen::VectorXf& optimized_coefficients) const
{
    //--- Check if the model is valid given the user constraints
    if (!isModelValid(model_coefficients))
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::CalibrationTargetSacModel"),
                     "%s: Model coefficients do not satisfy the model! "
                     "Returning the same coefficients.",
                     __PRETTY_FUNCTION__);
        return;
    }

    //--- Need more than the minimum sample size to make a difference
    if (inliers.size() <= sample_size_)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::CalibrationTargetSacModel"),
                     "%s: Not enough inliers! Returning the same coefficients.",
                     __PRETTY_FUNCTION__);
        return;
    }

    //--- check if CAD model is available
    if (pCadModelCloud_ == nullptr)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::CalibrationTargetSacModel"),
                     "%s: Cloud of CAD model not available! Returning the same coefficients.",
                     __PRETTY_FUNCTION__);
        return;
    }

    //--- point cloud that is to be used
    typename pcl::PointCloud<PointT>::Ptr pInlierCloud(new pcl::PointCloud<PointT>);

    //--- extract inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input_);
    extract.setIndices(pcl::IndicesPtr(new pcl::Indices(inliers)));
    extract.setNegative(false);
    extract.filter(*pInlierCloud);

    //--- get transformation guess from model_coefficients
    Eigen::Vector3f center = model_coefficients.block<3, 1>(7, 0);
    Eigen::Vector3f up     = model_coefficients.block<3, 1>(4, 0);
    Eigen::Vector3f normal = model_coefficients.block<3, 1>(0, 0);
    Eigen::Vector3f right  = up.cross(normal);

    Eigen::Matrix4f transformationGuess(Eigen::Matrix4f::Identity());
    transformationGuess.block<3, 1>(0, 0) = right;
    transformationGuess.block<3, 1>(0, 1) = up;
    transformationGuess.block<3, 1>(0, 2) = normal;
    transformationGuess.block<3, 1>(0, 3) = center;

    typename pcl::PointCloud<PointT>::Ptr alignedCloud(new pcl::PointCloud<PointT>);
    Eigen::Matrix4f optimizedTransform;

    //--- compute convex hull of target cloud for ICP
    typename pcl::PointCloud<PointT>::Ptr pCHullCloud(new pcl::PointCloud<PointT>);
    typename pcl::ConvexHull<PointT> convexHull;
    convexHull.setInputCloud(pInlierCloud);
    convexHull.reconstruct(*pCHullCloud);

#if 0
    //--- initialize and run ICP
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputTarget(pCadModelCloud_);
    icp.setInputSource(pInlierCloud);
    icp.setMaxCorrespondenceDistance(0.1 * std::min(model_coefficients[10],
                                                    model_coefficients[11]));
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.align(*alignedCloud, transformationGuess.inverse());
    optimizedTransform = icp.getFinalTransformation().inverse();
#else

    //--- convert PCL point cloud to list of Eigen vectors.
    // TODO: the PCL interface of small_gicp requires pcl >=1.11. Change if other PCL version is
    // available
    std::vector<Eigen::Vector4f> target_points, source_points;
    for (PointT point : *pCadModelCloud_)
        target_points.push_back(Eigen::Vector4f(point.getVector4fMap()));
    for (PointT point : *pCHullCloud)
        source_points.push_back(Eigen::Vector4f(point.getVector4fMap()));

    //--- configure and run GICP
    Eigen::Isometry3d guess               = Eigen::Isometry3d::Identity();
    guess.matrix()                        = transformationGuess.inverse().cast<double>();
    small_gicp::RegistrationResult result = align(target_points, source_points,
                                                  guess,
                                                  icpSettings_);

    optimizedTransform = result.T_target_source.matrix().inverse().cast<float>();
#endif

#if 0
    { /// DEBUG VISUALIZATION

        pcl::transformPointCloud(*pInlierCloud, *alignedCloud, optimizedTransform.inverse());

        typename pcl::PointCloud<PointT>::Ptr srcInitTransformCloud(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*pCHullCloud, *srcInitTransformCloud, transformationGuess.inverse());

        // Original point cloud is white
        pcl::visualization::PointCloudColorHandlerCustom<PointT> refColor(
          pCadModelCloud_, 255, 255, 255);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> transfColor(
          srcInitTransformCloud, 180, 25, 25); // red
        pcl::visualization::PointCloudColorHandlerCustom<PointT> alignColor(
          alignedCloud, 25, 180, 25); // green

        boost::shared_ptr<pcl::visualization::PCLVisualizer> pViewer(
          new pcl::visualization::PCLVisualizer("DEBUG Viewer"));
        pViewer->addPointCloud<PointT>(pCadModelCloud_, refColor, "refCloud");
        pViewer->addPointCloud<PointT>(srcInitTransformCloud, transfColor, "srcCloud");
        pViewer->addPointCloud<PointT>(alignedCloud, alignColor, "alignedCloud");

        pViewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "refCloud");
        pViewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "alignedCloud");
        pViewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "srcCloud");

        pViewer->setShowFPS(false);
        while (!pViewer->wasStopped())
        {
            pViewer->spinOnce(50);
        }
        pViewer->close();
        pViewer.reset();

    } /// DEBUG VISUALIZATION
#endif

    //--- update model coefficients from final transformation
    optimized_coefficients = model_coefficients;
    optimized_coefficients.block<3, 1>(7, 0) =
      optimizedTransform.block<3, 1>(0, 3); // center
    optimized_coefficients.block<3, 1>(4, 0) =
      optimizedTransform.block<3, 1>(0, 1).normalized(); // up
    optimized_coefficients.block<3, 1>(0, 0) =
      optimizedTransform.block<3, 1>(0, 2).normalized(); // normal

    //--- update 4th plane parameter
    optimized_coefficients[3] = -1 * (optimized_coefficients.block<3, 1>(0, 0)           // normal
                                        .dot(optimized_coefficients.block<3, 1>(7, 0))); // center

    //--- Make sure it results in a valid model
    if (!isModelValid(optimized_coefficients))
    {
        RCLCPP_DEBUG(rclcpp::get_logger("multisensor_calibration::CalibrationTargetSacModel"),
                     "%s: Optimized SAC model of calibration target is invalid. "
                     "Using unoptimized model coefficients.",
                     __PRETTY_FUNCTION__);
        optimized_coefficients = model_coefficients;
    }
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::setIcpCorrespondenceDistance(const double& ratio)
{
    icpSettings_.max_correspondence_distance = ratio *
                                               std::min(calibrationTarget_.boardSize.width,
                                                        calibrationTarget_.boardSize.height);
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::setIcpRotationTolerance(const double& tolerance_deg)
{
    icpSettings_.rotation_eps = tolerance_deg * M_PI / 180.0;
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::setIcpTranslationTolerance(const double& tolerance)
{
    icpSettings_.translation_eps = tolerance;
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::setIcpVariant(
  const small_gicp::RegistrationSetting::RegistrationType& type)
{
    icpSettings_.type = type;
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::setInputCloud(const PointCloudConstPtr& cloud)
{
    this->clearIndices();
    pcl::SampleConsensusModel<PointT>::setInputCloud(cloud);
}

//==================================================================================================
template <typename PointT>
void CalibrationTargetSacModel<PointT>::init(const CalibrationTarget& target)
{

    model_name_  = "SampleConsensusModelCalibrationTarget";
    sample_size_ = 3;
    model_size_  = 12;

    calibrationTarget_         = target;
    pCadModelCloud_            = nullptr;
    pPoseGuess_                = nullptr;
    isPoseGuessUpdateSet_      = true;
    *pWtaInlierCount_          = 0;
    rotationVarianceAngle_     = 1.0;
    translationVarianceMeters_ = 0.08;

    if (!calibrationTarget_.cadModelCloudPath.empty())
    {
        pCadModelCloud_.reset(new pcl::PointCloud<PointT>);
        pcl::io::loadPLYFile(calibrationTarget_.cadModelCloudPath, *pCadModelCloud_);
    }

    icpSettings_.type                        = small_gicp::RegistrationSetting::GICP;
    icpSettings_.num_threads                 = std::thread::hardware_concurrency();
    icpSettings_.downsampling_resolution     = 0.01;
    icpSettings_.max_correspondence_distance = 0.1 * std::min(calibrationTarget_.boardSize.width,
                                                              calibrationTarget_.boardSize.height);
    icpSettings_.rotation_eps                = 0.5 * M_PI / 180.0;
    icpSettings_.translation_eps             = 0.001;
}

//==================================================================================================
template <typename PointT>
bool CalibrationTargetSacModel<PointT>::isModelValid(
  const Eigen::VectorXf& model_coefficients) const
{
    //--- check that model_coefficients is of correct size
    if (!pcl::SampleConsensusModel<PointT>::isModelValid(model_coefficients))
        return (false);

    //--- Check against angular constraint, if given
    if (eps_angle_ > 0.0)
    {
        Eigen::Vector3f coeff(model_coefficients[0], model_coefficients[1],
                              model_coefficients[2]);
        double angleDiff = std::abs(pcl::getAngle3D(axis_, coeff, true));
        angleDiff        = (std::min)(angleDiff, 180.0 - angleDiff);

        //--- Check whether the current plane model satisfies our angle threshold criterion with
        //--- respect to the given axis
        if (angleDiff > eps_angle_)
            return (false);
    }

    //--- check that length of right vector is 1 and that its perpendicular to normal vector
    Eigen::Vector3f normalVec(model_coefficients[0],
                              model_coefficients[1],
                              model_coefficients[2]);
    Eigen::Vector3f upVec(model_coefficients[4],
                          model_coefficients[5],
                          model_coefficients[6]);
    if (std::abs(upVec.norm() - 1.) > FLT_EPSILON && upVec.dot(normalVec) > FLT_EPSILON)
        return false;

    //--- check if dimensions are real
    float width  = model_coefficients[10];
    float height = model_coefficients[11];
    if (width <= 0 || height <= 0)
        return false;

    return (true);
}

} // namespace multisensor_calibration

template class multisensor_calibration::CalibrationTargetSacModel<multisensor_calibration::InputPointType>;