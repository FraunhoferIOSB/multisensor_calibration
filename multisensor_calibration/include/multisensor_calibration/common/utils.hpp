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

#ifndef MULTISENSORCALIBRATION_UTILS_H
#define MULTISENSORCALIBRATION_UTILS_H

// STD
#include <cstdint>
#define USE_MATH_DEFINES
#include <cmath>
#include <filesystem>
#include <functional>

// Eigen
#include <Eigen/SVD>

// ROS
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Transform.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// Lib3D
#include <multisensor_calibration/common/lib3D/core/extrinsics.hpp>
#include <multisensor_calibration/common/lib3D/core/intrinsics.hpp>

// multisensor_calibration
#include "../calibration_target/CalibrationTarget.hpp"
#include "common.h"

namespace multisensor_calibration
{
namespace utils
{
/************************************************************************************************/
/**
 * @brief Method to ask for binary user input given the question string. The question can be
 * passed as separate sting arguments.
 *
 * @param[in] iQuestionStrs Individual text strings for the question. Each string will then be
 * printed into separate line.
 * @param[in] iLogger Logger object used for output.
 *
 * @return True, if user answered with yes. False if user answered with no.
 */
inline bool askForBinaryUserInput(const std::vector<std::string> iQuestionStrs,
                                  const rclcpp::Logger& iLogger)
{
    bool answer = true;

    //--- construct question string from input args
    std::stringstream strStrm; // question string stream
    for (std::string str : iQuestionStrs)
    {
        strStrm << "\n\t> " << str;
    }

    //--- print question
    RCLCPP_INFO(iLogger, "%s (y/n)", strStrm.str().c_str());

    //--- get user input
    bool isUserInputValid = false; // flag to indicate valid user input
    while (!isUserInputValid)
    {
        std::string userInput = ""; // user input
        std::cin.clear();
        std::cin >> userInput;
        if (userInput == "y" ||
            userInput == "Y")
        {
            answer           = true;
            isUserInputValid = true;
        }
        else if (userInput == "n" ||
                 userInput == "N")
        {
            answer           = false;
            isUserInputValid = true;
        }
        else
        {
            std::cout << "\t> Invalid answer. Please answer with 'y' or 'n'." << std::endl;
            answer           = false;
            isUserInputValid = false;
        }
    }

    return answer;
}
/************************************************************************************************/
/**
 * @brief Method to average a set of quaternions.
 * The algorithm used is described here:
 * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
 *
 * @see https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
 *
 * @param[in] iQuaternions List of quaternions to be averaged.
 * @return Averaged quaternion.
 */
inline Eigen::Vector4f averageQuaternion(const std::vector<Eigen::Vector4f> iQuaternions)
{
    if (iQuaternions.size() == 0)
    {
        std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
        return Eigen::Vector4f::Zero();
    }

    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();

    for (uint q = 0; q < iQuaternions.size(); ++q)
        A += iQuaternions[q] * iQuaternions[q].transpose();

    // normalize with the number of quaternions
    A /= iQuaternions.size();

    // Compute the SVD of this 4x4 matrix
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXf singularValues = svd.singularValues();
    Eigen::MatrixXf U              = svd.matrixU();

    // find the eigen vector corresponding to the largest eigen value
    int largestEigenValueIndex = 0;
    float largestEigenValue    = 0;
    bool first                 = true;

    for (int i = 0; i < singularValues.rows(); ++i)
    {
        if (first)
        {
            largestEigenValue      = singularValues(i);
            largestEigenValueIndex = i;
            first                  = false;
        }
        else if (singularValues(i) > largestEigenValue)
        {
            largestEigenValue      = singularValues(i);
            largestEigenValueIndex = i;
        }
    }

    Eigen::Vector4f average;
    average(0) = U(0, largestEigenValueIndex);
    average(1) = U(1, largestEigenValueIndex);
    average(2) = U(2, largestEigenValueIndex);
    average(3) = U(3, largestEigenValueIndex);

    return average;
}
/************************************************************************************************/
/**
 * @brief Method to backup file with given file path. This will rename the file, appending the date
 * and time of last modification to file name.
 *
 * @param[in] iFilePath Absolute path ot file. It is assumed that the file exists.
 */
inline void backupFile(const std::filesystem::path& iFilePath)
{
    using namespace std::filesystem;
    using namespace std::chrono;

    //--- get last modification date of file
    //-- https://stackoverflow.com/questions/56788745/how-to-convert-stdfilesystemfile-time-type-to-a-string-using-gcc-9
    std::time_t modTime =
      system_clock::to_time_t(
        time_point_cast<system_clock::duration>(last_write_time(iFilePath) -
                                                file_time_type::clock::now() +
                                                system_clock::now()));

    //--- construct file path of backup file
    std::stringstream backupFilePathStrS;
    backupFilePathStrS << canonical(iFilePath.parent_path()).string()
                       << path::preferred_separator
                       << iFilePath.stem().string()
                       << std::put_time(std::localtime(&modTime), "_%F_%T")
                       << iFilePath.extension().string();
    path backupFilePath = path(backupFilePathStrS.str());

    //--- rename file
    rename(iFilePath, backupFilePath);
}
/************************************************************************************************/
/**
 * @brief Calculate mean reprojection error for given object points w.r.t. given image points
 *
 * @param[in] iImagePnts Image points to be used as reference.
 * @param[in] iObjectPnts Object points that are to be projected. Need to be of same size and in same
 * order as the image points given in iImagePnts.
 * @param[in] iCameraMatrix Intrinsic camera matrix.
 * @param[in] iDistCoeffs Distortion coefficients.
 * @param[in] iExtrRotVec Rodrigues vector of extrinsic camera pose.
 * @param[in] iExtrTransVec Translation vector of extrinsic camera pose.
 * @param[in] iIndices List of indices for which to compute the reprojection error within iImagePnts and
 * iObjectPnts. If empty, the rpj error will be computed for all points in list.
 * @return Mean reprojection error.
 */
inline double calculateMeanReprojectionError(const std::vector<cv::Point2f> iImagePnts,
                                             const std::vector<cv::Point3f> iObjectPnts,
                                             const cv::Matx33d iCameraMatrix,
                                             const cv::Mat iDistCoeffs,
                                             const cv::Vec3d iExtrRotVec,
                                             const cv::Vec3d iExtrTransVec,
                                             const std::vector<int> iIndices = {})
{
    assert(iImagePnts.size() == iObjectPnts.size());

    //--- reproject object point into image
    std::vector<cv::Point2f> reprojectedImgPnts;
    cv::projectPoints(iObjectPnts, iExtrRotVec, iExtrTransVec, iCameraMatrix, iDistCoeffs,
                      reprojectedImgPnts);

    // calculated reprojection error
    double reprojectionError = 0;

    //--- if index list is empty iterate over all points, if not iterate over indices
    if (iIndices.empty())
    {
        for (std::vector<cv::Point2f>::const_iterator reprojItr = reprojectedImgPnts.begin(),
                                                      refItr    = iImagePnts.begin();
             (reprojItr != reprojectedImgPnts.end() &&
              refItr != iImagePnts.end());
             ++reprojItr, ++refItr)
        {
            reprojectionError += cv::norm(cv::Mat(*reprojItr), cv::Mat(*refItr));
        }
        reprojectionError /= reprojectedImgPnts.size();
    }
    else
    {
        for (std::vector<int>::const_iterator idxItr = iIndices.begin();
             idxItr != iIndices.end(); ++idxItr)
        {
            reprojectionError += cv::norm(cv::Mat(reprojectedImgPnts[*idxItr]),
                                          cv::Mat(iImagePnts[*idxItr]));
        }
        reprojectionError /= iIndices.size();
    }

    //--- calculate mean and return
    return reprojectionError;
}
/************************************************************************************************/
/**
 * @brief Calculate mean translational difference and mean angular error between the calibration
 * targets observed by the LiDAR and the camera. This will transform the poses observed by the camera
 * into the LiDAR coordinate system and calculate the mean difference vectors of the center points
 * as well as the mean angular error of the up vector.
 *
 * @param[in] iRefBoardPoses List of poses (i.e. up vector and center) of calibration targets
 * observed in reference coordinate system.
 * @param[in] iSrcBoardPoses List of poses (i.e. up vector and center) of calibration targets
 * observed in source coordinate system
 * @param[in] iRTSrc2Ref RT matrix transforming points from the source coordinate system into the
 * reference coordinate system.
 * @param[out] oMeanTranslDifference Mean translational x-y-z difference vector (in meters).
 * @param[out] oMeanAngularError Mean angular error (in degrees).
 */
inline void calculateMeanBoardPoseDifference(
  const std::vector<std::pair<cv::Vec3d, cv::Vec3d>>& iRefBoardPoses,
  const std::vector<std::pair<cv::Vec3d, cv::Vec3d>>& iSrcBoardPoses,
  const cv::Matx44d& iRTSrc2Ref,
  cv::Vec3d& oMeanTranslDifference,
  double& oMeanAngularError)
{
    // iterators of board poses observed in reference and source coordinate system
    std::vector<std::pair<cv::Vec3d, cv::Vec3d>>::const_iterator refPoseIter, srcPoseIter;

    //--- reset output values
    oMeanTranslDifference = cv::Vec3d(0, 0, 0);
    oMeanAngularError     = 0;

    //--- loop over board poses in lidar and camera
    int counter = 0;
    for (refPoseIter = iRefBoardPoses.begin(), srcPoseIter = iSrcBoardPoses.begin();
         (refPoseIter != iRefBoardPoses.end() && srcPoseIter != iSrcBoardPoses.end());
         ++refPoseIter, ++srcPoseIter)
    {
        cv::Vec3d refUp     = cv::normalize(refPoseIter->first); // up vector of target board observed by reference sensor
        cv::Vec3d refCenter = refPoseIter->second;               // center of target board observed by reference sensor

        cv::Vec3d srcCenter  = srcPoseIter->second;            // center of target board observed in local coordinate system
        cv::Vec3d srcUpPoint = srcCenter + srcPoseIter->first; // point on the target up-wards of local center

        //--- transform source pose into reference coordinate system

        // source center of target board observed, transformed into reference coordinate system
        cv::Vec4d transfSrcCenter = iRTSrc2Ref * cv::Vec4d(srcCenter(0),
                                                           srcCenter(1),
                                                           srcCenter(2),
                                                           1.0);

        // point on the target up-wards of source center, transformed into reference coordinate system
        cv::Vec4d transfSrcUpPoint = iRTSrc2Ref * cv::Vec4d(srcUpPoint(0),
                                                            srcUpPoint(1),
                                                            srcUpPoint(2),
                                                            1.0);

        // source up vector of target board, transformed into reference coordinate system
        cv::Vec3d transfSrcUp = cv::normalize(cv::Vec3d(transfSrcUpPoint(0),
                                                        transfSrcUpPoint(1),
                                                        transfSrcUpPoint(2)) -
                                              cv::Vec3d(transfSrcCenter(0),
                                                        transfSrcCenter(1),
                                                        transfSrcCenter(2)));

        //--- calculate translational difference / angular error
        oMeanTranslDifference += (cv::Vec3d(transfSrcCenter(0), transfSrcCenter(1), transfSrcCenter(2)) -
                                  refCenter);
        oMeanAngularError += ((std::acos(refUp.dot(transfSrcUp)) / M_PI) * 180.0);

        counter++;
    }

    //--- normalize to compute mean
    oMeanTranslDifference /= counter;
    oMeanAngularError /= counter;
}
/************************************************************************************************/
/**
 * @brief Calculate mean translational difference and mean angular error between the calibration
 * targets observed by the LiDAR and the camera. This will transform the poses observed by the camera
 * into the LiDAR coordinate system and calculate the mean difference vectors of the center points
 * as well as the mean angular error of the up vector.
 *
 * @param[in] iRefBoardPoses List of extrinsic poses of calibration targets observed in reference
 * coordinate system.
 * @param[in] iSrcBoardPoses List of extrinsic poses of calibration targets observed in source
 * coordinate system
 * @param[in] iRTSrc2Ref RT matrix transforming points from the source coordinate system into the
 * reference coordinate system.
 * @param[out] oMeanTranslDifference Mean translational x-y-z difference vector (in meters).
 */
inline void calculateMeanBoardPoseDifference(
  const std::vector<lib3d::Extrinsics>& iRefBoardPoses,
  const std::vector<lib3d::Extrinsics>& iSrcBoardPoses,
  const cv::Matx44d& iRTSrc2Ref,
  cv::Vec3d& oMeanTranslDifference)
{
    // iterators of board poses observed in reference and source coordinate system
    std::vector<lib3d::Extrinsics>::const_iterator refPoseIter, srcPoseIter;

    //--- reset output values
    oMeanTranslDifference = cv::Vec3d(0, 0, 0);

    //--- loop over board poses in lidar and camera
    int counter = 0;
    for (refPoseIter = iRefBoardPoses.begin(), srcPoseIter = iSrcBoardPoses.begin();
         (refPoseIter != iRefBoardPoses.end() && srcPoseIter != iSrcBoardPoses.end());
         ++refPoseIter, ++srcPoseIter)
    {
        cv::Vec3d refCenter = refPoseIter->getTranslationVec(); // center of target board observed by reference sensor
        cv::Vec3d srcCenter = srcPoseIter->getTranslationVec(); // center of target board observed in local coordinate system

        //--- transform source pose into reference coordinate system

        // source center of target board observed, transformed into reference coordinate system
        cv::Vec4d transfSrcCenter = iRTSrc2Ref * cv::Vec4d(srcCenter(0),
                                                           srcCenter(1),
                                                           srcCenter(2),
                                                           1.0);

        //--- calculate translational difference / angular error
        oMeanTranslDifference += (cv::Vec3d(transfSrcCenter(0), transfSrcCenter(1), transfSrcCenter(2)) -
                                  refCenter);

        counter++;
    }

    //--- normalize to compute mean
    oMeanTranslDifference /= counter;
}
/************************************************************************************************/
/**
 * @brief Function to calculate the root mean squared error (RMSE) between two aligned point clouds.
 *
 * @tparam SrcPointT Point type of the source input point cloud
 * @tparam RefPointT Point type of the reference input point cloud
 * @param[in] ipSrcCloud Source point cloud aligned to reference cloud ipRefCloud.
 * @param[in] ipRefCloud Reference point cloud.
 * @param[in] ipSrcIndices Pointer to list of point indices in ipSrcCloud which are to be used to
 * compute RMSE. Pass nullptr to use all points.
 * @param[in] ipRefIndices Pointer to list of point indices in ipRefCloud which are to be used to
 * compute RMSE. Pass nullptr to use all points.
 * @return Root mean squared error.
 */
template <typename SrcPointT, typename RefPointT>
inline double calculateRootMeanSquaredError(
  const typename pcl::PointCloud<SrcPointT>::Ptr& ipSrcCloud,
  const typename pcl::PointCloud<RefPointT>::Ptr& ipRefCloud,
  const typename pcl::IndicesPtr& ipSrcIndices = nullptr,
  const typename pcl::IndicesPtr& ipRefIndices = nullptr)
{
    typename pcl::PointCloud<SrcPointT>::Ptr pSrcCloudToUse, pRefCloudToUse;

    //--- check if only a sub cloud is to be used for source
    // NOTE: Could be moved into lambda function with template with c++20
    if (ipSrcIndices != nullptr)
    {
        pcl::ExtractIndices<SrcPointT> extract;
        extract.setInputCloud(ipSrcCloud);
        extract.setIndices(ipSrcIndices);
        extract.setNegative(false);

        pSrcCloudToUse.reset(new pcl::PointCloud<SrcPointT>);
        extract.filter(*pSrcCloudToUse);
    }
    else
    {
        pSrcCloudToUse = ipSrcCloud;
    }

    //--- check if only a sub cloud is to be used for reference
    // NOTE: Could be moved into lambda function with template with c++20
    if (ipRefIndices != nullptr)
    {
        pcl::ExtractIndices<RefPointT> extract;
        extract.setInputCloud(ipRefCloud);
        extract.setIndices(ipRefIndices);
        extract.setNegative(false);

        pRefCloudToUse.reset(new pcl::PointCloud<RefPointT>);
        extract.filter(*pRefCloudToUse);
    }
    else
    {
        pRefCloudToUse = ipRefCloud;
    }

    pcl::IterativeClosestPoint<SrcPointT, RefPointT> icp;
    icp.setInputTarget(pSrcCloudToUse);
    icp.setInputSource(pRefCloudToUse);
    icp.initCompute();

    return std::sqrt(icp.getFitnessScore());
}
/************************************************************************************************/
/**
 * @brief Function to calculate the root mean squared error (RMSE) between two point clouds. In this, the
 * source point cloud is first transformed by iSrcTransform before the RMSE is calculated
 *
 * @tparam SrcPointT Point type of the source input point cloud
 * @tparam RefPointT Point type of the reference input point cloud
 * @param[in] ipSrcCloud Source point cloud.
 * @param[in] ipRefCloud Reference point cloud.
 * @param[in] iSrcTransform Matrix with which to transform the source point cloud prior to
 * calculation.
 * @param[in] ipSrcIndices Pointer to list of point indices in ipSrcCloud which are to be used to
 * compute RMSE. Pass nullptr to use all points.
 * @param[in] ipRefIndices Pointer to list of point indices in ipRefCloud which are to be used to
 * compute RMSE. Pass nullptr to use all points.
 * @return Root mean squared error in the units of the point cloud.
 */
template <typename SrcPointT, typename RefPointT>
inline double calculateRootMeanSquaredError(
  const typename pcl::PointCloud<SrcPointT>::Ptr& ipSrcCloud,
  const typename pcl::PointCloud<RefPointT>::Ptr& ipRefCloud,
  const Eigen::Matrix4f& iSrcTransform,
  const typename pcl::IndicesPtr& ipSrcIndices = nullptr,
  const typename pcl::IndicesPtr& ipRefIndices = nullptr)
{
    //--- transform source cloud
    typename pcl::PointCloud<SrcPointT>::Ptr pAlignedSrcCloud(
      new pcl::PointCloud<SrcPointT>());
    pcl::transformPointCloud(*ipSrcCloud, *pAlignedSrcCloud,
                             iSrcTransform);

    return calculateRootMeanSquaredError<SrcPointT, RefPointT>(pAlignedSrcCloud,
                                                               ipRefCloud,
                                                               ipSrcIndices,
                                                               ipRefIndices);
}
/************************************************************************************************/
/**
 * @brief Function to calculate the root mean squared error (RMSE) between two point clouds. In this, the
 * source point cloud is first transformed by iSrcTransform before the RMSE is calculated
 *
 * @tparam SrcPointT Point type of the source input point cloud
 * @tparam RefPointT Point type of the reference input point cloud
 * @param[in] ipSrcCloud Source point cloud.
 * @param[in] ipRefCloud Reference point cloud.
 * @param[in] iExtrinsicTransform lib3d::Extrinsic with which to transform the source point cloud
 * prior to calculation.
 * @param[in] ipSrcIndices Pointer to list of point indices in ipSrcCloud which are to be used to
 * compute RMSE. Pass nullptr to use all points.
 * @param[in] ipRefIndices Pointer to list of point indices in ipRefCloud which are to be used to
 * compute RMSE. Pass nullptr to use all points.
 * @return Root mean squared error in the units of the point cloud.
 */
template <typename SrcPointT, typename RefPointT>
inline double calculateRootMeanSquaredError(
  const typename pcl::PointCloud<SrcPointT>::Ptr& ipSrcCloud,
  const typename pcl::PointCloud<RefPointT>::Ptr& ipRefCloud,
  const lib3d::Extrinsics& iExtrinsicTransform,
  const typename pcl::IndicesPtr& ipSrcIndices = nullptr,
  const typename pcl::IndicesPtr& ipRefIndices = nullptr)
{
    //--- get transform from extrinsic
    Eigen::Matrix4f transformEigen;
    cv::cv2eigen(cv::Mat(iExtrinsicTransform.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF)), transformEigen);

    return calculateRootMeanSquaredError<SrcPointT, RefPointT>(ipSrcCloud,
                                                               ipRefCloud,
                                                               transformEigen,
                                                               ipSrcIndices,
                                                               ipRefIndices);
}
/************************************************************************************************/
/**
 * @brief Function to compute normal vectors for point cloud of a rotating LiDAR sensor.
 *
 * For each point in the point cloud, this function uses the nearest-neighbor on the same ring,
 * as well as the nearest-neighbor on the neighboring ring to compute a normal vector by means
 * of a cross product. This approach is very fast but does not ensure a spatial consistency of the
 * normal vectors. Furthermore, as the method considers a ring-based neighborhood it requires
 * point types that also encode the ring number.
 *
 * @tparam PointInT Point type of the input point cloud (needs to contain a ring number).
 * @tparam PointOutT Point type of the output point cloud (needs to contain normal information).
 * @param[in] ipCloud Pointer to input cloud.
 * @param[in] nLidarRings Number of rings inside the input point cloud.
 * @param[out] opCloud Pointer to resulting output point cloud.
 */
template <typename PointInT, typename PointOutT>
inline void computeNormalVectorsForPointCloud(const typename pcl::PointCloud<PointInT>::Ptr& ipCloud,
                                              const int& nLidarRings,
                                              typename pcl::PointCloud<PointOutT>::Ptr& opCloud)
{
    //--- get rings

    // Point indices of each ring
    std::vector<pcl::IndicesPtr> pRingIndices(nLidarRings, nullptr);
    for (int pntIdx = 0; pntIdx < ipCloud->size(); ++pntIdx)
    {
        int ringIdx = ipCloud->at(pntIdx).ring;
        if (pRingIndices[ringIdx] == nullptr)
            pRingIndices[ringIdx].reset(new pcl::Indices);

        pRingIndices[ringIdx]->push_back(pntIdx);
    }

    // ring-wise KD-Tree
    std::vector<pcl::search::KdTree<PointInT>> ringKdTrees(nLidarRings);
    for (int ringIdx = 0; ringIdx < nLidarRings; ++ringIdx)
    {
        ringKdTrees[ringIdx].setInputCloud(ipCloud, pRingIndices[ringIdx]);
    }

    //--- loop over points of individual rings
    //--- to calculate normal vector take closest neighbor on same ring as well closest neighbor on
    //--- neighboring ring
    for (int ringIdx = 0; ringIdx < nLidarRings; ++ringIdx)
    {
        for (std::size_t pntIdxItr = 0; pntIdxItr < pRingIndices[ringIdx]->size(); ++pntIdxItr)
        {
            // current point for which the normal vector is to be estimated
            PointInT queryPoint = ipCloud->at(pRingIndices[ringIdx]->at(pntIdxItr));

            // ouptut point to store inside the point cloud
            PointOutT outputPnt;
            outputPnt.x         = queryPoint.x;
            outputPnt.y         = queryPoint.y;
            outputPnt.z         = queryPoint.z;
            outputPnt.intensity = queryPoint.intensity;

            // vectors used to compute normal vector n = v1 x v2
            Eigen::Vector3f v1, v2, n;

            // point index of nearest neighbor on previous and next lidar ring
            pcl::Indices pntIdxPrevNeighborRing(1), pntIdxNextNeighborRing(1);

            // distance from query point to nearest neighbor on previous and next lidar rin
            std::vector<float> prevNeighborPntDistance(1, FLT_MAX),
              nextNeighborPntDistance(1, FLT_MAX);

            // selected nearest neighbor point on neighboring ring
            PointInT pntNeighborRing;

            // nearest neighbor point on same lidar ring
            PointInT neighborPnt;

            //--- find nearest neighbor on previous and next ring, if reachable

            if (ringIdx > 0)
                ringKdTrees[ringIdx - 1]
                  .nearestKSearch(queryPoint, 1,
                                  pntIdxPrevNeighborRing,
                                  prevNeighborPntDistance);
            if (ringIdx < (nLidarRings - 1))
                ringKdTrees[ringIdx + 1].nearestKSearch(queryPoint, 1,
                                                        pntIdxNextNeighborRing,
                                                        nextNeighborPntDistance);

            //--- select point on neighbor ring which is closest to query point

            if (prevNeighborPntDistance[0] <= nextNeighborPntDistance[0])
                pntNeighborRing = ipCloud->at(pntIdxPrevNeighborRing[0]);
            else
                pntNeighborRing = ipCloud->at(pntIdxNextNeighborRing[0]);

            //--- select point on same lidar ring which is closest to query point

            float forwardDistance  = FLT_MAX,
                  backwardDistance = FLT_MAX;

            PointInT forwardNeighbor,
              backwardNeighbor;

            if (pntIdxItr > 0)
            {
                backwardNeighbor = ipCloud->at(pRingIndices[ringIdx]->at(pntIdxItr - 1));
                backwardDistance = Eigen::Vector3f(queryPoint.x - backwardNeighbor.x,
                                                   queryPoint.y - backwardNeighbor.y,
                                                   queryPoint.z - backwardNeighbor.z)
                                     .norm();
            }
            if (pntIdxItr < (pRingIndices[ringIdx]->size() - 1))
            {
                forwardNeighbor = ipCloud->at(pRingIndices[ringIdx]->at(pntIdxItr + 1));
                forwardDistance = Eigen::Vector3f(queryPoint.x - forwardNeighbor.x,
                                                  queryPoint.y - forwardNeighbor.y,
                                                  queryPoint.z - forwardNeighbor.z)
                                    .norm();
            }

            neighborPnt = (forwardDistance <= backwardDistance)
                            ? forwardNeighbor
                            : backwardNeighbor;

            //--- v1 is pointing forward (to the right) and v2 is pointing up, or
            //--- v1 is pointing backward and v2 is pointing down
            if (((forwardDistance <= backwardDistance) && (pntNeighborRing.ring < ringIdx)) ||
                ((backwardDistance < forwardDistance) && (ringIdx < pntNeighborRing.ring)))
            {
                v1 = Eigen::Vector3f(neighborPnt.x - queryPoint.x,
                                     neighborPnt.y - queryPoint.y,
                                     neighborPnt.z - queryPoint.z)
                       .normalized();
                v2 = Eigen::Vector3f(pntNeighborRing.x - queryPoint.x,
                                     pntNeighborRing.y - queryPoint.y,
                                     pntNeighborRing.z - queryPoint.z)
                       .normalized();
            }
            else
            {
                v1 = Eigen::Vector3f(pntNeighborRing.x - queryPoint.x,
                                     pntNeighborRing.y - queryPoint.y,
                                     pntNeighborRing.z - queryPoint.z)
                       .normalized();
                v2 = Eigen::Vector3f(neighborPnt.x - queryPoint.x,
                                     neighborPnt.y - queryPoint.y,
                                     neighborPnt.z - queryPoint.z)
                       .normalized();
            }

            //--- calculate normal vector by using cross product and store into output

            n = v1.cross(v2);

            outputPnt.normal_x = n.x();
            outputPnt.normal_y = n.y();
            outputPnt.normal_z = n.z();

            opCloud->push_back(outputPnt);
        }
    }
}
/************************************************************************************************/
/**
 * @brief Function to compute normal vectors for point cloud.
 *
 * This leverages the function provided by the PCL to compute normal vectors by means of a radius
 * search to find the nearest neighbors. The search radius is given in meters.
 *
 * @tparam PointInT Point type of the input point cloud.
 * @tparam PointOutT Point type of the output point cloud (needs to contain normal information).
 * @param[in] ipCloud Pointer to input cloud.
 * @param[in] iSearchRadiusMeter Search radius in meters to find nearest neighbor.
 * @param[out] opCloud Pointer to resulting output point cloud.
 */
template <typename PointInT, typename PointOutT>
inline void computeNormalVectorsForPointCloudPcl(const typename pcl::PointCloud<PointInT>::Ptr& ipCloud,
                                                 const float& iSearchRadiusMeter,
                                                 typename pcl::PointCloud<PointOutT>::Ptr& opCloud)
{
    //--- copy point cloud to keep at least xyz position
    pcl::copyPointCloud(*ipCloud, *opCloud);

    //--- instantiate normal estimation
    pcl::NormalEstimation<PointInT, PointOutT> ne;
    ne.setInputCloud(ipCloud);

    //--- instantiate kd tree
    typename pcl::search::KdTree<PointInT>::Ptr tree(new pcl::search::KdTree<PointInT>());
    ne.setSearchMethod(tree);

    ne.setRadiusSearch(iSearchRadiusMeter);
    ne.compute(*opCloud);
}
/************************************************************************************************/
/**
 * @brief Function to compute the oriented bounding box for a given input point cloud.
 *
 * This will first transform the point cloud according to its principal components into a local
 * coordinate system, which axes are aligned along principal components. From the transformed point
 * cloud it computes the width, height and length of bounding box. These correspond to the length
 * of the point cloud along its ordered principal components. Furthermore the function computes
 * the center point of the point cloud as well as the transformation of the bounding box so that it
 * fits to the input cloud.
 *
 * @tparam PointT Point type of the point cloud.
 * @param[in] ipCloud Pointer to point cloud that for which the oriented bounding box is to be
 * computed.
 * @param[in] iCalibTarget Object of CalibrationTarget for which a bounding box is to be computed.
 * @param[in] iTrueUp Flag to allow to estimate true up vector based on cutouts on calibration target
 * @param[out] oBboxWidth Width of the oriented bounding box. Since the point cloud is transformed
 * according to its principal components prior to computation of the bounding box, the width will
 * always be along the larges principal component.
 * @param[out] oBboxHeight Height of the oriented bounding box, i.e. length of the point cloud along its
 * second larges principal component.
 * @param[out] oBboxDepth Depth of the oriented bounding box, i.e. length of the point cloud along its
 * second smallest principal component.
 * @param[out] oBboxTransform Transformation of the bounding box to fit the input point cloud. This is
 * equivalent to a transformation from a local to a reference coordinate system. Meaning, that the
 * translational part is equivalent to the center of the bounding box as seen from the reference
 * system and, that the column vectors of the rotation matrix represent the orientation of the
 * axes of the local coordinate system as seen from the reference system. The rotation matrix
 * is set in such a way, that the z-axis is pointing towards the center of the sensor.
 *
 * @see http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
 */
template <typename PointT>
inline void computeOrientedBoundingBox(const typename pcl::PointCloud<PointT>::Ptr ipCloud,
                                       const CalibrationTarget& iCalibTarget,
                                       const bool iTrueUp,
                                       float& oBboxWidth, float& oBboxHeight, float& oBboxDepth,
                                       Eigen::Matrix4f& oBboxTransform)
{
    //--- compute centroid and principal directions

    // centroid of the input cloud
    Eigen::Vector4f cloudCentroid;
    pcl::compute3DCentroid(*ipCloud, cloudCentroid);

    // object of principal component analysis
    pcl::PCA<PointT> pca;
    pca.setInputCloud(ipCloud);

    // Eigenvectors of the input cloud (stored as columns), representing principal directions
    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();

    // Pointer to cloud holing ipCloud transformed by transformation
    typename pcl::PointCloud<PointT>::Ptr pTransformedCloud(new pcl::PointCloud<PointT>);

    // transformation with supposedly min. num of points in cutouts
    Eigen::Matrix4f wtaTransformation(Eigen::Matrix4f::Identity());

    //--- if iTrueUp is set, try to find transformation that best fits to the cutouts of the
    //--- calibration target. If not set, use transformation purely based on eigen vectors
    if (iTrueUp)
    {
        //--- create list of transformations in order to test different orientations of the target
        Eigen::Matrix3f ROT_Y_180, ROT_X_180, ROT_Z_180;
        ROT_Y_180 << -1, 0, 0, 0, 1, 0, 0, 0, -1;
        ROT_X_180 << 1, 0, 0, 0, -1, 0, 0, 0, -1;
        ROT_Z_180 << -1, 0, 0, 0, -1, 0, 0, 0, 1;
        std::vector<Eigen::Matrix3f> rotationCandidates;
        rotationCandidates.push_back(eigenVectors);             // orientation according to eigen vectors of cloud
        rotationCandidates.push_back(eigenVectors * ROT_Y_180); // orientation according to eigen vectors of cloud, horizontally flipped
        rotationCandidates.push_back(eigenVectors * ROT_X_180); // orientation according to eigen vectors of cloud, vertically flipped
        rotationCandidates.push_back(eigenVectors * ROT_Z_180); // orientation according to eigen vectors of cloud, horizontally and vertically flipped

        //--- iterate through the transformation and test number of point lying in areas of cutouts
        int wtaNumPoints = INT_MAX; // minimum number of points within areas of cutouts
        for (const auto& rot : rotationCandidates)
        {
            //--- transform input cloud into local coordinate system where principal directions are aligned
            //--- to the coordinate axes

            Eigen::Matrix4f tmpTransform(Eigen::Matrix4f::Identity());
            tmpTransform.block<3, 3>(0, 0) = rot.transpose();
            tmpTransform.block<3, 1>(0, 3) = -1.f * (tmpTransform.block<3, 3>(0, 0) * cloudCentroid.head<3>());
            pcl::transformPointCloud(*ipCloud, *pTransformedCloud, tmpTransform);

            // KDTree initialized with ipCloud to do a radius search in order to know where the cutouts are
            pcl::KdTreeFLANN<PointT> tree;
            tree.setInputCloud(pTransformedCloud);

#if 0
            // DEBUG VISUALIZATION
            {
                // Eigen::Vector3f center = cloudCentroid.head<3>();
                Eigen::Vector3f center = Eigen::Vector3f(0.f, 0.f, 0.f);

                // Eigen::Vector3f right  = rot.block<3, 1>(0, 3);
                // Eigen::Vector3f up     = rot.block<3, 1>(1, 3);
                // Eigen::Vector3f normal = rot.block<3, 1>(2, 3);
                Eigen::Vector3f right  = Eigen::Vector3f(1.f, 0.f, 0.f);
                Eigen::Vector3f up     = Eigen::Vector3f(0.f, 1.f, 0.f);
                Eigen::Vector3f normal = Eigen::Vector3f(0.f, 0.f, 1.f);

                boost::shared_ptr<pcl::visualization::PCLVisualizer>
                  pViewer(
                    new pcl::visualization::PCLVisualizer("DEBUG Viewer"));
                pViewer->addPointCloud<PointT>(pTransformedCloud, "id");
                pViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                          3,
                                                          "id");
                pViewer->setShowFPS(false);

                pcl::PointXYZ p1(center.x(), center.y(), center.z());
                pcl::PointXYZ p2(Eigen::Vector3f(center + right).x(),
                                 Eigen::Vector3f(center + right).y(),
                                 Eigen::Vector3f(center + right).z());
                pcl::PointXYZ p3(Eigen::Vector3f(center + up).x(),
                                 Eigen::Vector3f(center + up).y(),
                                 Eigen::Vector3f(center + up).z());
                pcl::PointXYZ p4(Eigen::Vector3f(center + normal).x(),
                                 Eigen::Vector3f(center + normal).y(),
                                 Eigen::Vector3f(center + normal).z());
                pViewer->addArrow(p2, p1, 255, 0, 0, false, "right");
                pViewer->addArrow(p3, p1, 0, 255, 0, false, "up");
                pViewer->addArrow(p4, p1, 0, 0, 255, false, "normal");

                pViewer->initCameraParameters();
                pViewer->setCameraPosition(0, 0, 0, 0, -1, 0, 0, 0, 1);
                while (!pViewer->wasStopped())
                {
                    pViewer->spinOnce(50);
                }
                pViewer->close();
                pViewer.reset();
                // DEBUG VISUALIZATION
            }
#endif

            //--- Use method 'searchRadius' from object of KDTree to count number of points that are
            //--- located within the cutouts of the calibration target.
            int numPoints = 0;
            for (const auto& cutout : iCalibTarget.pBoardCutouts)
            {
                //--- define search point
                PointT searchPoint;
                searchPoint.x = cutout->getCenterX();
                searchPoint.y = cutout->getCenterY();
                searchPoint.z = 0.f;

                //--- search for points inside the cutout
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                numPoints += tree.radiusSearch(searchPoint, cutout->getRadius() * 0.75, pointIdxRadiusSearch,
                                               pointRadiusSquaredDistance);
                // numPoints += static_cast<int>(pointIdxRadiusSearch.size());
            }

            //--- if num points are smaller than current WTA solution, update.
            if (numPoints < wtaNumPoints)
            {
                wtaNumPoints      = numPoints;
                wtaTransformation = tmpTransform;
            }
        }
    }
    else
    {
        wtaTransformation.block<3, 3>(0, 0) = eigenVectors.transpose();
        wtaTransformation.block<3, 1>(0, 3) = -1.f * (wtaTransformation.block<3, 3>(0, 0) *
                                                      cloudCentroid.head<3>());
    }

    //--- final transformation of ipCloud by wta transformation
    pcl::transformPointCloud(*ipCloud, *pTransformedCloud, wtaTransformation);

    //--- get center, width, height and depth of the bounding box
    //--- since it it transformed according to the principal components, the width is always the
    //--- larger than the height

    Eigen::Vector4f minPnt, maxPnt;
    pcl::getMinMax3D(*pTransformedCloud, minPnt, maxPnt);
    oBboxWidth  = std::abs(maxPnt.x() - minPnt.x());
    oBboxHeight = std::abs(maxPnt.y() - minPnt.y());
    oBboxDepth  = std::abs(maxPnt.z() - minPnt.z());

    //--- get transformation of bounding box (Direction: LOCAL_2_REF, i.e. as seen from reference system)

    oBboxTransform = wtaTransformation.inverse();
}
/************************************************************************************************/
/**
 * @brief Method to convert geometry_msgs::Pose to tf::Transform.
 *
 * @param[in] iPose geometry_msgs::Pose
 * @param[out] oTransform tf::Transform
 */
inline void cvtGeometryPose2TfTransform(const geometry_msgs::msg::Pose& iPose,
                                        tf2::Transform& oTransform)
{
    oTransform = tf2::Transform(tf2::Quaternion(iPose.orientation.x,
                                                iPose.orientation.y,
                                                iPose.orientation.z,
                                                iPose.orientation.w),
                                tf2::Vector3(iPose.position.x,
                                             iPose.position.y,
                                             iPose.position.z));
}
/************************************************************************************************/
/**
 * @brief Method to convert geometry_msgs::Transform to tf::Transform.
 *
 * @param[in] iTransform geometry_msgs::Transform
 * @param[out] oTransform tf::Transform
 */
inline void cvtGeometryTransform2TfTransform(const geometry_msgs::msg::Transform& iTransform,
                                             tf2::Transform& oTransform)
{
    oTransform = tf2::Transform(tf2::Quaternion(iTransform.rotation.x,
                                                iTransform.rotation.y,
                                                iTransform.rotation.z,
                                                iTransform.rotation.w),
                                tf2::Vector3(iTransform.translation.x,
                                             iTransform.translation.y,
                                             iTransform.translation.z));
}
/************************************************************************************************/
/**
 * @brief Method to convert tf::Transform to geometry_msgs::Pose.
 *
 * @param[in] iTransform tf::Transform
 * @param[out] oPose geometry_msgs::Pose
 */
inline void cvtTfTransform2GeometryPose(const tf2::Transform& iTransform,
                                        geometry_msgs::msg::Pose& oPose)
{
    const tf2::Vector3& origin = iTransform.getOrigin();
    oPose.position.x           = origin.x();
    oPose.position.y           = origin.y();
    oPose.position.z           = origin.z();

    const tf2::Quaternion& rotation = iTransform.getRotation();
    oPose.orientation.w             = rotation.w();
    oPose.orientation.x             = rotation.x();
    oPose.orientation.y             = rotation.y();
    oPose.orientation.z             = rotation.z();
}
/************************************************************************************************/
/**
 * @brief Method to convert tf::Transform to geometry_msgs::Transform.
 *
 * @param[in] iTransform tf::Transform
 * @param[out] oPose geometry_msgs::Transform
 */
inline void cvtTfTransform2GeometryTransform(const tf2::Transform& iTransform,
                                             geometry_msgs::msg::Transform& oPose)
{
    const tf2::Vector3& origin = iTransform.getOrigin();
    oPose.translation.x        = origin.x();
    oPose.translation.y        = origin.y();
    oPose.translation.z        = origin.z();

    const tf2::Quaternion& rotation = iTransform.getRotation();
    oPose.rotation.w                = rotation.w();
    oPose.rotation.x                = rotation.x();
    oPose.rotation.y                = rotation.y();
    oPose.rotation.z                = rotation.z();
}
/************************************************************************************************/
/**
 * @brief Method to draw a random float number in the range of 0,1 (inclusively)
 */
inline float drawRandomFloat()
{
    //--- draw a random float number in the range of 0,1 (inclusively)
    std::srand(time(NULL));
    float rand_float = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
    return rand_float;
}
/************************************************************************************************/
/**
 * @brief Wrapper around ROS spin_until_future_complete for GUI responsiveness
 */
template <typename FutureT>
inline rclcpp::FutureReturnCode doWhileWaiting(rclcpp::Executor::SharedPtr pExecutor, FutureT& future, std::function<void()> doInLoopFunction, int64_t duration_ms = 100)
{
    rclcpp::FutureReturnCode retCode;
    do
    {
        pExecutor->cancel();
        retCode = pExecutor->spin_until_future_complete(future, std::chrono::milliseconds(duration_ms));
        doInLoopFunction();
    } while (retCode == rclcpp::FutureReturnCode::TIMEOUT);

    return retCode;
}

/************************************************************************************************/
/**
 * @brief Gets the cameraInfo topic from an image topic
 * @param[in] iImageTopic image topic
 * @param[in] iCameraNamespace if the camera has another namespace as the image
 *
 * @returns the cameraInfo topic
 */
static inline std::string image2CameraInfoTopic(std::string iImageTopic, std::string iCameraNamespace)
{
    std::string cameraInfoTopicName = "";
    if (iCameraNamespace.empty())
    {
        std::size_t pos     = iImageTopic.find_last_of("/");
        cameraInfoTopicName = iImageTopic.substr(0, pos) + "/camera_info";
    }
    else
    {
        //--- strip cameraNamespace_ from last "/"
        cameraInfoTopicName = iCameraNamespace;
        while (cameraInfoTopicName.back() == '/')
            cameraInfoTopicName.pop_back();
        cameraInfoTopicName += "/camera_info";
    }

    return cameraInfoTopicName;
}

/**
 * @brief Function to set camera extrinsics by using the data from a tf transform.
 *
 * @param[in] iTfTransform Tf transform.
 * @param[out] oCameraExtrinsics Resulting camera extrinsics.
 */
inline void setCameraExtrinsicsFromTfTransform(const tf2::Transform& iTfTransform,
                                               lib3d::Extrinsics& oCameraExtrinsics)
{
    tf2::Vector3 tfOrigin = iTfTransform.getOrigin();
    tf2::Quaternion tfQ   = iTfTransform.getRotation();
    Eigen::Quaternionf eigenQ(tfQ.getW(), tfQ.getX(), tfQ.getY(), tfQ.getZ());
    Eigen::Matrix4f eigenRtMatrix   = Eigen::Matrix4f::Identity();
    eigenRtMatrix.block(0, 3, 3, 1) = Eigen::Vector3f(tfOrigin.x(), tfOrigin.y(), tfOrigin.z());
    eigenRtMatrix.block(0, 0, 3, 3) = eigenQ.toRotationMatrix();
    cv::Mat cvRtMatrix;
    cv::eigen2cv(eigenRtMatrix, cvRtMatrix);
    oCameraExtrinsics.setRTMatrix(cvRtMatrix, lib3d::Extrinsics::REF_2_LOCAL);
}

/**
 * @brief Function to set camera extrinsics by using the data from a tf transform.
 *
 * @param[in] iTfTransform Tf transform.
 * @param[out] oCameraExtrinsics Resulting camera extrinsics.
 */
inline void setCameraExtrinsicsFromTfTransform(const geometry_msgs::msg::TransformStamped& iTfTransform,
                                               lib3d::Extrinsics& oCameraExtrinsics)
{
    tf2::Vector3 tfOrigin;
    tfOrigin.setX(iTfTransform.transform.translation.x);
    tfOrigin.setY(iTfTransform.transform.translation.y);
    tfOrigin.setZ(iTfTransform.transform.translation.z);

    tf2::Quaternion tfQ;
    tfQ.setX(iTfTransform.transform.rotation.x);
    tfQ.setY(iTfTransform.transform.rotation.y);
    tfQ.setZ(iTfTransform.transform.rotation.z);
    tfQ.setW(iTfTransform.transform.rotation.w);

    Eigen::Quaternionf eigenQ(tfQ.getW(), tfQ.getX(), tfQ.getY(), tfQ.getZ());
    Eigen::Matrix4f eigenRtMatrix   = Eigen::Matrix4f::Identity();
    eigenRtMatrix.block(0, 3, 3, 1) = Eigen::Vector3f(tfOrigin.x(), tfOrigin.y(), tfOrigin.z());
    eigenRtMatrix.block(0, 0, 3, 3) = eigenQ.toRotationMatrix();
    cv::Mat cvRtMatrix;
    cv::eigen2cv(eigenRtMatrix, cvRtMatrix);
    oCameraExtrinsics.setRTMatrix(cvRtMatrix, lib3d::Extrinsics::REF_2_LOCAL);
}

/**
 * @brief Function to set camera intrinsics by using the data from sensor_msgs::CameraInfo.
 *
 * @param[in] iCamInfo Camera info message.
 * @param[out] oCameraIntrinsics Resulting camera intrinsics.
 */
inline void setCameraIntrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo& iCamInfo,
                                              lib3d::Intrinsics& oCameraIntrinsics,
                                              const EImageState& iImgState = DISTORTED)
{
    oCameraIntrinsics.setImageSize(iCamInfo.width, iCamInfo.height);

    switch (iImgState)
    {
    default:
    case DISTORTED:
    {
        oCameraIntrinsics.setBy_K(cv::Matx33d(iCamInfo.k[0], iCamInfo.k[1], iCamInfo.k[2],
                                              iCamInfo.k[3], iCamInfo.k[4], iCamInfo.k[5],
                                              iCamInfo.k[6], iCamInfo.k[7], iCamInfo.k[8]));
        oCameraIntrinsics.setDistortionCoeffs(cv::Mat(iCamInfo.d, true));
    }
    break;

    case UNDISTORTED:
    case STEREO_RECTIFIED:
    {
        //--- But since ROS Electric, the camera_calibration package does monocular calibration to
        //--- get K' using OpenCV's getOptimalNewCameraMatrix() function with argument 'alpha'=0.0
        //--- that cause K' ≠ K.
        //--- Projection onto the output image is by the P matrix, which is formed from K' and the
        //--- optional rotation and translation, in this case the identity and 0, respectively.
        //--- see: http://wiki.ros.org/image_pipeline/CameraInfo

        oCameraIntrinsics.setBy_K(cv::Matx33d(iCamInfo.p[0], iCamInfo.p[1], iCamInfo.p[2],
                                              iCamInfo.p[4], iCamInfo.p[5], iCamInfo.p[6],
                                              iCamInfo.p[8], iCamInfo.p[9], iCamInfo.p[10]));
        oCameraIntrinsics.setDistortionCoeffs(cv::Mat());
    }
    break;
    }
}

/**
 * @brief Function to set tf transform from lib3D::Extrinsics.
 *
 * @param[in] iCameraExtrinsics Camera extrinsics.
 * @param[out] oTfTransform Resulting TF Transform.
 */
inline void setTfTransformFromCameraExtrinsics(const lib3d::Extrinsics& iCameraExtrinsics,
                                               tf2::Transform& oTfTransform)
{
    //--- get RT Matrix in Eigen format
    cv::Mat cvRtMatrix = cv::Mat(iCameraExtrinsics.getRTMatrix(lib3d::Extrinsics::REF_2_LOCAL));
    Eigen::Matrix4f eigenRtMatrix;
    cv::cv2eigen(cvRtMatrix, eigenRtMatrix);

    //--- get quaternion from rotation matrix
    Eigen::Quaternionf eigenQ(Eigen::Matrix3f(eigenRtMatrix.block(0, 0, 3, 3)));
    Eigen::Vector3f eigenT = eigenRtMatrix.block(0, 3, 3, 1);

    //--- store in tf
    oTfTransform.setOrigin(tf2::Vector3(eigenT.x(), eigenT.y(), eigenT.z()));
    oTfTransform.setRotation(tf2::Quaternion(eigenQ.x(), eigenQ.y(), eigenQ.z(), eigenQ.w()));
}

inline auto getTopicList(uint32_t nanoSecondsToAccum = 5e8)
{
    auto node        = rclcpp::Node::make_shared("topic_info_node");
    auto topics_info = node->get_topic_names_and_types();

    auto last = node->now().nanoseconds();

    while (node->now().nanoseconds() - last < nanoSecondsToAccum)
    {
        rclcpp::spin_some(node);
        topics_info = node->get_topic_names_and_types();
    }

    return topics_info;
}

} // namespace utils
} // namespace multisensor_calibration

#endif
