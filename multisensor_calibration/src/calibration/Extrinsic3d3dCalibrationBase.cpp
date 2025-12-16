/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration/Extrinsic3d3dCalibrationBase.h"

// Std
#include <thread>
#include <vector>

// PCL
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"
#include "multisensor_calibration/sensor_data_processing/LidarDataProcessor.h"
#include "multisensor_calibration/sensor_data_processing/ReferenceDataProcessor3d.h"

namespace multisensor_calibration
{

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
Extrinsic3d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  Extrinsic3d3dCalibrationBase(ECalibrationType type) :
  ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>(type)
{
}

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
template <typename PointT>
lib3d::Extrinsics Extrinsic3d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::
  computeExtrinsicsFromPointCorrespondences(
    const typename pcl::PointCloud<PointT>::Ptr pSrcCloud,
    const typename pcl::PointCloud<PointT>::Ptr pRefCloud,
    const pcl::Correspondences& pointCorrespondences)

{
    // estimated transformation
    Eigen::Matrix<float, 4, 4> eigenRigidTransf;

    // Pointer to transformation estimation algorithm
    typename pcl::registration::TransformationEstimation<PointT, PointT, float>::Ptr pTransfEst(
      new pcl::registration::TransformationEstimationSVD<PointT, PointT, float>);

    pTransfEst->estimateRigidTransformation(*pSrcCloud, *pRefCloud, pointCorrespondences,
                                            eigenRigidTransf);

#if 0
    {
        typename pcl::PointCloud<PointT>::Ptr pAlignedSrcCloud(
          new pcl::PointCloud<PointT>);

        pcl::transformPointCloud(*pSrcCloud, *pAlignedSrcCloud, eigenRigidTransf);

        // Original point cloud is white
        pcl::visualization::PointCloudColorHandlerCustom<PointT> refColor(
          pRefCloud, (int)255, (int)255, (int)255);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> transfColor(
          pSrcCloud, (int)180, (int)25, (int)25); // red
        pcl::visualization::PointCloudColorHandlerCustom<PointT> alignColor(
          pAlignedSrcCloud, (int)25, (int)180, (int)25); // green

        boost::shared_ptr<pcl::visualization::PCLVisualizer> pViewer(
          new pcl::visualization::PCLVisualizer("DEBUG Viewer"));
        // pcl::visualization::PointCloudColorHandlerGenericField<InputPointType>
        //   pointCloudColorHandler(pRefLidarTargetClouds, "intensity");
        pViewer->addPointCloud<PointT>(pRefCloud, refColor, "refCloud");
        pViewer->addPointCloud<PointT>(pSrcCloud, transfColor, "transformedSrcCloud");
        pViewer->addPointCloud<PointT>(pAlignedSrcCloud, alignColor, "alignedCloud");

        pViewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "refCloud");
        pViewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "alignedCloud");
        pViewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "transformedSrcCloud");

        pViewer->setShowFPS(false);
        while (!pViewer->wasStopped())
        {
            pViewer->spinOnce(50);
        }
        pViewer->close();
        pViewer.reset();
    }
#endif

    //--- convert to lib3d extrinsics

    cv::Mat cvRigidTransf;
    cv::eigen2cv(eigenRigidTransf, cvRigidTransf);

    lib3d::Extrinsics sensorExtrinsics = lib3d::Extrinsics();
    sensorExtrinsics.setRTMatrix(cvRigidTransf, lib3d::Extrinsics::LOCAL_2_REF);

    return sensorExtrinsics;
}
template lib3d::Extrinsics Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  computeExtrinsicsFromPointCorrespondences<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr,
    const pcl::Correspondences&);
template lib3d::Extrinsics Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  computeExtrinsicsFromPointCorrespondences<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr,
    const pcl::Correspondences&);
template lib3d::Extrinsics Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  computeExtrinsicsFromPointCorrespondences<pcl::PointXYZINormal>(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr,
    const pcl::Correspondences&);
template lib3d::Extrinsics Extrinsic3d3dCalibrationBase<LidarDataProcessor,
                                                        ReferenceDataProcessor3d>::
  computeExtrinsicsFromPointCorrespondences<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr,
    const pcl::Correspondences&);
template lib3d::Extrinsics Extrinsic3d3dCalibrationBase<LidarDataProcessor,
                                                        ReferenceDataProcessor3d>::
  computeExtrinsicsFromPointCorrespondences<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr,
    const pcl::Correspondences&);
template lib3d::Extrinsics Extrinsic3d3dCalibrationBase<LidarDataProcessor,
                                                        ReferenceDataProcessor3d>::
  computeExtrinsicsFromPointCorrespondences<pcl::PointXYZINormal>(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr,
    const pcl::Correspondences&);

//==================================================================================================
template <class SrcDataProcessorT, class RefDataProcessorT>
template <typename PointT>
double Extrinsic3d3dCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::runIcp(
  const typename pcl::PointCloud<PointT>::Ptr pSrcCloud,
  const typename pcl::PointCloud<PointT>::Ptr pRefCloud,
  const small_gicp::RegistrationSetting::RegistrationType& icpVariant,
  const double& maxCorrespondenceDistance,
  const double& rotationToleranceDegree,
  const double& translationTolerance,
  const double& downsamplingResolution)
{
    double rmseBefore = INFINITY, rmseAfter = INFINITY;

    //--- construct transformation guess from local_2_ref of cameraExtrinsics
    Eigen::Matrix4f transformationGuess(Eigen::Matrix4f::Identity());
    const lib3d::Extrinsics& prevExtrinsics =
      ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::sensorExtrinsics_.back();
    cv::cv2eigen(cv::Mat(prevExtrinsics.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF)),
                 transformationGuess);

    //--- calculate rmse before
    rmseBefore = utils::calculateRootMeanSquaredError<PointT, PointT>(
      pSrcCloud, pRefCloud, transformationGuess);

    //--- convert PCL point cloud to list of Eigen vectors.
    // TODO: the PCL interface of small_gicp requires pcl >=1.11. Change if other PCL version is
    // available
    std::vector<Eigen::Vector4f> target_points, source_points;
    for (PointT point : *pRefCloud)
        target_points.push_back(Eigen::Vector4f(point.getVector4fMap()));
    for (PointT point : *pSrcCloud)
        source_points.push_back(Eigen::Vector4f(point.getVector4fMap()));

    //--- configure and run GICP

    small_gicp::RegistrationSetting setting;
    setting.type                        = icpVariant;
    setting.num_threads                 = std::thread::hardware_concurrency();
    setting.downsampling_resolution     = downsamplingResolution;
    setting.max_correspondence_distance = maxCorrespondenceDistance;
    setting.rotation_eps                = rotationToleranceDegree * M_PI / 180.0;
    setting.translation_eps             = translationTolerance;

    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.matrix()          = transformationGuess.cast<double>();

    small_gicp::RegistrationResult result = align(target_points, source_points,
                                                  guess, setting);

    RCLCPP_INFO(logger_,
                "GICP convergence: %s",
                (result.converged ? "true" : "false"));
    RCLCPP_INFO(logger_,
                "GICP iterations: %li",
                (result.iterations));

    //--- get final transformations and write into camera extrinsics
    Eigen::Matrix4f icpTransformEigen =
      Eigen::Matrix4f(result.T_target_source.matrix().cast<float>());

    //--- calculate rmse after
    rmseAfter = utils::calculateRootMeanSquaredError<PointT, PointT>(
      pSrcCloud, pRefCloud, icpTransformEigen);

    //--- only add extrinsic estimated by icp, if rmse is higher than before
    if (rmseAfter > rmseBefore)
    {
        RCLCPP_DEBUG(logger_,
                     "RMSE after ICP is larger than before. Rejecting pose estimated by ICP.");
    }
    else
    {
        cv::Mat icpTransformCv;
        cv::eigen2cv(icpTransformEigen, icpTransformCv);

        lib3d::Extrinsics newExtrinsics;
        newExtrinsics.setRTMatrix(icpTransformCv, lib3d::Extrinsics::LOCAL_2_REF);
        ExtrinsicCalibrationBase<SrcDataProcessorT, RefDataProcessorT>::sensorExtrinsics_
          .push_back(newExtrinsics);
    }

    // DEBUG VISUALIZATION
#if 0
    {
        // pointer to cloud holding regions from the source LiDAR transformed into the reference frame
        // prior to running GICP
        typename pcl::PointCloud<PointT>::Ptr pPreAlignedSrcCloud(
          new pcl::PointCloud<PointT>());

        //--- transform camera target clouds based on guess
        pcl::transformPointCloud(*pSrcCloud, *pPreAlignedSrcCloud,
                                 transformationGuess);

        // pointer to cloud after icp alignment
        typename pcl::PointCloud<PointT>::Ptr pAlignedCloud(
          new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*pSrcCloud, *pAlignedCloud,
                                 icpTransformEigen);

        // Original point cloud is white
        pcl::visualization::PointCloudColorHandlerCustom<PointT> refColor(
          pRefCloud, (int)255, (int)255, (int)255);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> transfColor(
          pPreAlignedSrcCloud, (int)180, (int)25, (int)25); // red
        pcl::visualization::PointCloudColorHandlerCustom<PointT> alignColor(
          pAlignedCloud, (int)25, (int)180, (int)25); // green

        boost::shared_ptr<pcl::visualization::PCLVisualizer> pViewer(
          new pcl::visualization::PCLVisualizer("DEBUG Viewer"));
        // pcl::visualization::PointCloudColorHandlerGenericField<InputPointType>
        //   pointCloudColorHandler(pRefLidarTargetClouds, "intensity");
        pViewer->addPointCloud<PointT>(pRefCloud, refColor, "refCloud");
        pViewer->addPointCloud<PointT>(pPreAlignedSrcCloud, transfColor, "transformedSrcCloud");
        pViewer->addPointCloud<PointT>(pAlignedCloud, alignColor, "alignedCloud");

        pViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                                  "refCloud");
        pViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                                  "alignedCloud");
        pViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                                  "transformedSrcCloud");

        pViewer->setShowFPS(false);
        while (!pViewer->wasStopped())
        {
            pViewer->spinOnce(50);
        }
        pViewer->close();
        pViewer.reset();
    }
#endif
    // DEBUG VISUALIZATION

    //--- return root mean squared error (RMSE)
    return std::min(rmseBefore, rmseAfter);
}
template double Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  runIcp<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr,
    const small_gicp::RegistrationSetting::RegistrationType&,
    const double&, const double&, const double&, const double&);
template double Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  runIcp<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr,
    const small_gicp::RegistrationSetting::RegistrationType&,
    const double&, const double&, const double&, const double&);
template double Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>::
  runIcp<pcl::PointXYZINormal>(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr,
    const small_gicp::RegistrationSetting::RegistrationType&,
    const double&, const double&, const double&, const double&);
template double Extrinsic3d3dCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>::
  runIcp<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr,
    const small_gicp::RegistrationSetting::RegistrationType&,
    const double&, const double&, const double&, const double&);
template double Extrinsic3d3dCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>::
  runIcp<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr,
    const small_gicp::RegistrationSetting::RegistrationType&,
    const double&, const double&, const double&, const double&);
template double Extrinsic3d3dCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>::
  runIcp<pcl::PointXYZINormal>(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr,
    const small_gicp::RegistrationSetting::RegistrationType&,
    const double&, const double&, const double&, const double&);

template class Extrinsic3d3dCalibrationBase<LidarDataProcessor, LidarDataProcessor>;
template class Extrinsic3d3dCalibrationBase<LidarDataProcessor, ReferenceDataProcessor3d>;

} // namespace multisensor_calibration
