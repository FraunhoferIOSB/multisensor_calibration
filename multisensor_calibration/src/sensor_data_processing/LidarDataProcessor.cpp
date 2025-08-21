/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/sensor_data_processing/LidarDataProcessor.h"

// Std
#include <iostream>

// ROS
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/transforms.hpp>

// multisensor_calibration
#include "multisensor_calibration/common/utils.hpp"

namespace multisensor_calibration
{

//==================================================================================================
LidarDataProcessor::LidarDataProcessor(const std::string& iLoggerName,
                                       const std::string& iSensorName,
                                       const fs::path& iCalibTargetFilePath) :
  DataProcessor3d(iLoggerName, iSensorName, iCalibTargetFilePath),
  pCalibTargetSacModel_(nullptr),
  pPreprocFilter_(nullptr)
{

    //--- set verbosity level for pcl
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    //--- initialize sample consensus model of target pattern
    pCalibTargetSacModel_.reset(
      new CalibrationTargetSacModel<InputPointType>(
        calibrationTarget_,
        std::make_shared<pcl::PointCloud<InputPointType>>()));
    isInitialized_ &= (pCalibTargetSacModel_ != nullptr);
}

//==================================================================================================
LidarDataProcessor::~LidarDataProcessor()
{
}

//==================================================================================================
LidarDataProcessor::EProcessingResult LidarDataProcessor::processData(
  const pcl::PointCloud<InputPointType>& iPointCloud,
  const EProcessingLevel& iProcLevel)
{
    pcl::PointCloud<InputPointType>::Ptr pPointCloud = iPointCloud.makeShared();

    // pointer to temporary point cloud container
    pcl::PointCloud<InputPointType>::Ptr pRawInputCloud(new pcl::PointCloud<InputPointType>);

    // pointer to temporary point cloud container
    pcl::PointCloud<InputPointType>::Ptr pTmpCloud(new pcl::PointCloud<InputPointType>);

    // pointer KD Search tree
    pcl::search::Search<InputPointType>::Ptr pSearchKdTree(new pcl::search::KdTree<InputPointType>);

    // pointer to normal vectors corresponding to input cloud
    pcl::PointCloud<pcl::Normal>::Ptr pPointNormals(new pcl::PointCloud<pcl::Normal>);

    // point indices of individual region clusters as result of region growing
    std::vector<pcl::PointIndices> regionClustersIndices;

    // point cloud of a segmented cluster
    pcl::PointCloud<InputPointType>::Ptr pClusterCloud(new pcl::PointCloud<InputPointType>);

    // planar projection of the cluster cloud
    pcl::PointCloud<InputPointType>::Ptr pPlanarClusterCloud(new pcl::PointCloud<InputPointType>);

    // reset roi cloud
    pRoisCloud_.reset(new pcl::PointCloud<InputPointType>);

    //--- apply range filter if applicable
    if (targetDetectionParams_.max_range.value > 0.001)
    {
        const double THRESHOLD_2 = targetDetectionParams_.max_range.value *
                                   targetDetectionParams_.max_range.value;

        for (pcl::PointCloud<InputPointType>::iterator itr = pPointCloud->begin();
             itr != pPointCloud->end(); ++itr)
        {
            double x      = static_cast<double>(itr->x);
            double y      = static_cast<double>(itr->y);
            double z      = static_cast<double>(itr->z);
            double range2 = x * x + y * y + z * z;

            if (range2 <= THRESHOLD_2)
                pTmpCloud->push_back(*itr);
        }

        //--- swap pointer between input and temporary cloud
        pTmpCloud.swap(pPointCloud);
        pTmpCloud->clear();
    }

    //--- transform point cloud if applicable
    if (pDataTransform_)
    {
        pcl_ros::transformPointCloud(*pPointCloud, *pTmpCloud, *pDataTransform_);

        //--- swap pointer between input and temporary cloud
        pTmpCloud.swap(pPointCloud);
        pTmpCloud->clear();
    }

    //--- copy transformed raw point cloud into container
    pcl::copyPointCloud(*pPointCloud, *pRawInputCloud);

    //--- If filter is set, apply
    if (pPreprocFilter_)
    {
        //--- apply filter
        pPreprocFilter_->setInputCloud(pPointCloud);
        pPreprocFilter_->filter(*pTmpCloud);

        //--- swap pointer between input and temporary cloud
        pTmpCloud.swap(pPointCloud);
    }

#if 0 // save point cloud into file
    if (iProcLevel == TARGET_DETECTION)
    {
        fs::path filePath = fs::current_path();
        filePath /= "point_cloud_" +
                    ipCloudMsg->header.frame_id +
                    "_" +
                    std::to_string(ipCloudMsg->header.stamp.sec) +
                    "-" +
                    std::to_string(ipCloudMsg->header.stamp.nsec) +
                    ".ply";
        pcl::PLYWriter writer;
        writer.write<InputPointType>(filePath.string(), *pPointCloud, true, false);
        RCLCPP_INFO(logger_, "Point cloud saved to file: %s", filePath.string().c_str());
    }
#endif

    //--- do normal estimation
    estimateCloudNormals(pPointCloud, pSearchKdTree, pPointNormals);

    //--- do region growing
    doRegionGrowing(pPointCloud, pPointNormals, pSearchKdTree, regionClustersIndices);

    //--- loop through the region clusters, extract points from point cloud, compute bounding box
    //--- and filter based on its area

    pcl::ExtractIndices<InputPointType> extractIndices;
    extractIndices.setInputCloud(pPointCloud);

    for (std::vector<pcl::PointIndices>::iterator clusterIndicesItr = regionClustersIndices.begin();
         clusterIndicesItr != regionClustersIndices.end(); ++clusterIndicesItr)
    {
        //--- get cloud from indices
        pcl::PointIndices::Ptr pClusterIndices(new pcl::PointIndices());
        pClusterIndices->header  = clusterIndicesItr->header;
        pClusterIndices->indices = clusterIndicesItr->indices;
        extractIndices.setIndices(pClusterIndices);

        pClusterCloud->clear();
        extractIndices.filter(*pClusterCloud);

        // DEBUG VISUALIZATION
#if 0
        {
            boost::shared_ptr<pcl::visualization::PCLVisualizer> pViewer(
              new pcl::visualization::PCLVisualizer("DEBUG Viewer"));
            pcl::visualization::PointCloudColorHandlerGenericField<PointType>
              pointCloudColorHandler(pClusterCloud, "intensity");
            pViewer->addPointCloud<PointType>(pClusterCloud, pointCloudColorHandler, "id");
            pViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                      3,
                                                      "id");
            pViewer->setShowFPS(false);

            // Eigen::Quaternionf bbquat(Eigen::Matrix3f(bboxTransform.block<3, 3>(0, 0)));
            // Eigen::Vector3f bbtrasl(bboxTransform.block<3, 1>(0, 3));
            // pViewer->addCube(bbtrasl, bbquat, bboxWidth, bboxHeight, bboxDepth, "cube", 0);

            // const Eigen::Vector3f BBOX_CENTER     = bboxTransform.block<3, 1>(0, 3);
            // const Eigen::Vector3f BBOX_UP_VEC     = bboxTransform.block<3, 1>(0, 1);
            // const Eigen::Vector3f BBOX_NORMAL_VEC = bboxTransform.block<3, 1>(0, 2);
            // const Eigen::Vector3f BBOX_RIGHT_VEC =
            //   BBOX_UP_VEC.normalized().cross(BBOX_NORMAL_VEC.normalized());

            // pcl::PointXYZ p1(BBOX_CENTER.x(), BBOX_CENTER.y(), BBOX_CENTER.z());
            // pcl::PointXYZ p2(Eigen::Vector3f(BBOX_CENTER + BBOX_RIGHT_VEC).x(),
            //                  Eigen::Vector3f(BBOX_CENTER + BBOX_RIGHT_VEC).y(),
            //                  Eigen::Vector3f(BBOX_CENTER + BBOX_RIGHT_VEC).z());
            // pcl::PointXYZ p3(Eigen::Vector3f(BBOX_CENTER + BBOX_UP_VEC).x(),
            //                  Eigen::Vector3f(BBOX_CENTER + BBOX_UP_VEC).y(),
            //                  Eigen::Vector3f(BBOX_CENTER + BBOX_UP_VEC).z());
            // pViewer->addArrow(p2, p1, 255, 0, 0, false, "right");
            // pViewer->addArrow(p3, p1, 0, 255, 0, false, "up");

            pViewer->initCameraParameters();
            pViewer->setCameraPosition(0, 0, 0, 0, -1, 0, 0, 0, 1);
            while (!pViewer->wasStopped())
            {
                pViewer->spinOnce(50);
            }
            pViewer->close();
            pViewer.reset();
        }
#endif
        // DEBUG VISUALIZATION

        //-- do size filter test
        // Transformation of the bounding box to fit the cluster cloud.
        Eigen::Matrix4f bboxTransform;
        bool isRegionOfInterest = testClusterSize(pClusterCloud, bboxTransform);
        if (isRegionOfInterest)
        {
            //--- added tested cluster cloud to roi cloud
            pRoisCloud_->insert(pRoisCloud_->end(), pClusterCloud->begin(), pClusterCloud->end());

            //--- if processing level is preview, continue with looping over individual regions
            if (iProcLevel == PREVIEW)
                continue;

            //--- project cluster points to planar model

            // plane parameters of the planar model used for projecting the cluster cloud
            Eigen::VectorXf planeModelCoefficients;
            bool isSuccessful = projectClusterToPlanarModel(pClusterCloud,
                                                            pPlanarClusterCloud,
                                                            planeModelCoefficients);
            if (!isSuccessful)
                continue;

            //--- try to detect calibration target

            // Model coefficients of detected calibration target
            Eigen::VectorXf calibTargetModelCoefficients;

            // score for model coefficients from ransac and icp
            std::size_t ransacInliers, icpInliers;

            isSuccessful = detectCalibrationTargetInCluster(pPlanarClusterCloud,
                                                            calibTargetModelCoefficients,
                                                            ransacInliers,
                                                            icpInliers);
            if (!isSuccessful)
                continue;

            //--- convert model coefficients to extrinsic pose to store observation
            lib3d::Extrinsics targetPose = lib3d::Extrinsics();
            calibrationTarget_.computePose(
              calibTargetModelCoefficients.block<3, 1>(7, 0),
              calibTargetModelCoefficients.block<3, 1>(4, 0),
              calibTargetModelCoefficients.block<3, 1>(0, 0),
              targetPose);

            RCLCPP_INFO(logger_,
                        "Found observation. RANSAC Inliers: %li, ICP Inliers: %li. "
                        "Using pose found by %s.",
                        ransacInliers, icpInliers,
                        (ransacInliers >= icpInliers) ? "RANSAC" : "ICP");

            //--- get inliers given the model coefficients
            // point indices belonging to detected calibration target
            pcl::PointIndices::Ptr pCalibTargetInlierIndices(new pcl::PointIndices);
            pCalibTargetSacModel_->selectWithinDistance(
              calibTargetModelCoefficients,
              targetDetectionParams_.ransac_distance_thresh.value,
              pCalibTargetInlierIndices->indices);

            //--- extract calibration target cloud from inliers
            calibrationTargetCloudPtrs_.push_back(nullptr);
            calibrationTargetCloudPtrs_.back().reset(new pcl::PointCloud<InputPointType>());
            extractIndices.setInputCloud(pPlanarClusterCloud);
            extractIndices.setIndices(pCalibTargetInlierIndices);
            extractIndices.filter(*(calibrationTargetCloudPtrs_.back()));

            //--- if calibration target cloud is empty, return
            if (calibrationTargetCloudPtrs_.back()->empty())
            {
                RCLCPP_ERROR(logger_, "Calibration target cloud is empty.");
                calibrationTargetCloudPtrs_.pop_back();
                return FAILED;
            }

            //--- store input point cloud and set intensity to 0
            inputCloudPtrs_.push_back(pRawInputCloud);
            std::transform(inputCloudPtrs_.back()->points.begin(),
                           inputCloudPtrs_.back()->points.end(),
                           inputCloudPtrs_.back()->points.begin(),
                           [&](InputPointType p)
                           { p.intensity = 0.f; return p; });

            //--- set intensity value of calibration target cloud to 100
            std::transform(calibrationTargetCloudPtrs_.back()->points.begin(),
                           calibrationTargetCloudPtrs_.back()->points.end(),
                           calibrationTargetCloudPtrs_.back()->points.begin(),
                           [&](InputPointType p)
                           { p.intensity = 100.f; return p; });

            //--- Compute points of markers, store the id of the marker to which it belongs in
            //--- intensity value

            // Output point cloud of corner points of the ArUco markers
            estimatedMarkerCornersCloudPtrs_.push_back(nullptr);
            estimatedMarkerCornersCloudPtrs_.back().reset(
              new pcl::PointCloud<pcl::PointXYZI>());

            //--- compute marker corners from pose
            std::vector<uint> markerIds;
            std::vector<std::array<cv::Point3f, 4>> markerCorners;
            calibrationTarget_.computeMarkerCornersFromPose(
              calibTargetModelCoefficients.block<3, 1>(7, 0),
              calibTargetModelCoefficients.block<3, 1>(4, 0),
              calibTargetModelCoefficients.block<3, 1>(0, 0),
              markerIds,
              markerCorners);

            //--- copy marker corners to output cloud
            auto pMarkerCloud = estimatedMarkerCornersCloudPtrs_.back();
            for (uint m = 0; m < markerIds.size(); m++)
            {
                for (cv::Point3f corner : markerCorners[m])
                {
                    //--- construct point for point cloud

                    pcl::PointXYZI pclPnt;
                    pclPnt.x         = static_cast<float>(corner.x);
                    pclPnt.y         = static_cast<float>(corner.y);
                    pclPnt.z         = static_cast<float>(corner.z);
                    pclPnt.intensity = markerIds[m];

                    pMarkerCloud->push_back(pclPnt);
                }
            }

            //--- store calibration target pose
            estimatedMarkerIds_.push_back(markerIds);
            estimatedMarkerCorners_.push_back(markerCorners);
            capturedCalibTargetPoses_.push_back(targetPose);
#if 0
                // DEBUG VISUALIZATION
                {
                    Eigen::Vector3f center = calibTargetModelCoefficients.block<3, 1>(7, 0);
                    Eigen::Vector3f up     = calibTargetModelCoefficients.block<3, 1>(4, 0);
                    Eigen::Vector3f normal = calibTargetModelCoefficients.block<3, 1>(0, 0);
                    Eigen::Vector3f right  = up.cross(normal);

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pVisPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                    int counter = 0;
                    for (pcl::PointCloud<PointType>::iterator itr = pPlanarClusterCloud->begin();
                         itr != pPlanarClusterCloud->end(); ++itr)
                    {
                        pcl::PointXYZRGB pnt(255, 255, 255);
                        pnt.x = itr->x;
                        pnt.y = itr->y;
                        pnt.z = itr->z;

                        if (std::find(pCalibTargetInlierIndices->indices.begin(),
                                      pCalibTargetInlierIndices->indices.end(), counter) != pCalibTargetInlierIndices->indices.end())
                        {
                            pnt.g = 0;
                            pnt.b = 0;
                        }

                        pVisPointCloud->push_back(pnt);

                        counter++;
                    }

                    pcl::PolygonMeshPtr pCadModel(new pcl::PolygonMesh);
                    pcl::io::loadPLYFile(calibrationTarget_.cadModelMeshPath, *pCadModel);

                    Eigen::Matrix4f eigenTransformMat(Eigen::Matrix4f::Identity());
                    eigenTransformMat.block<3, 1>(0, 0) = right;
                    eigenTransformMat.block<3, 1>(0, 1) = up;
                    eigenTransformMat.block<3, 1>(0, 2) = normal;
                    eigenTransformMat.block<3, 1>(0, 3) = center;

                    pcl::PointCloud<pcl::PointXYZ> tmpMeshCloud;
                    pcl::fromPCLPointCloud2(pCadModel->cloud, tmpMeshCloud);
                    pcl::transformPointCloud(tmpMeshCloud, tmpMeshCloud, eigenTransformMat);
                    pcl::toPCLPointCloud2(tmpMeshCloud, pCadModel->cloud);

                    boost::shared_ptr<pcl::visualization::PCLVisualizer>
                      pViewer(
                        new pcl::visualization::PCLVisualizer("DEBUG Viewer"));
                    pViewer->addPointCloud<pcl::PointXYZRGB>(pVisPointCloud, "id");
                    pViewer->addPolygonMesh(*pCadModel);
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
                    pViewer->addArrow(p2, p1, 255, 0, 0, false, "right");
                    pViewer->addArrow(p3, p1, 0, 255, 0, false, "up");

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

            return SUCCESS;
        }
    }

    //--- if this point is reached and the processing level is preview,
    //--- then the ROI cloud has been filled, thus return success.
    //--- Otherwise, calibration target could not be found, thus return failed
    if (iProcLevel == PREVIEW)
    {
        return SUCCESS;
    }
    else
    {
        RCLCPP_WARN(logger_, "Calibration target SAC model could not be detected! "
                             "Please try to capture target again.");
        return FAILED;
    }
}

//==================================================================================================
void LidarDataProcessor::setPreprocFilter(const pcl::Filter<InputPointType>::Ptr& pFilter)
{
    pPreprocFilter_ = pFilter;
}

//==================================================================================================
void LidarDataProcessor::setParameters(const LidarTargetDetectionParameters& iParams)
{
    targetDetectionParams_ = iParams;
}

//==================================================================================================
bool LidarDataProcessor::detectCalibrationTargetInCluster(
  const pcl::PointCloud<InputPointType>::Ptr& ipClusterCloud,
  Eigen::VectorXf& oModelCoefficients,
  std::size_t& oRansacInlierCnt,
  std::size_t& oIcpInlierCnt) const
{

    //--- get oriented bounding box
    float bboxWidth, bboxHeight, bboxDepth;
    Eigen::Matrix4f bboxTransform;
    utils::computeOrientedBoundingBox<InputPointType>(ipClusterCloud, calibrationTarget_, true,
                                                      bboxWidth, bboxHeight,
                                                      bboxDepth, bboxTransform);

    //--- get pose of bounding box from bboxtransform
    const Eigen::Vector3f BBOX_CENTER     = bboxTransform.block<3, 1>(0, 3);
    const Eigen::Vector3f BBOX_UP_VEC     = bboxTransform.block<3, 1>(0, 1);
    const Eigen::Vector3f BBOX_NORMAL_VEC = bboxTransform.block<3, 1>(0, 2);

    // point to sample consensus model of calibration target
    pCalibTargetSacModel_->setInputCloud(ipClusterCloud);
    pCalibTargetSacModel_->setAxis(BBOX_NORMAL_VEC);
    pCalibTargetSacModel_->setEpsAngle(5); // due to planar projection this will not have an effect
    pCalibTargetSacModel_->setPoseGuess(BBOX_CENTER, BBOX_UP_VEC, BBOX_NORMAL_VEC);
    pCalibTargetSacModel_->setRotationVariance(
      targetDetectionParams_.ransac_rotation_variance.value);
    pCalibTargetSacModel_->setTranslationVariance(
      targetDetectionParams_.ransac_translation_variance.value);
    pCalibTargetSacModel_->setIncrementalPoseGuessUpdate(true);

    // RANSAC to detect calibration target
    pcl::RandomSampleConsensus<InputPointType> targetRansac(pCalibTargetSacModel_);
    targetRansac.setDistanceThreshold(targetDetectionParams_.ransac_distance_thresh.value);

    bool isSuccessful = targetRansac.computeModel();
    if (!isSuccessful)
        return false;

    // model coefficients of calibration target model
    Eigen::VectorXf ransacModelCoefficients;
    targetRansac.getModelCoefficients(ransacModelCoefficients);

    // score for model coefficients from ransac
    oRansacInlierCnt =
      pCalibTargetSacModel_->countWithinDistance(
        ransacModelCoefficients,
        targetDetectionParams_.ransac_distance_thresh.value);

    // optimized model coefficients of calibration target model
    Eigen::VectorXf icpModelCoefficients;
    if (targetDetectionParams_.ransac_optimize_coefficients.value)
    {
        pCalibTargetSacModel_->setIcpVariant(
          static_cast<small_gicp::RegistrationSetting::RegistrationType>(
            targetDetectionParams_.target_icp_variant.value));
        pCalibTargetSacModel_->setIcpCorrespondenceDistance(
          targetDetectionParams_.target_icp_max_correspondence_distance.value);
        pCalibTargetSacModel_->setIcpRotationTolerance(
          targetDetectionParams_.target_icp_rotation_tolerance.value);
        pCalibTargetSacModel_->setIcpTranslationTolerance(
          targetDetectionParams_.target_icp_translation_tolerance.value);

        pCalibTargetSacModel_->optimizeModelCoefficients(ransacModelCoefficients,
                                                         icpModelCoefficients);

        // score for model coefficients from icp
        oIcpInlierCnt =
          pCalibTargetSacModel_->countWithinDistance(
            icpModelCoefficients,
            targetDetectionParams_.ransac_distance_thresh.value);
    }
    else
    {
        oIcpInlierCnt = 0;
    }

    // final model coefficients depending on score
    oModelCoefficients = (oRansacInlierCnt >= oIcpInlierCnt)
                           ? ransacModelCoefficients
                           : icpModelCoefficients;

    return true;
}

//==================================================================================================
void LidarDataProcessor::doRegionGrowing(
  const pcl::PointCloud<InputPointType>::Ptr& ipInputCloud,
  const pcl::PointCloud<pcl::Normal>::Ptr& ipNormalCloud,
  pcl::search::Search<InputPointType>::Ptr& iopSearchTree,
  std::vector<pcl::PointIndices>& oClusterIndices) const
{
    //--- find point indices whose coordinates are NAN
    pcl::IndicesPtr nanIndices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*ipInputCloud, *nanIndices);

    //--- do region growing and extract region cluster

    pcl::RegionGrowing<InputPointType, pcl::Normal> regionGrowing;
    regionGrowing.setMinClusterSize(
      targetDetectionParams_.region_growing_cluster_size_min.value);
    regionGrowing.setMaxClusterSize(
      targetDetectionParams_.region_growing_cluster_size_max.value);
    regionGrowing.setSearchMethod(iopSearchTree);
    regionGrowing.setNumberOfNeighbours(
      targetDetectionParams_.region_growing_number_neighbors.value);
    regionGrowing.setInputCloud(ipInputCloud);
    regionGrowing.setIndices(nanIndices);
    regionGrowing.setInputNormals(ipNormalCloud);
    regionGrowing.setSmoothnessThreshold(
      static_cast<float>(targetDetectionParams_.region_growing_angle_thresh.value / 180.0 * M_PI));
    regionGrowing.setCurvatureThreshold(
      static_cast<float>(targetDetectionParams_.region_growing_curvature_thresh.value));

    // point indices grouped into clusters resulting from the region growing
    regionGrowing.extract(oClusterIndices);
}

//==================================================================================================
void LidarDataProcessor::estimateCloudNormals(
  const pcl::PointCloud<InputPointType>::Ptr& ipInputCloud,
  pcl::search::Search<InputPointType>::Ptr& iopSearchTree,
  pcl::PointCloud<pcl::Normal>::Ptr& opNormalCloud) const
{
    //--- do normal estimation
    pcl::NormalEstimationOMP<InputPointType, pcl::Normal> normalEstimator;
    normalEstimator.setNumberOfThreads(); // Set number of threads to automatic
    normalEstimator.setSearchMethod(iopSearchTree);
    normalEstimator.setInputCloud(ipInputCloud);
    bool useRadiusSearch = (targetDetectionParams_.normal_estimation_search_method.value ==
                            LidarTargetDetectionParameters::RADIUS_SEARCH);
    if (useRadiusSearch)
        normalEstimator.setRadiusSearch(
          targetDetectionParams_.normal_estimation_search_radius.value);
    else
        normalEstimator.setKSearch(
          static_cast<int>(targetDetectionParams_.normal_estimation_search_radius.value));

    normalEstimator.compute(*opNormalCloud);
}

//==================================================================================================
bool LidarDataProcessor::projectClusterToPlanarModel(
  const pcl::PointCloud<InputPointType>::Ptr& ipClusterCloud,
  pcl::PointCloud<InputPointType>::Ptr& opPlanarClusterCloud,
  Eigen::VectorXf& oPlaneParameters) const
{
    //--- detect plane with ransac and project points onto plane
    pcl::SampleConsensusModelPlane<InputPointType>::Ptr pPlaneSacModel(
      new pcl::SampleConsensusModelPlane<InputPointType>(ipClusterCloud));
    pcl::RandomSampleConsensus<InputPointType> planeRansac(pPlaneSacModel);
    planeRansac.setDistanceThreshold(targetDetectionParams_.ransac_distance_thresh.value);

    bool isSuccessful = planeRansac.computeModel();
    if (!isSuccessful)
        return false;

    //--- refine model
    planeRansac.refineModel();
    planeRansac.getModelCoefficients(oPlaneParameters);

    // point indices found as inliers by ransac detection
    pcl::Indices inlierIndices;
    planeRansac.getInliers(inlierIndices);

    //--- project point cloud
    opPlanarClusterCloud->clear();
    pPlaneSacModel->projectPoints(inlierIndices, oPlaneParameters,
                                  *opPlanarClusterCloud, false);

    return true;
}

//==================================================================================================
bool LidarDataProcessor::testClusterSize(
  const pcl::PointCloud<InputPointType>::Ptr& ipCluster,
  Eigen::Matrix4f& oBboxTransform) const
{
    bool retVal = false;

    if (ipCluster->size() < 3)
        return false;

    //--- get oriented bounding box
    float bboxWidth, bboxHeight, bboxDepth;
    utils::computeOrientedBoundingBox<InputPointType>(ipCluster, calibrationTarget_, false,
                                                      bboxWidth, bboxHeight,
                                                      bboxDepth, oBboxTransform);

    //--- filter based on dimension
    const float WIDTH_TOLERANCE_MIN =
      static_cast<float>(targetDetectionParams_.size_filter_width_min_tolerance.value);
    const float WIDTH_TOLERANCE_MAX =
      static_cast<float>(targetDetectionParams_.size_filter_width_max_tolerance.value);
    const float HEIGHT_TOLERANCE_MIN =
      static_cast<float>(targetDetectionParams_.size_filter_height_min_tolerance.value);
    const float HEIGHT_TOLERANCE_MAX =
      static_cast<float>(targetDetectionParams_.size_filter_width_max_tolerance.value);

    const cv::Size2f BOARD_SIZE = calibrationTarget_.boardSize;
    const float MIN_AREA        = (BOARD_SIZE.width - WIDTH_TOLERANCE_MIN) *
                           (BOARD_SIZE.height - HEIGHT_TOLERANCE_MIN);
    const float MAX_AREA = (BOARD_SIZE.width + WIDTH_TOLERANCE_MAX) *
                           (BOARD_SIZE.height + HEIGHT_TOLERANCE_MAX);
    const float MAX_ASPECT_RATIO =
      std::max(BOARD_SIZE.width + WIDTH_TOLERANCE_MAX, BOARD_SIZE.height + HEIGHT_TOLERANCE_MAX) /
      std::min(BOARD_SIZE.width - WIDTH_TOLERANCE_MIN, BOARD_SIZE.height - HEIGHT_TOLERANCE_MIN);
    float bboxArea        = bboxWidth * bboxHeight;
    float bboxAspectRatio = std::max(bboxWidth, bboxHeight) /
                            std::min(bboxWidth, bboxHeight);
    retVal = ((MIN_AREA <= bboxArea && bboxArea <= MAX_AREA) &&
              bboxAspectRatio <= MAX_ASPECT_RATIO);

    return retVal;
}

//==================================================================================================
bool LidarDataProcessor::extractPlaneFromPointCloud(
  const pcl::PointCloud<InputPointType>::Ptr& ipInputCloud,
  const Eigen::Vector3f& iNormalVec,
  const double& iAngleTolerance,
  pcl::PointCloud<InputPointType>::Ptr& opOutputCloud)
{
    //--- check if output pointer is a nullptr
    if (opOutputCloud == nullptr)
        opOutputCloud.reset(new pcl::PointCloud<InputPointType>);

    pcl::SampleConsensusModelPerpendicularPlane<InputPointType>::Ptr pPlaneSacModel(
      new pcl::SampleConsensusModelPerpendicularPlane<InputPointType>(ipInputCloud));
    pPlaneSacModel->setAxis(iNormalVec);
    pPlaneSacModel->setEpsAngle(pcl::deg2rad(iAngleTolerance));

    pcl::RandomSampleConsensus<InputPointType> planeRansac(pPlaneSacModel);
    planeRansac.setDistanceThreshold(0.1);

    bool isSuccessful = planeRansac.computeModel();
    if (!isSuccessful)
        return false;

    //--- refine model
    planeRansac.refineModel();

    // point indices found as inliers by ransac detection
    pcl::IndicesPtr inlierIndices(new pcl::Indices);
    planeRansac.getInliers(*inlierIndices);

    pcl::ExtractIndices<InputPointType> extract;
    extract.setInputCloud(ipInputCloud);
    extract.setIndices(inlierIndices);
    extract.setNegative(false);
    extract.filter(*opOutputCloud);

    return true;
}

//==================================================================================================
bool LidarDataProcessor::extractPlaneFromPointCloud(
  const pcl::PointCloud<InputPointType>::Ptr& ipInputCloud,
  const tf2::Vector3& iNormalVec,
  const double& iAngleTolerance,
  pcl::PointCloud<InputPointType>::Ptr& opOutputCloud)
{
    return LidarDataProcessor::extractPlaneFromPointCloud(
      ipInputCloud,
      Eigen::Vector3f(
        static_cast<float>(iNormalVec.x()),
        static_cast<float>(iNormalVec.y()),
        static_cast<float>(iNormalVec.z())),
      iAngleTolerance, opOutputCloud);
}

} // namespace multisensor_calibration