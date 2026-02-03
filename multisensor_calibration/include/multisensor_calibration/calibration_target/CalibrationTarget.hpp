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

#ifndef MULTISENSORCALIBRATION_CALIBRATIONTARGET_HPP
#define MULTISENSORCALIBRATION_CALIBRATIONTARGET_HPP

// Std
#include <memory>
#include <vector>

// Eigen
#include <Eigen/Core>

// Boost
#include <boost/filesystem.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// OpenCV
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

// multisensor_calibration
#include "CircularCutout.h"
#include <multisensor_calibration/common/lib3D/core/extrinsics.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup  target_detection
 * @brief Struct resembling the used calibration target, and its geometry.
 *
 */
struct CalibrationTarget
{

    /// Size of calibration board in meters
    cv::Size2f boardSize = cv::Size2f();

    /// Side length of aruco markers in meters
    float markerSize = 0.0f;

    /// IDs of marker used (order: clockwise, starting from top left of board)
    std::vector<int> markerIds = {};

    /// x,y,z marker positions (top-left) on board relative to top-left corner (planar --> z = 0)
    std::vector<cv::Point3f> markerPositions = {};

    /// x,y,z markers' corners positions on board relative to top-left corner (planar --> z = 0)
    std::vector<std::vector<cv::Point3f>> boardCorners = {};

    /// cutouts on board
    std::vector<std::shared_ptr<Cutout>> pBoardCutouts = {};

    /// minimum number of markers that need to be detected.
    int minMarkerDetection = 3;

    /// file path to CAD model of the calibration target as mesh
    std::string cadModelMeshPath = "";

    /// file path to CAD model of the calibration target as cloud
    std::string cadModelCloudPath = "";

    /// Pointer to ArUco board
    cv::Ptr<cv::aruco::Board> pArucoBoard = nullptr;

    /// Pointer to ArUco dictoionary used (Default DICT_6X6_250)
    cv::Ptr<cv::aruco::Dictionary> pArucoDictionary = nullptr;

    /**
     * @brief Method to create ArUco Board based on the marker positions and their ids
     *
     */
    void createArUcoBoard()
    {
        //--- create ArUco dictionary
        pArucoDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        //--- create arUco board

        // number of markers on the calibration board
        uint nMarkers = markerIds.size();

        // corner points of each marker on the calibration board
        boardCorners = std::vector<std::vector<cv::Point3f>>(nMarkers, std::vector<cv::Point3f>());
        for (uint i = 0; i < nMarkers; ++i)
        {
            cv::Point3f markerPos = markerPositions[i];
            boardCorners[i].push_back(markerPos); // top-left
            markerPos.x += markerSize;
            boardCorners[i].push_back(markerPos); // top-right
            markerPos.y -= markerSize;
            boardCorners[i].push_back(markerPos); // bottom-right
            markerPos.x -= markerSize;
            boardCorners[i].push_back(markerPos); // bottom-left
        }

        try
        {
            pArucoBoard = cv::aruco::Board::create(boardCorners, pArucoDictionary,
                                                   markerIds);
        }
        catch (cv::Exception& ex)
        {
            RCLCPP_FATAL(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                         "cv::exception: %s", ex.what());
            return;
        }
    }

    /**
     * @brief Read parameters from given yaml file.
     *
     * @param[in] iFilePath Path to yaml file. It is assumed that the yaml file exist.
     * Thus, no check is performed.
     */
    void readFromYamlFile(const std::string iFilePath)
    {
        try
        {
            // file storage object to read file
            cv::FileStorage fileStorage;

            fileStorage.open(iFilePath, cv::FileStorage::READ);

            //--- read data from file

            if (fileStorage.isOpened())
            {
                boardSize = cv::Size2f(fileStorage["board_width"].operator float(),
                                       fileStorage["board_height"].operator float());
                if (boardSize.empty())
                    RCLCPP_WARN(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                                "%s: Board size ('board_width', 'board_height') in target "
                                "configuration file not specified!",
                                __PRETTY_FUNCTION__);

                markerSize = fileStorage["marker_size"].operator float();
                if (markerSize == 0.0f)
                    RCLCPP_WARN(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                                "%s: Size of ArUco markers ('marker_size') in target configuration "
                                "file not specified!",
                                __PRETTY_FUNCTION__);

                cv::Mat markerIdsMat = fileStorage["marker_ids"].mat();
                for (int r = 0; r < markerIdsMat.rows; ++r)
                    markerIds.push_back(markerIdsMat.at<int>(r, 0));
                if (markerIds.empty())
                    RCLCPP_WARN(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                                "%s: Ids of ArUco markers ('marker_ids') in target configuration "
                                "file not specified!",
                                __PRETTY_FUNCTION__);

                cv::Mat markerPosMat = fileStorage["marker_positions"].mat();
                for (int r = 0; r < markerPosMat.rows; ++r)
                    markerPositions.push_back(cv::Point3f(markerPosMat.at<float>(r, 0),
                                                          markerPosMat.at<float>(r, 1),
                                                          0));
                if (markerPosMat.empty())
                    RCLCPP_WARN(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                                "%s: Positions of ArUco markers ('marker_positions') in target "
                                "configuration file not specified!",
                                __PRETTY_FUNCTION__);

                if (markerIds.size() != markerPositions.size())
                    RCLCPP_WARN(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                                "%s: Unequal number of ArUco marker IDs ('marker_ids') and marker "
                                "positions ('marker_positions') in target configuration file not "
                                "specified!",
                                __PRETTY_FUNCTION__);

                cv::Mat cutoutsMat = fileStorage["cutouts"].mat();
                if (!cutoutsMat.empty())
                {

                    //--- Loop over columns of cutoutMat, select specific cutout implementation based
                    //--- on the id and then parse the parameters according to the required number
                    //--- specified by the cutout implementation.

                    int colIdx = 0;
                    while (colIdx < cutoutsMat.cols)
                    {
                        auto parseParametersFn = [&](Cutout* ioCutoutObj) -> void
                        {
                            std::vector<float> cutoutCoefficients;
                            for (int i = 1; i <= ioCutoutObj->getNumCoefficients(); ++i)
                                cutoutCoefficients.push_back(cutoutsMat.at<float>(0, colIdx + i));
                            ioCutoutObj->setCoefficients(cutoutCoefficients);
                        };

                        std::shared_ptr<Cutout> pCutout = std::make_shared<CircularCutout>();
                        if (static_cast<int>(cutoutsMat.at<float>(0, colIdx)) ==
                            pCutout->getGeometryId())
                        {
                            parseParametersFn(pCutout.get());
                        }

                        pBoardCutouts.push_back(pCutout);
                        colIdx += pCutout->getNumCoefficients() + 1;
                    }

                    if (cutoutsMat.rows > 1)
                        RCLCPP_WARN(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                                    "%s: Specification of cutouts in target configuration file "
                                    "do not match desired format (i.e. single-row matrix).",
                                    __PRETTY_FUNCTION__);
                }

                minMarkerDetection = fileStorage["min_marker_detection"].operator int();
                if (minMarkerDetection < 1 ||
                    static_cast<uint>(minMarkerDetection) > markerIds.size())
                {
                    RCLCPP_WARN(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                                "%s: min_marker_detection exceeds limits "
                                "[1, <number of marker ids>]. "
                                "Truncating min_marker_detection at limits.",
                                __PRETTY_FUNCTION__);

                    minMarkerDetection = std::max(1,
                                                  std::min(minMarkerDetection,
                                                           static_cast<int>(markerIds.size())));
                }

                cadModelMeshPath = fileStorage["cad_model_mesh"].operator std::string();
                if (cadModelMeshPath.empty())
                {
                    RCLCPP_WARN(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                                "%s: cad_model_mesh is empty, i.e. no file path to a CAD model is "
                                "provided. This will hinder a pose optimization.",
                                __PRETTY_FUNCTION__);
                }
                else
                {
                    //--- if path is relative, make it absolute with respect to path of the yaml file
                    cadModelMeshPath = boost::filesystem::absolute(
                                         boost::filesystem::path(cadModelMeshPath),
                                         boost::filesystem::path(iFilePath).parent_path())
                                         .string();
                }

                cadModelCloudPath = fileStorage["cad_model_cloud"].operator std::string();
                if (cadModelCloudPath.empty())
                {
                    RCLCPP_WARN(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"),
                                "%s: cad_model_cloud is empty, i.e. no file path to a CAD model is "
                                "provided. This will hinder a pose optimization.",
                                __PRETTY_FUNCTION__);
                }
                else
                {
                    //--- if path is relative, make it absolute with respect to path of the yaml file
                    cadModelCloudPath = boost::filesystem::absolute(
                                          boost::filesystem::path(cadModelCloudPath),
                                          boost::filesystem::path(iFilePath).parent_path())
                                          .string();
                }
            }
        }
        catch (cv::Exception& ex)
        {
            RCLCPP_FATAL(rclcpp::get_logger("multisensor_calibration::CalibrationTarget"), "cv::Exception: %s", ex.what());
            return;
        }

        createArUcoBoard();
    }

    /**
     * @brief Returns true if target specifications are valid. False, otherwise.
     */
    bool isValid() const
    {
        bool retVal = true;

        retVal &= !boardSize.empty();
        retVal &= (markerSize > 0.0);
        retVal &= !markerIds.empty();
        retVal &= !markerPositions.empty();
        retVal &= (markerIds.size() == markerPositions.size());
        retVal &= (pArucoBoard != nullptr);
        retVal &= (pArucoDictionary != nullptr);

        return retVal;
    }

    /**
     * @brief Method to test whether a point lies inside a cutout.
     *
     * Loops over all cutouts and performs individual test.
     *
     * @param[in] iPntX X-value of the point in the local coordinate system of the calibration target.
     * @param[in] iPntY Y-value of the point in the local coordinate system of the calibration target.
     * @param[out] opDistance Pointer to return value providing the distance to the center of the
     * cutout. Set to nullptr if not used.
     * @param[out] opPenalty Pointer to return value providing a penalty based on the distance of
     * the point to the center of the cutout.
     * @return True, if point is inside the cutout. False, otherwise.
     */
    bool isLocalPointInsideCutout(const float iPntX, const float iPntY,
                                  float* opDistance = nullptr, float* opPenalty = nullptr) const
    {
        for (uint i = 0; i < pBoardCutouts.size(); i++)
        {
            if (pBoardCutouts[i]->isPointInside(iPntX, iPntY, opDistance, opPenalty))
                return true;
        }

        return false;
    }

    /**
     * @overload
     * @brief Method to test whether a point lies inside a cutout.
     *
     * Loops over all cutouts and performs individual test.
     *
     * @param[in] iPnt 2D point given in the local coordinate system of the calibration target.
     * @param[out] opDistance Pointer to return value providing the distance to the center of the
     * cutout. Set to nullptr if not used.
     * @param[out] opPenalty Pointer to return value providing a penalty based on the distance of
     * the point to the center of the cutout.
     */
    bool isLocalPointInsideCutout(const cv::Vec2f iPnt,
                                  float* opDistance = nullptr, float* opPenalty = nullptr) const
    {
        return isLocalPointInsideCutout(iPnt(0), iPnt(1), opDistance, opPenalty);
    }

    /**
     * @brief Method to test whether a point (given in local coordinates) is an inlier, i.e. lies
     * on target board.
     *
     * @param[in] iPntX X-value of the point in the local coordinate system of the calibration target.
     * @param[in] iPntY Y-value of the point in the local coordinate system of the calibration target.
     * @param[out] opDistance Pointer to return value providing the distance. Set to nullptr if not used.
     * @param[out] opPenalty Pointer to return value providing a penalty based on the distance.
     * @return True, if point lies on the board. False, otherwise.
     */
    bool isLocalPointInlier(const float iPntX, const float iPntY,
                            float* opDistance = nullptr, float* opPenalty = nullptr) const
    {
        bool isInlier = true;

        //--- check if point lies inside calibration target (origin of target is in center)
        const float WIDTH_2  = boardSize.width / 2.f;
        const float HEIGHT_2 = boardSize.height / 2.f;
        if ((iPntX > -WIDTH_2 && iPntX < WIDTH_2) && (iPntY > -HEIGHT_2 && iPntY < HEIGHT_2))
        {
            isInlier = true;

            //--- set distance and penalty if not nullptr
            if (opDistance)
                *opDistance = 0;
            if (opPenalty)
                *opPenalty = 0;
        }
        else if (iPntX <= -WIDTH_2 || iPntY <= -HEIGHT_2)
        {
            isInlier = false;

            float tmpX = std::min(0.f, iPntX + WIDTH_2);
            float tmpY = std::min(0.f, iPntY + HEIGHT_2);

            //--- set distance and penalty if not nullptr
            if (opDistance)
                *opDistance = std::sqrt(tmpX * tmpX + tmpY * tmpY);
            if (opPenalty)
                *opPenalty = 1;
        }
        else if (iPntX >= WIDTH_2 || iPntY >= HEIGHT_2)
        {
            isInlier = false;

            float tmpX = std::max(0.f, iPntX - WIDTH_2);
            float tmpY = std::max(0.f, iPntY - HEIGHT_2);

            //--- set distance and penalty if not nullptr
            if (opDistance)
                *opDistance = std::sqrt(tmpX * tmpX + tmpY * tmpY);
            if (opPenalty)
                *opPenalty = 1;
        }

        //--- if isInlier check if it is inside cutout
        if (isInlier)
        {
            //--- check if point lies inside cutout
            float tmpDistance, tmpPenalty;
            if (isLocalPointInsideCutout(iPntX, iPntY, &tmpDistance, &tmpPenalty))
            {
                isInlier = false;

                //--- set distance and penalty if not nullptr
                if (opDistance)
                    *opDistance = tmpDistance;
                if (opPenalty)
                    *opPenalty = tmpPenalty;
            }
        }

        return isInlier;
    }

    /**
     * @brief Compute 3D coordinates of each marker corner based on the 6DOF pose of the calibration
     * target.
     *
     * @tparam PointT Class used for the marker corner points
     * @param[in] iCenter Center point of the calibration target pose.
     * @param[in] iUp Up vector of the calibration target pose.
     * @param[in] iNormal Normal vector of the calibration target pose.
     * @param[out] oMarkerIds List of marker IDs for which the coordinates were computed.
     * @param[out] oMarkerCorners List of 4D arrays holding the 3D corner points of each marker
     * corresponding to the ID in oMarkerIDs. The corner points are given in clockwise order,
     * starting from the top left.
     */
    template <typename PointT>
    void computeMarkerCornersFromPose(const Eigen::Vector3f& iCenter,
                                      const Eigen::Vector3f& iUp,
                                      const Eigen::Vector3f& iNormal,
                                      std::vector<uint>& oMarkerIds,
                                      std::vector<std::array<PointT, 4>>& oMarkerCorners)
    {
        //--- compute pose
        lib3d::Extrinsics pose;
        computePose(iCenter, iUp, iNormal, pose);
        const cv::Matx44d POSE_RT = pose.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF);

        // offset of marker corner from top-left corner (origin) of marker
        std::vector<cv::Point3f> cornerOffsets = {cv::Point3f(0, 0, 0),                    // top-left
                                                  cv::Point3f(markerSize, 0, 0),           // top-right
                                                  cv::Point3f(markerSize, -markerSize, 0), // bottom-right
                                                  cv::Point3f(0, -markerSize, 0)};         // bottom-left

        oMarkerIds.clear();
        oMarkerCorners.clear();
        for (uint i = 0; i < markerIds.size(); ++i)
        {
            oMarkerIds.push_back(markerIds[i]);
            oMarkerCorners.push_back(std::array<PointT, 4>());
            for (uint j = 0; j < 4; ++j)
            {
                cv::Matx31d markerCorner =
                  (POSE_RT * cv::Vec4d(markerPositions[i].x + cornerOffsets[j].x,
                                       markerPositions[i].y + cornerOffsets[j].y,
                                       markerPositions[i].z + cornerOffsets[j].z,
                                       1))
                    .get_minor<3, 1>(0, 0);
                oMarkerCorners.back()[j] =
                  PointT(static_cast<typename PointT::value_type>(markerCorner(0, 0)),
                         static_cast<typename PointT::value_type>(markerCorner(1, 0)),
                         static_cast<typename PointT::value_type>(markerCorner(2, 0)));
            }
        }
    }

    /**
     * @brief Get 3D coordinates of each marker corner relative to the top left corner (Z = 0)
     *
     * @param[out] oMarkerCorners List of 4D arrays holding the 3D corner points of each marker
     * corresponding to the ID in oMarkerIDs. The corner points are given in clockwise order,
     * starting from the top left.
     */
    void getMarkerCornersRelative(std::vector<std::vector<cv::Point3f>>& oMarkerPoints, std::vector<uint>& oMarkerIds) const
    {
        //--- compute pose
        oMarkerIds.clear();
        oMarkerPoints.clear();
        for (uint i = 0; i < markerIds.size(); ++i)
        {
            oMarkerIds.push_back(markerIds.at(i));
            oMarkerPoints.push_back(boardCorners.at(i));
        }
    }

    /**
     * @brief Compute calibration target pose from coordinates of top-left marker corners.
     *
     * @tparam PointT Class used for the marker corner points
     * @param[in] iMarkerIds List of marker IDs for which the top-left corner coordinate has been
     * observed.
     * @param[in] iTlMarkerCorners 3D points of the top-left marker corners corresponding to the
     * IDs in iMarkerIds
     * @param[out] oCenter Translational vector as seen from the reference coordinate system.
     * @param[out] oUp Up vector as seen from the reference coordinate system (normalized).
     * @param[out] oRight Right vector as seen from the reference coordinate system (normalized).
     * @param[out] oNormal Normal vector as seen from the reference coordinate system and pointing
     * into direction of the origin of the reference coordinate system (normalized).
     */
    template <typename PointT>
    void computePoseFromTopLeftMarkerCorners(const std::vector<uint>& iMarkerIds,
                                             const std::vector<PointT>& iTlMarkerCorners,
                                             Eigen::Vector3f& oCenter,
                                             Eigen::Vector3f& oUp,
                                             Eigen::Vector3f& oRight,
                                             Eigen::Vector3f& oNormal)
    {
        //--- get indices of markers according to ids
        // uint marker1Idx = std::distance(std::find(iMarkerIds.cbegin(), iMarkerIds.cend(), 1),
        //                                 iMarkerIds.cbegin());
        // uint marker2Idx = std::distance(std::find(iMarkerIds.cbegin(), iMarkerIds.cend(), 2),
        //                                 iMarkerIds.cbegin());
        // uint marker3Idx = std::distance(std::find(iMarkerIds.cbegin(), iMarkerIds.cend(), 3),
        //                                 iMarkerIds.cbegin());
        // uint marker4Idx = std::distance(std::find(iMarkerIds.cbegin(), iMarkerIds.cend(), 4),
        //                                 iMarkerIds.cbegin());
        uint marker1Idx = std::find(iMarkerIds.cbegin(), iMarkerIds.cend(), 1) - iMarkerIds.cbegin();
        uint marker2Idx = std::find(iMarkerIds.cbegin(), iMarkerIds.cend(), 2) - iMarkerIds.cbegin();
        uint marker3Idx = std::find(iMarkerIds.cbegin(), iMarkerIds.cend(), 3) - iMarkerIds.cbegin();
        uint marker4Idx = std::find(iMarkerIds.cbegin(), iMarkerIds.cend(), 4) - iMarkerIds.cbegin();

        //--- get 3D points of top-left marker corners
        const PointT& tlMarkerCorner1 = iTlMarkerCorners.at(marker1Idx);
        const PointT& tlMarkerCorner2 = iTlMarkerCorners.at(marker2Idx);
        const PointT& tlMarkerCorner3 = iTlMarkerCorners.at(marker3Idx);
        const PointT& tlMarkerCorner4 = iTlMarkerCorners.at(marker4Idx);

        //--- FIND UP VECTOR
        //--- TODO: generalize finding marker combination, which is best suited to compute up vector

        // up vector between marker 4 and 1
        Eigen::Vector3f upVec41 =
          Eigen::Vector3f(static_cast<float>(tlMarkerCorner1.x - tlMarkerCorner4.x),
                          static_cast<float>(tlMarkerCorner1.y - tlMarkerCorner4.y),
                          static_cast<float>(tlMarkerCorner1.z - tlMarkerCorner4.z));
        // up vector between marker 3 and 2
        Eigen::Vector3f upVec32 =
          Eigen::Vector3f(static_cast<float>(tlMarkerCorner2.x - tlMarkerCorner3.x),
                          static_cast<float>(tlMarkerCorner2.y - tlMarkerCorner3.y),
                          static_cast<float>(tlMarkerCorner2.z - tlMarkerCorner3.z));

        oUp = (upVec41 + upVec32) / 2.f;
        oUp.normalize();

        //--- FIND RIGHT VECTOR
        //--- TODO: generalize finding marker combination, which is best suited to compute up vector

        // right vector between marker 1 and 2
        Eigen::Vector3f rightVec12 =
          Eigen::Vector3f(static_cast<float>(tlMarkerCorner2.x - tlMarkerCorner1.x),
                          static_cast<float>(tlMarkerCorner2.y - tlMarkerCorner1.y),
                          static_cast<float>(tlMarkerCorner2.z - tlMarkerCorner1.z));
        // right vector between marker 4 and 3
        Eigen::Vector3f rightVec43 =
          Eigen::Vector3f(static_cast<float>(tlMarkerCorner3.x - tlMarkerCorner4.x),
                          static_cast<float>(tlMarkerCorner3.y - tlMarkerCorner4.y),
                          static_cast<float>(tlMarkerCorner3.z - tlMarkerCorner4.z));

        oRight = (rightVec12 + rightVec43) / 2.f;
        oRight.normalize();

        //--- compute normal vector
        oNormal = oRight.cross(oUp);

        //--- compute center
        //--- TODO: generalize
        Eigen::Vector3f center1 = Eigen::Vector3f(static_cast<float>(tlMarkerCorner1.x),
                                                  static_cast<float>(tlMarkerCorner1.y),
                                                  static_cast<float>(tlMarkerCorner1.z)) +
                                  std::abs(markerPositions[0].x) * oRight +
                                  std::abs(markerPositions[0].y) * oUp * -1.f;
        Eigen::Vector3f center2 = Eigen::Vector3f(static_cast<float>(tlMarkerCorner2.x),
                                                  static_cast<float>(tlMarkerCorner2.y),
                                                  static_cast<float>(tlMarkerCorner2.z)) +
                                  std::abs(markerPositions[1].x) * oRight * -1.f +
                                  std::abs(markerPositions[1].y) * oUp * -1.f;
        Eigen::Vector3f center3 = Eigen::Vector3f(static_cast<float>(tlMarkerCorner3.x),
                                                  static_cast<float>(tlMarkerCorner3.y),
                                                  static_cast<float>(tlMarkerCorner3.z)) +
                                  std::abs(markerPositions[2].x) * oRight * -1.f +
                                  std::abs(markerPositions[2].y) * oUp;
        Eigen::Vector3f center4 = Eigen::Vector3f(static_cast<float>(tlMarkerCorner4.x),
                                                  static_cast<float>(tlMarkerCorner4.y),
                                                  static_cast<float>(tlMarkerCorner4.z)) +
                                  std::abs(markerPositions[3].x) * oRight +
                                  std::abs(markerPositions[3].y) * oUp;

        oCenter = (center1 + center2 + center3 + center4) / 4.f;
    }

    /**
     * @brief Compute extrinsic pose based on given vectors.
     *
     * @param[in] iCenter Translational vector as seen from the reference coordinate system.
     * @param[in] iUp Up vector as seen from the reference coordinate system.
     * @param[in] iNormal Normal vector as seen from the reference coordinate system and pointing
     * into direction of the origin of the reference coordinate system.
     * @param[out] oPose Computed pose with transformation direction local to ref.
     */
    static void computePose(const Eigen::Vector3f& iCenter,
                            const Eigen::Vector3f& iUp,
                            const Eigen::Vector3f& iNormal,
                            lib3d::Extrinsics& oPose)
    {
        //--- set transformation direction
        oPose.setTransfDirection(lib3d::Extrinsics::LOCAL_2_REF);

        //--- set center as translation
        oPose.setTranslationVec(iCenter.x(), iCenter.y(), iCenter.z());

        //--- set rotation matrix as right, down and negative normal as x-, y- and z-axis, respectively
        Eigen::Vector3f normalNorm = iNormal.normalized();
        Eigen::Vector3f upNorm     = iUp.normalized();
        Eigen::Vector3f rightNorm  = upNorm.cross(normalNorm);
        cv::Matx33d rotationMat    = cv::Matx33d(rightNorm.x(), upNorm.x(), normalNorm.x(),
                                                 rightNorm.y(), upNorm.y(), normalNorm.y(),
                                                 rightNorm.z(), upNorm.z(), normalNorm.z());
        oPose.setRotationMat(rotationMat);
    }

    /**
     * @brief Decompose extrinsic pose into three vectors.
     *
     * @param[in] iPose Extrinsic pose.
     * @param[out] oCenter Translational vector as seen from the reference coordinate system.
     * @param[out] oUp Up vector as seen from the reference coordinate system (normalized).
     * @param[out] oNormal Normal vector as seen from the reference coordinate system and pointing
     * into direction of the origin of the reference coordinate system (normalized).
     */
    static void decomposePose(const lib3d::Extrinsics& iPose,
                              Eigen::Vector3f& oCenter,
                              Eigen::Vector3f& oUp,
                              Eigen::Vector3f& oNormal)
    {
        //--- set transformation direction
        Eigen::Matrix4d rtMatrix;
        cv::cv2eigen(iPose.getRTMatrix(lib3d::Extrinsics::LOCAL_2_REF), rtMatrix);

        //--- get center from translation
        oCenter = rtMatrix.block<3, 1>(0, 3).cast<float>();

        //--- up and normal vector from rotation
        oUp     = rtMatrix.block<3, 1>(0, 1).cast<float>().normalized();
        oNormal = rtMatrix.block<3, 1>(0, 2).cast<float>().normalized();
    }

    /**
     * @brief Compute 4D plane parameters from lib3D::Extrinsics pose.
     *
     * @param[in] iPose Board pose.
     * @param[out] oPlaneParameters 4D plane parameters.
     */
    static void computePlaneParametersFromPose(const lib3d::Extrinsics& iPose,
                                               cv::Vec4d& oPlaneParameters)
    {
        //--- compute plane parameterization of target board
        //--- get rotation matrix of board pose (local_2_ref)
        //--- the columns of this matrix encode the orientation of the axes of the
        //--- board coordinates system as seen from the camera coordinate system
        //--- the normal vector can then be deduced from the 3rd column (z-axis)
        //--- the orthogonal distance can then be computed by using the normal vector and the
        //--- translation vector of the detected board pose

        //--- set transformation direction
        lib3d::Extrinsics pose = (iPose.getTransfDirection() == lib3d::Extrinsics::LOCAL_2_REF)
                                   ? iPose
                                   : iPose.getInverse();

        // rotation matrix of the board as seen from the camera coordinate system
        cv::Matx33d local2RefRotMat = pose.getRotationMat();

        // plane parameterization of the calibration board
        oPlaneParameters = cv::Vec4d(local2RefRotMat(0, 2),
                                     local2RefRotMat(1, 2),
                                     local2RefRotMat(2, 2), 0);
        oPlaneParameters(3) =
          -1 * pose.getTranslationVec().dot(oPlaneParameters.get_minor<3, 1>(0, 0));
    }
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CALIBRATIONTARGET_HPP
