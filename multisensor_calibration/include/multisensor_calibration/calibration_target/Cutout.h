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

#ifndef MULTISENSORCALIBRATION_CUTOUT_H
#define MULTISENSORCALIBRATION_CUTOUT_H

// Std
#include <cmath>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

// OpenCV
#include <opencv2/core.hpp>

namespace multisensor_calibration
{

/**
 * @ingroup  target_detection
 * @brief Abstract base struct of a cutout geometry from the calibration board.
 *
 * This is used to determine whether a given point within the local coordinate system of the board
 * lies inside a cutout and, thus, has no 3D information
 */
class Cutout
{

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Construct setting the geometry ID and the number of coefficients.
     */
    Cutout(const int iGeometryId, const int iNumCoefficients);

    /**
     * @brief Construct setting the geometry ID, the number of coefficients, as well as the coefficients
     * themself.
     */
    Cutout(const int iGeometryId, const int iNumCoefficients, const std::vector<float> iCoefficients);

    /**
     * @brief Destructor
     */
    virtual ~Cutout();

    /**
     * @brief Returns ID of cutout. Identifying the geometry.
     */
    int getGeometryId() const;

    /**
     * @brief Returns number of coefficients needed to define the geometry.
     */
    int getNumCoefficients() const;

    /**
     * @brief Returns model coefficients defining the specific geometry.
     */
    std::vector<float> getCoefficients() const;

    /**
     * @brief Get radius of cutout. If cutout is not circular, this should be the biggest radius
     * which fully fits into the cutout.
     */
    virtual float getRadius() const;

    /**
     * @brief Get X coordinate of center point.
     */
    virtual float getCenterX() const;

    /**
     * @brief Get Y coordinate of center point.
     */
    virtual float getCenterY() const;

    /**
     * @brief Set model coefficients defining the specific geometry.
     *
     * @param iCoefficients
     */
    void setCoefficients(const std::vector<float>& iCoefficients);

    /**
     * @brief Method to test whether a point lies inside the cutout.
     *
     * @param[in] iPnt 2D point given in the local coordinate system of the calibration target.
     * @param[out] opDistance Pointer to return value providing the distance to the center of the
     * cutout. Set to nullptr if not used.
     * @param[out] opPenalty Pointer to return value providing a penalty based on the distance of
     * the point to the center of the cutout.
     * @return True, if point is inside the cutout. False, otherwise.
     */
    bool isPointInside(const cv::Vec2f iPnt,
                       float* opDistance = nullptr, float* opPenalty = nullptr) const;

    /**
     * @overload
     * @brief Method to test whether a point lies inside the cutout.
     *
     * @param[in] x X-value of the point in the local coordinate system of the calibration target.
     * @param[in] y Y-value of the point in the local coordinate system of the calibration target.
     * @param[out] opDistance Pointer to return value providing the distance to the center of the
     * cutout. Set to nullptr if not used.
     * @param[out] opPenalty Pointer to return value providing a penalty based on the distance of
     * the point to the center of the cutout.
     * @return True, if point is inside the cutout. False, otherwise.
     */
    virtual bool isPointInside(const float x, const float y,
                               float* opDistance = nullptr, float* opPenalty = nullptr) const = 0;

    //--- MEMBER DECLARATION ---//

  private:
    /// Geometry specific ID
    int geometryId_;

    /// Number of coefficients needed to define the geometry
    int numCoefficients_;

    /// Coefficients defining the specific geometry
    std::vector<float> coefficients_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_CUTOUT_H