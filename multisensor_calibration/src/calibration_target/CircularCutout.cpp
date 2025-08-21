/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration_target/CircularCutout.h"

namespace multisensor_calibration
{

//==================================================================================================
CircularCutout::CircularCutout() :
  Cutout(1, 3)
{
}

//==================================================================================================
CircularCutout::CircularCutout(const std::vector<float> iCoefficients) :
  Cutout(1, 3, iCoefficients)
{
}

//==================================================================================================
CircularCutout::~CircularCutout()
{
}

//==================================================================================================
float CircularCutout::getRadius() const
{
    return getCoefficients()[2];
}

//==================================================================================================
float CircularCutout::getCenterX() const
{
    return getCoefficients()[0];
}

//==================================================================================================
float CircularCutout::getCenterY() const
{
    return getCoefficients()[1];
}

//==================================================================================================
bool CircularCutout::isPointInside(const float x, const float y,
                                   float* opDistance, float* opPenalty) const
{
    cv::Vec2d centerPnt = cv::Vec2d(getCenterX(), getCenterY());
    double radius       = getRadius();
    double norm         = cv::norm(cv::Vec2d(x, y), centerPnt, cv::NORM_L2);
    bool isInside       = (norm <= radius) ? true : false;

    if (opDistance)
        *opDistance = static_cast<float>(norm);

    if (opPenalty)
        *opPenalty = 10.f;
    // *opPenalty = -(5.f / radius) * norm + 6.f;
    // *opPenalty = 1.f - (1.f / radius) * static_cast<float>(std::log(norm / radius));

    return isInside;
}

} // namespace multisensor_calibration