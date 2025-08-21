/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/calibration_target/Cutout.h"

#include "multisensor_calibration/common/common.h"

namespace multisensor_calibration
{

//==================================================================================================
Cutout::Cutout(const int iId, const int iNumCoefficients) :
  Cutout(iId, iNumCoefficients, {})
{
}

//==================================================================================================
Cutout::Cutout(const int iId, const int iNumCoefficients, const std::vector<float> iCoefficients) :
  geometryId_(iId),
  numCoefficients_(iNumCoefficients),
  coefficients_(iCoefficients)
{
    assert(iCoefficients.empty() || iCoefficients.size() == static_cast<uint>(iNumCoefficients));
}

//==================================================================================================
Cutout::~Cutout()
{
}

//==================================================================================================
int Cutout::getGeometryId() const
{
    return geometryId_;
}

//==================================================================================================
int Cutout::getNumCoefficients() const
{
    return numCoefficients_;
}

//==================================================================================================
std::vector<float> Cutout::getCoefficients() const
{
    return coefficients_;
}

//==================================================================================================
float Cutout::getRadius() const
{
    return 0.f;
}

//==================================================================================================
float Cutout::getCenterX() const
{
    return 0.f;
}

//==================================================================================================
float Cutout::getCenterY() const
{
    return 0.f;
}

//==================================================================================================
void Cutout::setCoefficients(const std::vector<float>& iCoefficients)
{
    assert(iCoefficients.size() == static_cast<uint>(numCoefficients_));

    coefficients_ = iCoefficients;
}

//==================================================================================================
bool Cutout::isPointInside(const cv::Vec2f iPnt,
                           float* opDistance, float* opPenalty) const
{
    UNUSED_VAR(opDistance);
    UNUSED_VAR(opPenalty);

    return isPointInside(iPnt(0), iPnt(1));
}

} // namespace multisensor_calibration