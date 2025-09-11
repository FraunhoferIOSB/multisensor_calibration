#ifndef LIB3D_INTRINSICS_H
#define LIB3D_INTRINSICS_H

// Std
#define _USE_MATH_DEFINES
#include <cmath>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "geometry.hpp"
#include "exceptions.hpp"

namespace lib3d
{

  /**
   @ingroup camera_geom
   @brief Implementation of the intrinsic parameterization of cameras.

   Apart form the image size, each camera has a projction model, which is represented by a specific
   calibration matrix \f$\mathrm{K}\f$ and performs the mapping of a 3D scene point (given in the
   camera coordinate system) to 2D point in the image, as well as a distortion model.

   <h3>Projection</h3>
   Most cameras have either a perspective or orthographic (parallel) projection (cf. chapter 6 of
   (Hartley & Zisserman, 2004)@cite Hartley2004).

   <h4>Perspective Projection</h4>
   The perspective projection model (central projection) as described in is the most commonly used projection model for
   cameras. It is described by a \f$3\times 3\f$ calibration matrix:
   \f[
      K  =
      \begin{bmatrix}
      f_x & \alpha & c_x  \\
      0   & f_y & c_y  \\
      0   & 0   & 1
      \end{bmatrix} ,
   \f]
   with \f$f_x\f$ and \f$f_y\f$ being the focal length, \f$c_x\f$ and \f$c_y\f$ being the coordinates
   of the principal point and \f$\alpha \f$ being the skew factor.

   <h4>Orthographic Projection</h4>
   The orthographic projection model (parallel projection) is described by a \f$3\times 3\f$ calibration matrix:
   \f[
      K  =
      \begin{bmatrix}
      f_x & \alpha & c_x  \\
      0   & f_y & c_y  \\
      0   & 0   & 0
      \end{bmatrix} ,
   \f]
   with \f$f_x\f$ and \f$f_y\f$ being the scaling factor, \f$c_x\f$ and \f$c_y\f$ being the coordinates
   of the principal point and \f$\alpha \f$ being the skew factor.

   Note the difference in the bottom-right place of the matrix. The omission of '1' will result in a loss
   of depth information when 3D scene points are projected into 2D image points. Thus resulting in a
   parallel projection.

   <h3>Distortion</h3>

   This class supports the distortion models of [OpenCV](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
   with a maximum of 14 distortion parameters (2 for tangential distortion, 6 for radial distortion, 4
   for prism distortion and 2 perspective distortion (Scheimpflug principle).
   */
  class Intrinsics
  {
    //--- ENUM DECLERATION ---//

  public:
    /**
     @brief Enumeration holding possible projection models for the camera intrinsics.
     */
    enum EProjectionModel
    {
      PERSPECTIVE = 0, ///< Perspective projection

      ORTHOGRAPHIC ///< Orthographic or parallel projection
    };

    //--- METHOD DECLERATION ---//

  public:
    /**
     @brief Default constructor, performing a zero initialization.
     */
    explicit Intrinsics() : Intrinsics(cv::Size(), cv::Point2d(), cv::Point2d(), cv::Mat(), 0, PERSPECTIVE)
    {
    }

    /**
     @brief Initialization constructor
     @param[in] iImageSize Initialization of the frame size.
     @param[in] iFocalLength Initialization of the focal length.
     @param[in] iPrincipalPnt Initialization of the principal point.
     @param[in] iDistCoeffs Initialization of the distortion coefficients.
                See https://docs.opencv.org/master/d9/d0c/group__calib3d.html for more information.
     @param[in] iSkew Initialization of skew factor.
     @param[in] iProjMod Initialization of projection model.
     */
    explicit Intrinsics(const cv::Size &iImageSize, const cv::Point2d &iFocalLength,
                        const cv::Point2d &iPrincipalPnt, const cv::Mat &iDistCoeffs = cv::Mat(),
                        const double &iSkew = 0, const EProjectionModel &iProjMod = PERSPECTIVE) : mProjectionModel(iProjMod),
                                                                                                   mImageSize(iImageSize),
                                                                                                   mFocalLength(iFocalLength),
                                                                                                   mPrincipalPnt(iPrincipalPnt),
                                                                                                   mDistortionCoeffs(iDistCoeffs),
                                                                                                   mSkew(iSkew)
    {
    }

    /**
     @brief Initialization constructor

     @overload
     */
    explicit Intrinsics(const int &iWidth, const int &iHeight, const double &iF, const double &iCx,
                        const double &iCy, const cv::Mat &iDistCoeff = cv::Mat(),
                        const double &iSkew = 0, const EProjectionModel &iProjMod = PERSPECTIVE) : Intrinsics(cv::Size(iWidth, iHeight), cv::Point2d(iF, iF), cv::Point2d(iCx, iCy),
                                                                                                              iDistCoeff, iSkew, iProjMod)

    {
    }

    /**
     @brief Initialization constructor

     @overload
     */
    explicit Intrinsics(const int &iWidth, const int &iHeight, const double &iFx, const double &iFy,
                        const double &iCx, const double &iCy, const cv::Mat &iDistCoeff = cv::Mat(),
                        const double &iSkew = 0, const EProjectionModel &iProjMod = PERSPECTIVE) : Intrinsics(cv::Size(iWidth, iHeight), cv::Point2d(iFx, iFy), cv::Point2d(iCx, iCy),
                                                                                                              iDistCoeff, iSkew, iProjMod)
    {
    }

    /**
     @brief Copy constructor
     */
    Intrinsics(const Intrinsics &rhs)
    {
      mProjectionModel = rhs.mProjectionModel;
      mImageSize = rhs.mImageSize;
      mFocalLength = rhs.mFocalLength;
      mPrincipalPnt = rhs.mPrincipalPnt;
      mDistortionCoeffs = rhs.mDistortionCoeffs;
      mSkew = rhs.mSkew;
    }

    /**
     @brief Move constructor.
     */
    Intrinsics(Intrinsics &&rhs)
    {
      mProjectionModel = rhs.mProjectionModel;
      rhs.mProjectionModel = PERSPECTIVE;

      mImageSize = rhs.mImageSize;
      rhs.mImageSize = cv::Size();

      mFocalLength = rhs.mFocalLength;
      rhs.mFocalLength = cv::Point2d();

      mPrincipalPnt = rhs.mPrincipalPnt;
      rhs.mPrincipalPnt = cv::Point2d();

      mDistortionCoeffs = rhs.mDistortionCoeffs;
      rhs.mDistortionCoeffs = cv::Mat();

      mSkew = rhs.mSkew;
      rhs.mSkew = 0.0;
    }

    /**
     @brief Copy assignment operator.
     */
    Intrinsics &operator=(const Intrinsics &rhs)
    {
      mProjectionModel = rhs.mProjectionModel;
      mImageSize = rhs.mImageSize;
      mFocalLength = rhs.mFocalLength;
      mPrincipalPnt = rhs.mPrincipalPnt;
      mDistortionCoeffs = rhs.mDistortionCoeffs;
      mSkew = rhs.mSkew;
      return *this;
    }

    /**
     @brief Move assignment operator.
     */
    Intrinsics &operator=(Intrinsics &&rhs)
    {
      if (this != &rhs)
      {
        mProjectionModel = rhs.mProjectionModel;
        rhs.mProjectionModel = PERSPECTIVE;

        mImageSize = rhs.mImageSize;
        rhs.mImageSize = cv::Size();

        mFocalLength = rhs.mFocalLength;
        rhs.mFocalLength = cv::Point2d();

        mPrincipalPnt = rhs.mPrincipalPnt;
        rhs.mFocalLength = cv::Point2d();

        mDistortionCoeffs = rhs.mDistortionCoeffs;
        rhs.mDistortionCoeffs = cv::Mat();

        mSkew = rhs.mSkew;
        rhs.mSkew = 0.0;
      }

      return *this;
    }

    /**
     @brief Returns projection model which is followed by this object.
     */
    EProjectionModel getProjectionModel() const
    {
      return mProjectionModel;
    }

    /**
     @brief Method to set projection model which should be represented by the camera,
     i.e. PERSPECTIVE or ORTHOGRAPHIC.
     */
    void setProjectionModel(const EProjectionModel &projectionModel)
    {
      mProjectionModel = projectionModel;
    }

    /**
     @brief Returns image size of camera.
     */
    cv::Size getImageSize() const
    {
      return mImageSize;
    }

    /**
     @brief Returns image width of camera.
     */
    int getWidth() const
    {
      return mImageSize.width;
    }

    /**
     @brief Returns image height of camera.
     */
    int getHeight() const
    {
      return mImageSize.height;
    }

    /**
     * @brief Set image size of camera.
     */
    void setImageSize(const cv::Size &iImageSize)
    {
      if (iImageSize.width < 0 || iImageSize.height < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iFrameSize.width < 0 || iFrameSize.height < 0)"));

      mImageSize = iImageSize;
    }

    /**
     @overload
     */
    void setImageSize(const int &iWidth, const int &iHeight)
    {
      if (iWidth < 0 || iHeight < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iWidth < 0 || iHeight < 0)"));

      mImageSize.width = iWidth;
      mImageSize.height = iHeight;
    }

    /**
     @overload
     */
    void setWidth(const int &iWidth)
    {
      if (iWidth < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iWidth < 0)"));

      mImageSize.width = iWidth;
    }

    /**
     @overload
     */
    void setHeight(const int &iHeight)
    {
      if (iHeight < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iHeight < 0)"));

      mImageSize.height = iHeight;
    }

    /**
     @brief Returns focal length of camera as cv::Point2d.
     @note If the projection model is set to Orhtographic this is interpreted as skaling factor.
     */
    cv::Point2d getFocalLength() const
    {
      return mFocalLength;
    }

    /**
     @brief Returns focal length in x-direction.
     */
    double getFx() const
    {
      return mFocalLength.x;
    }

    /**
     @brief Returns focal length in y-direction.
     */
    double getFy() const
    {
      return mFocalLength.y;
    }

    /**
     @brief Method to set focal length of camera.
     @param[in] iFocalLength Focal length.
                <b>NOTE:</b> If the projection model is set to Orhtographic this is interpreted
                as skaling factor.
     */
    void setFocalLength(const cv::Point2d &iFocalLength)
    {
      if (iFocalLength.x < 0 || iFocalLength.y < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iFocalLength.x < 0 || iFocalLength.y < 0)"));

      mFocalLength = iFocalLength;
    }

    /**
     @overload
     */
    void setFocalLength(const double &iFx, const double &iFy)
    {
      if (iFx < 0 || iFy < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iFx < 0 || iFy < 0)"));

      mFocalLength.x = iFx;
      mFocalLength.y = iFy;
    }

    /**
     @brief Method to set focal length in x-direction.
     */
    void setFx(const double &iFx)
    {
      if (iFx < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iFx < 0)"));

      mFocalLength.x = iFx;
    }

    /**
     @brief Method to set focal length in y-direction.
     */
    void setFy(const double &iFy)
    {
      if (iFy < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iFy < 0)"));

      mFocalLength.y = iFy;
    }

    /**
     @brief Returns principal point of camera as cv::Point2d.
     */
    cv::Point2d getPrincipalPnt() const
    {
      return mPrincipalPnt;
    }

    /**
     @brief Returns x-coordinate of principal point.
     */
    double getCx() const
    {
      return mPrincipalPnt.x;
    }

    /**
     @brief Returns y-coordinate of principal point.
     */
    double getCy() const
    {
      return mPrincipalPnt.y;
    }

    /**
     @brief Method to set principal point of camera.
     @param[in] iPrincipalPnt Coordinates of principal point.
     */
    void setPrincipalPnt(const cv::Point2d &iPrincipalPnt)
    {
      if (iPrincipalPnt.x < 0 || iPrincipalPnt.y < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iPrincipalPnt.x < 0 || iPrincipalPnt.y < 0)"));

      mPrincipalPnt = iPrincipalPnt;
    }

    /**
     @overload
     */
    void setPrincipalPnt(const double &iCx, const double &iCy)
    {
      if (iCx < 0 || iCy < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iCx < 0 || iCy < 0)"));

      mPrincipalPnt.x = iCx;
      mPrincipalPnt.y = iCy;
    }

    /**
     @brief Method to set x-coordinate of principal point.
     */
    void setCx(const double &iCx)
    {
      if (iCx < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iCx < 0)"));

      mPrincipalPnt.x = iCx;
    }

    /**
     @brief Method to set y-coordinate of principal point.
     */
    void setCy(const double &iCy)
    {
      if (iCy < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iCy < 0)"));

      mPrincipalPnt.y = iCy;
    }

    /**
     @brief Returns skew factor of camera.
     */
    double getSkew() const
    {
      return mSkew;
    }

    /**
     @brief Method to set skew factor.
     */
    void setSkew(double skew)
    {
      mSkew = skew;
    }

    /**
     @brief Returns the horizontal field of view (in degrees) computed from focal length and sensor
     width.
     @note If not all data is avaliable for computation this will return -1.
     */
    double getHFov() const
    {
      double hFov = -1;

      if (mImageSize.width > 0 && mFocalLength.x > 0)
      {
        hFov = 2 * std::atan(static_cast<double>(mImageSize.width) /
                             (2 * static_cast<double>(mFocalLength.x)));

        hFov = (hFov / M_PI) * 180.0;
      }

      return hFov;
    }

    /**
     @brief Returns the vertical field of view (in degrees) computed from focal length and sensor
     height.
     @note If not all data is avaliable for computation this will return -1.
     */
    double getVFov() const
    {
      double vFov = -1;

      if (mImageSize.height > 0 && mFocalLength.y > 0)
      {
        vFov = 2 * std::atan(static_cast<double>(mImageSize.height) /
                             (2 * static_cast<double>(mFocalLength.y)));

        vFov = (vFov / M_PI) * 180.0;
      }

      return vFov;
    }

    /**
     @brief Method to get the intrinsic parameters as a \f$3\times 3\f$ calibration matrix \f$\mathrm{K}\f$.
     @param[in] iProjectionModel Provide projection model as which \f$\mathrm{K}\f$ is to be returned.
     Default is PERSPECTIVE. <b>NOTE:</b> This will not change the internal projection model of the object.
     */
    cv::Matx33d getK_as3x3(const EProjectionModel &iProjectionModel = PERSPECTIVE) const
    {
      cv::Matx33d K(mFocalLength.x, mSkew, mPrincipalPnt.x,
                    0, mFocalLength.y, mPrincipalPnt.y,
                    0, 0, 1);

      //--- adjust K for orthographic projection
      if (mProjectionModel == ORTHOGRAPHIC || iProjectionModel == ORTHOGRAPHIC)
      {
        K(2, 2) = 0;
      }

      return K;
    }

    /**
     @brief Method to get the intrinsic parameters as a \f$3\times 4\f$ calibration matrix \f$\mathrm{K}\f$.

     In this a zero valued column is appended to \f$K\f$.
     @param[in] iProjectionModel Provide projection model as which \f$\mathrm{K}\f$ is to be returned.
     Default is PERSPECTIVE. <b>NOTE:</b> This will not change the internal projection model of the object.
     */
    cv::Matx34d getK_as3x4(const EProjectionModel &iProjectionModel = PERSPECTIVE) const
    {
      cv::Matx34d K(mFocalLength.x, mSkew, mPrincipalPnt.x, 0,
                    0, mFocalLength.y, mPrincipalPnt.y, 0,
                    0, 0, 1, 0);

      //--- adjust K for orthographic projection
      if (mProjectionModel == ORTHOGRAPHIC || iProjectionModel == ORTHOGRAPHIC)
      {
        K(2, 2) = 0;
        K(2, 3) = 1;
      }

      return K;
    }

    /**
     @brief Method to set the intrinsic parameters by a \f$3\times 3\f$ calibration matrix \f$\mathrm{K}\f$.
     @note If `iK(2,2) == 0`, this is interpreted as an orthographic calibration matrix and thus mProjectionModel
     will be updated.
     */
    void setBy_K(const cv::Matx33d &iK)
    {
      if (iK(0, 0) < 0 || iK(1, 1) < 0 || iK(0, 2) < 0 || iK(1, 2) < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iK(0,0) < 0 || iK(1,1) < 0 || iK(0,2) < 0 || iK(1,2) < 0)"));

      mFocalLength.x = iK(0, 0);
      mFocalLength.y = iK(1, 1);
      mPrincipalPnt.x = iK(0, 2);
      mPrincipalPnt.y = iK(1, 2);
      mSkew = iK(0, 1);

      //--- set projection model
      if (iK(2, 2) == 1.0)
        mProjectionModel = PERSPECTIVE;
      else
        mProjectionModel = ORTHOGRAPHIC;
    }

    /**
     @brief Method to set the intrinsic parameters by a \f$3\times 4\f$ calibration matrix \f$\mathrm{K}\f$.
     @note If `iK(2,2) == 0 && iK(2,3) == 1`, this is interpreted as an orthographic calibration matrix and thus mProjectionModel
     will be updated.
     */
    void setBy_K(const cv::Matx34d &iK)
    {
      if (iK(0, 0) < 0 || iK(1, 1) < 0 || iK(0, 2) < 0 || iK(1, 2) < 0)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(iK(0,0) < 0 || iK(1,1) < 0 || iK(0,2) < 0 || iK(1,2) < 0)"));

      mFocalLength.x = iK(0, 0);
      mFocalLength.y = iK(1, 1);
      mPrincipalPnt.x = iK(0, 2);
      mPrincipalPnt.y = iK(1, 2);
      mSkew = iK(0, 1);

      //--- set projection model
      if (iK(2, 2) == 1.0 && iK(2, 3) == 0.0)
        mProjectionModel = PERSPECTIVE;
      else
        mProjectionModel = ORTHOGRAPHIC;
    }

    /**
     @brief Method to get the projective matrix of the camera as a \f$3\times 3\f$ matrix.
     In case of a perspective camera this should return an identity matrix.
     @param[in] iProjectionModel Provide projection model as which \f$\mathrm{K}\f$ is to be returned.
     Default is PERSPECTIVE. <b>NOTE:</b> This will not change the internal projection model of the object.
     */
    cv::Matx33d getP_as3x3(const EProjectionModel &iProjectionModel = PERSPECTIVE) const
    {
      cv::Matx33d P = cv::Matx33d(1, 0, 0,
                                  0, 1, 0,
                                  0, 0, 1);

      //--- adjust K for orthographic projection
      if (mProjectionModel == ORTHOGRAPHIC || iProjectionModel == ORTHOGRAPHIC)
      {
        P(2, 2) = 0;
      }

      return P;
    }

    /**
     @brief Method to get the projective matrix of the camera as a \f$3\times 4\f$ matrix.


     In this a zero valued column is appended to \f$P\f$.
     @param[in] iProjectionModel Provide projection model as which \f$\mathrm{K}\f$ is to be returned.
     Default is PERSPECTIVE. <b>NOTE:</b> This will not change the internal projection model of the object.
     */
    cv::Matx34d getP_as3x4(const EProjectionModel &iProjectionModel = PERSPECTIVE) const
    {
      cv::Matx34d P = cv::Matx34d(1, 0, 0, 0,
                                  0, 1, 0, 0,
                                  0, 0, 1, 0);

      //--- adjust K for orthographic projection
      if (mProjectionModel == ORTHOGRAPHIC || iProjectionModel == ORTHOGRAPHIC)
      {
        P(2, 2) = 0;
        P(2, 3) = 1;
      }

      return P;
    }

    /**
     @brief Returns distortion coefficients.

     According to the definition of [OpenCV](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
     this list will hold the fist two radial distortion coefficients (\f$k_1, k_2\f$) in the first
     two places, followd by the two tangential distortion coefficients (\f$p_1, p_2\f$), optinally
     followed by the four remaining radial distortion coefficients (\f$k_3 - k_6\f$) and the and
     the prism distortion coefficients.
     */
    cv::Mat getDistortionCoeffs() const
    {
      return mDistortionCoeffs;
    }

    /**
     @brief Method to set distortion coefficients.

     @param[in] distortionCoeff According to the definition of [OpenCV](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
     this list will hold the fist two radial distortion coefficients (\f$k_1, k_2\f$) in the first
     two places, followd by the two tangential distortion coefficients (\f$p_1, p_2\f$), optinally
     followed by the four remaining radial distortion coefficients (\f$k_3 - k_6\f$) and the and
     the prism distortion coefficients.
     */
    void setDistortionCoeffs(const cv::Mat &distortionCoeff)
    {

      if (!distortionCoeff.empty() && distortionCoeff.size() != cv::Size(1, 4) &&
          distortionCoeff.size() != cv::Size(1, 5) && distortionCoeff.size() != cv::Size(1, 8) &&
          distortionCoeff.size() != cv::Size(1, 12) && distortionCoeff.size() != cv::Size(1, 14))
        throw(lib3d::InvalidArgumentException(FN_NAME, "(!distortionCoeff.empty() && distortionCoeff.size() != cv::Size(1,4) && \
                                              distortionCoeff.size() != cv::Size(1,5) && distortionCoeff.size() != cv::Size(1,8) && \
                                              distortionCoeff.size() != cv::Size(1,12) && distortionCoeff.size() != cv::Size(1,14))"));

      mDistortionCoeffs = distortionCoeff;
    }

    /**
     @brief Returns the radial distortion coefficients as list of doubles.
     */
    std::vector<double> getRadialDistortionCoeffs() const
    {
      std::vector<double> radialDistCoeffs;

      if (!mDistortionCoeffs.empty())
      {
        //--- copy coeffs
        double *dataPtr = reinterpret_cast<double *>(mDistortionCoeffs.data);
        for (int i = 0; i < std::max(mDistortionCoeffs.size[0], mDistortionCoeffs.size[1]); i++)
        {
            if (i < 8)
                radialDistCoeffs.push_back(dataPtr[i]);
        }
      }

      return radialDistCoeffs;
    }

    /**
     @brief Method to set the radial distortion coefficients as list of doubles.

     This will leave the tangential distortion cooefficients unchanged or initialize them to 0.
     */
    void setRadialDistortionCoeffs(const std::vector<double> &radialDistCoeffs)
    {
      if (radialDistCoeffs.size() > 6)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(radialDistCoeffs.size() > 6)"));

      std::vector<double> distCoeffs = radialDistCoeffs;
      if (!mDistortionCoeffs.empty())
      {
        distCoeffs.insert(distCoeffs.begin() + 2, reinterpret_cast<double *>(mDistortionCoeffs.data)[2]);
        distCoeffs.insert(distCoeffs.begin() + 3, reinterpret_cast<double *>(mDistortionCoeffs.data)[3]);
      }
      else
      {
        distCoeffs.insert(distCoeffs.begin() + 2, 0.0);
        distCoeffs.insert(distCoeffs.begin() + 3, 0.0);
      }

      mDistortionCoeffs = cv::Mat(1, distCoeffs.size(), CV_64FC1, 0.0);
      std::memcpy(reinterpret_cast<char *>(mDistortionCoeffs.ptr(0, 0)), distCoeffs.data(),
                  distCoeffs.size() * sizeof(double));
    }

    /**
     @brief Returns the tangential distortion coefficients as list of doubles.
     */
    std::vector<double> getTangentialDistortionCoeffs() const
    {
      std::vector<double> tangDistCoeffs;

      if (!mDistortionCoeffs.empty())
      {
        //--- copy coeffs
        double *dataPtr = reinterpret_cast<double *>(mDistortionCoeffs.data);
        for (int i = 0; i < 4; i++)
        {
          if (i >= 2)
            tangDistCoeffs.push_back(dataPtr[i]);
        }
      }

      return tangDistCoeffs;
    }

    /**
     @brief Method to set the tangential distortion coefficients as list of doubles.

     This will leave the radial distortion cooefficients unchanged or initialize the mandatory ones to 0.
     */
    void setTangentialDistortionCoeffs(const std::vector<double> &tangentialDistCoeffs)
    {
      if (tangentialDistCoeffs.size() != 2)
        throw(lib3d::InvalidArgumentException(FN_NAME, "(tangentialDistCoeffs.size() != 2)"));

      if (mDistortionCoeffs.empty())
        mDistortionCoeffs = cv::Mat(1, 4, CV_64FC1, 0.0);

      mDistortionCoeffs.at<double>(0, 2) = tangentialDistCoeffs[0];
      mDistortionCoeffs.at<double>(0, 3) = tangentialDistCoeffs[1];
    }

    /**
     @brief Method to undistort a given point.

     As described by [OpenCV](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
     the distortion is considered after the projective devision. Thus the coordinates of the points are
     to be given with respect to the center of distortion, which typically is the same as the principle
     point. If not, this class provides a center offset which can be used to shift the center of distortion
     with respect to the principle point.
     @param[in] iX Distorted x-coordinate.
     @param[in] iY Distorted y-coordinate.
     @return Non-distorted coordinates as std::pair with x at first position.
     */
    std::pair<double, double> undistort(const double &iX, const double &iY) const
    {
      cv::Point2d undistortedPnt = undistort(cv::Point2d(iX, iY));
      return std::make_pair(undistortedPnt.x, undistortedPnt.y);
    }

    /**
     @overload

     @param[in] iX Distorted x-coordinate.
     @param[in] iY Distorted y-coordinate.
     @param[out] oX Non-distorted x-coordinate.
     @param[out] oY Non-distorted y-coordinate.
     */
    void undistort(const double &iX, const double &iY, double &oX, double &oY) const
    {
      cv::Point2d undistortedPnt = undistort(cv::Point2d(iX, iY));
      oX = undistortedPnt.x;
      oY = undistortedPnt.y;
    }

    /**
     @overload
     */
    cv::Point2d undistort(const cv::Point2d &iImgPnt) const
    {
      std::vector<cv::Point2d> imgPnts = {iImgPnt};
      return undistort(imgPnts).front();
    }

    /**
     @brief Method to undistort a given list of image points.
     */
    std::vector<cv::Point2d> undistort(const std::vector<cv::Point2d> &iImgPnts) const
    {
      assert(mSkew == 0);

      std::vector<cv::Point2d> undistortedPnts;

      if (mDistortionCoeffs.empty())
        undistortedPnts = iImgPnts;
      else
        cv::undistortPoints(iImgPnts, undistortedPnts, this->getK_as3x3(), mDistortionCoeffs);

      return undistortedPnts;
    }

    /**
     @brief Method to adapt the intrinsics for the given zoom level
     @param[in] intrinsics Original intrinsics object
     @param[in] zoom Zoom level in [0,1]
     @param[in] minF Minimum focal length
     @param[in] maxF Maximum focal length
     @param[out] outCompensatedIntrinsics Adapted intrinsics objects
     */
    static void updateIntrinsicFocalLength(Intrinsics intrinsics, double zoom, double minF, double maxF,
                                           Intrinsics &outCompensatedIntrinsics)
    {
      outCompensatedIntrinsics = intrinsics;

      double minZoom = 0.;
      double maxZoom = 1.;

      if (zoom >= minZoom && zoom <= maxZoom)
      {

        // double m = (maxF-minF) / (maxZoom-minZoom);
        // mActualFocalLength = zoom * m + minF;

        double actualFocalLength = interpolateLinear(zoom, 0., 1., minF, maxF);
        outCompensatedIntrinsics.setFx(outCompensatedIntrinsics.getFx() / minF * actualFocalLength);
        outCompensatedIntrinsics.setFy(outCompensatedIntrinsics.getFy() / minF * actualFocalLength);
      }
    }

    /**
     * @brief Method to create Intrinsics object from field-of-view and image size
     * @param[in] fov Field-of-view in degrees, used for vertical and horizontal fov
     * @param[in] width Image width in pixels
     * @param[in] height Image height in pixels
     * @return Created Intrinsics object
     */
    static Intrinsics createIntrinsicsFromFOV(double fov, int width, int height)
    {
      double fovyhalfrad = degreeToRadian(fov / 2.0);

      double tanfovyhalf = tan(fovyhalfrad);
      double tanfovxhalf = (16.0 / 9.0) * tanfovyhalf;

      double cx = width / 2.0;
      double cy = height / 2.0;
      double fx = cx / tanfovxhalf;
      double fy = cy / tanfovyhalf;

      return Intrinsics(width, height, fx, fy, cx, cy);
    }

    //--- MEMBER DECLERATION ---//

  private:
    /// Projection model of the camera
    EProjectionModel mProjectionModel;

    /// Frame size.
    cv::Size mImageSize;

    /// Focal length or scaling factor of the camera
    cv::Point2d mFocalLength;

    /// Principal point of the camera
    cv::Point2d mPrincipalPnt;

    /// Distortion coefficients of the cameras
    cv::Mat mDistortionCoeffs;

    /// Skew factor of the camera
    double mSkew;
  };

} // namespace lib3d

#endif // LIB3D_INTRINSICS_H
