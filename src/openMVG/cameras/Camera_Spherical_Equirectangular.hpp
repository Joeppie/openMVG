#ifndef OPENMVG_CAMERAS_CAMERA_SPHERICAL_EQUIRECTANGULAR_HPP
#define OPENMVG_CAMERAS_CAMERA_SPHERICAL_EQUIRECTANGULAR_HPP

#include "openMVG/numeric/numeric.h"
#include "openMVG/cameras/Camera_Intrinsics.hpp"

namespace openMVG
{
  namespace cameras
  {

/**
 * @brief Implement a Spherical camera model
 */
    class Intrinsic_Spherical_Equirectangular : public openMVG::cameras::IntrinsicBase
    {

      using class_type = Intrinsic_Spherical_Equirectangular;

    public:

      /**
      * @brief Constructor
      * @param w Width of the image plane
      * @param h Height of the image plane
      */
      Intrinsic_Spherical_Equirectangular
          (
              unsigned int w = 0,
              unsigned int h = 0
          )
          : IntrinsicBase(w, h)
      {
      }

      ~Intrinsic_Spherical_Equirectangular() override = default;

      /**
      * @brief Tell from which type the embed camera is
      * @retval CAMERA_SPHERICAL
      */
      virtual openMVG::cameras::EINTRINSIC getType() const override
      {
        return openMVG::cameras::CAMERA_SPHERICAL_EQUIRECTANGULAR;
      }

      /**
      * @brief Data wrapper for non linear optimization (get data)
      * @return an empty vector of parameter since a spherical camera does not have any intrinsic parameter
      */
      virtual std::vector<double> getParams() const override
      {
        return {};
      }

      /**
      * @brief Data wrapper for non linear optimization (update from data)
      * @param params List of params used to update this intrinsic
      * @retval true if update is correct
      * @retval false if there was an error during update
      */
      virtual bool updateFromParams(const std::vector<double> &params) override
      {
        return true;
      }

      /**
      * @brief Return the list of parameter indexes that must be held constant
      * @param parametrization The given parametrization
      */
      virtual std::vector<int> subsetParameterization(
          const openMVG::cameras::Intrinsic_Parameter_Type &parametrization
      ) const override
      {
        return {};
      }

      /**
      * @brief Transform a point from the camera plane to the image plane
      * @param p Camera plane point
      * @return Point on image plane
      */
      virtual openMVG::Vec2 cam2ima(const openMVG::Vec2 &p) const override
      {
        const size_t size = std::max(h(), w());
        const double pixelSize = (M_PI * 2.0) / static_cast<double>(size);

        double x = (p.x() + M_PI) / pixelSize;
        double y = p.y() / pixelSize;
        return {x, y};
      }

      /**
      * @brief Transform a point from the image plane to the camera plane
      * @param p Image plane point
      * @return camera plane point
      */
      virtual openMVG::Vec2 ima2cam(const openMVG::Vec2 &p) const override
      {
        const size_t size = std::max(h(), w());
        const double pixelSize = (M_PI * 2.0) / static_cast<double>(size);
        return {
            (p.x()) * pixelSize - M_PI,
            (p.y()) * pixelSize
        };
      }

      /**
      * @brief Get bearing vector of a point given an image coordinate
      * @return bearing vector
      */
      virtual openMVG::Vec3 operator()(const openMVG::Vec2 &p) const override
      {
        const openMVG::Vec2 hv = ima2cam(p);

        double h = hv[0];
        double v = hv[1];


        return {
            sin(h) * sin(v),
            cos(h) * sin(v),
            cos(v)
        };
      }

      virtual Vec2 residual(
          const geometry::Pose3 &pose,
          const Vec3 &X,
          const Vec2 &x) const
      {
        const Vec2 proj = this->project(pose, X);
        Vec2 residual = x - proj;

        //Same Wrap function in javascript; function(a,b,width) { dx = a-b; sign=Math.min(1, Math.max(-1, dx)); if(Math.abs(dx) > width/2) { dx = -(width-(dx*sign))*sign; return dx } return dx;  }

        double& dx = (residual.x()); //grab residual x, and fix if necessary.
        if (abs(dx) > w() / 2) //Since this is a cylindrical projection, if delta more than half the circle (expressed in pixels), invert result.
        {
          double sign = std::min(1.0, std::max(-1.0, dx)); //1 or -1, for respectively positive and negative dx.
          dx = -(w() - (dx * sign)) * sign;
        }

        //Equivalent to x-proj for non-spherical cameras, or spherical when dx smaller than half the width/circle.
        return residual;
      }

      /**
      * @brief Compute projection of a 3D point into the image plane
      * (Apply pose, disto (if any) and Intrinsics)
      * @param pose Pose used to compute projection
      * @param pt3D 3D-point to project on image plane
      * @return Projected (2D) point on image plane
      */
      openMVG::Vec2 project(
          const openMVG::geometry::Pose3 &pose,
          const openMVG::Vec3 &pt3D) const override
      {
        const openMVG::Vec3 X = pose(pt3D); // apply pose

        double d = sqrt(X.x() * X.x() + X.y() * X.y() + X.z() * X.z());

        openMVG::Vec3 unitVector = X / d;

        double h = atan2(unitVector.x(), unitVector.y());
        double v = acos(unitVector.z());

        // denormalization (angle to pixel value)
        return cam2ima({h, v});
      }

      /**
      * @brief Does the camera model handle a distortion field?
      * @retval false
      */
      virtual bool have_disto() const override
      { return false; }

      /**
      * @brief Add the distortion field to a point (that is in normalized camera frame)
      * @param p Point before distortion computation (in normalized camera frame)
      * @return the initial point p (spherical camera does not have distortion field)
      */
      virtual openMVG::Vec2 add_disto(const openMVG::Vec2 &p) const override
      { return p; }

      /**
      * @brief Remove the distortion to a camera point (that is in normalized camera frame)
      * @param p Point with distortion
      * @return the initial point p (spherical camera does not have distortion field)
      */
      virtual openMVG::Vec2 remove_disto(const openMVG::Vec2 &p) const override
      { return p; }

      /**
      * @brief Return the un-distorted pixel (with removed distortion)
      * @param p Input distorted pixel
      * @return Point without distortion
      */
      virtual openMVG::Vec2 get_ud_pixel(const openMVG::Vec2 &p) const override
      { return p; }

      /**
      * @brief Return the distorted pixel (with added distortion)
      * @param p Input pixel
      * @return Distorted pixel
      */
      virtual openMVG::Vec2 get_d_pixel(const openMVG::Vec2 &p) const override
      { return p; }

      /**
      * @brief Normalize a given unit pixel error to the camera plane
      * @param value Error in image plane
      * @return error of passing from the image plane to the camera plane
      */
      virtual double imagePlane_toCameraPlaneError(double value) const override
      { return value; }

      /**
      * @brief Return the projection matrix (interior & exterior) as a simplified projective projection
      * @param pose Extrinsic matrix
      * @return Concatenation of intrinsic matrix and extrinsic matrix
      */
      virtual openMVG::Mat34 get_projective_equivalent(const openMVG::geometry::Pose3 &pose) const override
      {
        return openMVG::HStack(pose.rotation(), pose.translation());
      }

      /**
      * @brief Serialization out
      * @param ar Archive
      */
      template<class Archive>
      inline void save(Archive &ar) const;

      /**
      * @brief  Serialization in
      * @param ar Archive
      */
      template<class Archive>
      inline void load(Archive &ar);

      /**
      * @brief Clone the object
      * @return A clone (copy of the stored object)
      */
      IntrinsicBase *clone(void) const override
      {
        return new class_type(*this);
      }

    };

  }
}

#endif // #ifndef OPENMVG_CAMERAS_CAMERA_SPHERICAL_EQUIRECTANGULAR_HPP
