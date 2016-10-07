#ifndef CERES_GENERIC_FISHEYE_REPROJECTION_ERROR_H_
#define CERES_GENERIC_FISHEYE_REPROJECTION_ERROR_H_

#include "ceres/rotation.h"
#include "ceres/jet.h"

namespace ceres {
namespace fisheye {

// Templated fisheye camera model for used with Ceres.  The camera is
// parameterized using 12 parameters: 3 for rotation, 3 for translation, 2 for  principal point, 4 for
// the 4-order fisheye f-theta parameter.
struct GenericFisheyeReprojectionError {
  GenericFisheyeReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera_intrinsic,
                  const T* const omega_t,
                  const T* const point,
                  T* residuals) const {
    // omega_t[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(omega_t, point, p);

    // omega_t[3,4,5] are the translation.
    p[0] += omega_t[3];
    p[1] += omega_t[4];
    p[2] += omega_t[5];


    T len = sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
    T theta = acos(p[2] / len);

    //camera_intrinsic[2, 3, 4, 5] are the 4-order coefficent for fisheye
    T radius = T(0.0);
    for (int i = 0; i < 4; i++) {
        radius += camera_intrinsic[2 + i] * pow(theta, 2 * i + 1);
    }

    //camera_intrinsic[0, 1] are the principal point
    T r_xy = sqrt(p[0]*p[0] + p[1]*p[1]);

    T predicted_x = radius * p[0] / r_xy + camera_intrinsic[0];
    T predicted_y = radius * p[1] / r_xy + camera_intrinsic[1];

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<GenericFisheyeReprojectionError, 2, 6, 6, 3>(
                new GenericFisheyeReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};


}  // namespace fisheye
}  // namespace ceres

#endif  // CERES_GENERIC_FISHEYE_REPROJECTION_ERROR_H_
