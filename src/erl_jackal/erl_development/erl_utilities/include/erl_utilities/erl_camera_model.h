#ifndef __ERL_CAMERA_MODEL_H_
#define __ERL_CAMERA_MODEL_H_

#include "erl_utilities/erl_geom_utils.h"

namespace erl {


/*! @brief: Returns the pixel observation of 3-D point y from camera pose (p,R)
 *
 * @Input: <br>
 *    \f$ R = 3 x 3 \f$ = camera orientation in the world FLU frame (x forward, y left, z up) <br>
 *    \f$ p = 3 x 1\f$  = camera position in the world FLU frame <br>
 *    \f$ y = 3 x 1\f$  = observed point position in the world FLU frame <br>
 *    \f$ K = 2 x 3 \f$ = intrinsic calibration <br>
 *
 *   \f$ K = \begin{bmatrix} &f*su &f*st &cu \\
 *           &0 &f*sv &cv \end{bmatrix} \f$
 *
 *   where
 *     f = focal scaling in meters <br>
 *     su = pixels per meter <br>
 *     sv = pixels per meter <br>
 *     cu = image center <br>
 *     cv = image center <br>
 *     st = nonrectangular pixel scaling <br>
 *
 * @Output:
 *   z = 2 x 1 = pixel coordiantes of the point y
 */
inline Eigen::Vector2d cameraModel(const Eigen::Matrix3d &R, const Eigen::Vector3d &p,
                                   const Eigen::Vector3d &y, const Eigen::Matrix<double, 2, 3> &K) {
  //Eigen::Matrix3d oRc; // Camera-to-Optical frame transformation
  //oRc << 0.0, -1.0, 0.0,
  //    0.0, 0.0, -1.0,
  //    1.0, 0.0, 0.0;
  //Eigen::Vector3d y_o = oRc * R.transpose() * (y - p); // point in the camera optical frame
  Eigen::Vector3d y_o = R.transpose() * (y - p); // point in the camera optical frame
  return K * y_o / y_o.z();
}


inline Eigen::Matrix<double, 2, 3> cameraModelJacobianY(const Eigen::Matrix3d &R,
                                                        const Eigen::Vector3d &p,
                                                        const Eigen::Vector3d &y,
                                                        const Eigen::Matrix<double, 2, 3> &K) {
  //Eigen::Matrix3d oRc; // Camera-to-Optical frame transformation
  //oRc << 0.0, -1.0, 0.0,
  //    0.0, 0.0, -1.0,
  //    1.0, 0.0, 0.0;
  //Eigen::Matrix3d oRw = oRc * R.transpose();
  return K * perspectiveJacobian(R.transpose() * (y - p)) * oRw;
}

inline Eigen::Matrix<double, 2, 3> cameraModelJacobianP(const Eigen::Matrix3d &R,
                                                        const Eigen::Vector3d &p,
                                                        const Eigen::Vector3d &y,
                                                        const Eigen::Matrix<double, 2, 3> &K) {
  return -cameraModelJacobianY(R,p,y,K);
}


inline Eigen::Matrix<double, 2, 3> cameraModelJacobianTheta(const Eigen::Matrix3d &R,
                                                            const Eigen::Vector3d &p,
                                                            const Eigen::Vector3d &y,
                                                            const Eigen::Matrix<double, 2, 3> &K) {
  //Eigen::Matrix3d oRc; // Camera-to-Optical frame transformation
  //oRc << 0.0, -1.0, 0.0,
  //    0.0, 0.0, -1.0,
  //    1.0, 0.0, 0.0;
  //Eigen::Matrix3d oRw = oRc * R.transpose();
  Eigen::AngleAxisd ThetaAA(R);
  Eigen::Vector3d Theta = ThetaAA.angle() * ThetaAA.axis();
  return cameraModelJacobianY(R,p,y,K) * axangle2skew(y - p) * SO3RightJacobian(-Theta);
  //return K.block<2, 2>(0, 0) * projectionJacobian(oRw * (y - p)) * oRw * axangle2skew(y - p)
  //    * SO3RightJacobian(-Theta);
}

}
#endif
