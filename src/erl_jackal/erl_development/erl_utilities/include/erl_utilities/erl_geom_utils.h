#ifndef __ERL_GEOM_UTILS_H_
#define __ERL_GEOM_UTILS_H_

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/optional.hpp>

namespace erl {

/**
 * @file erl_geom_utils.h
 * @brief Geometry utility functions. Contains camera geometry tools, and rotations and transformation utilities common
 * in robotics applications.
 */

template<typename T>
inline Eigen::Matrix<T, 3, 1> perspectiveProjection(const Eigen::Matrix<T, 3, 1> &v) { return v/v.z(); }
template<typename T>
inline Eigen::Matrix<T, 4, 1> perspectiveProjection(const Eigen::Matrix<T, 4, 1> &v) { return v/v.z(); }

template<typename T>
inline Eigen::Matrix<T, 3, 3> perspectiveJacobian(const Eigen::Matrix<T, 3, 1> &v) {
  Eigen::Matrix<T, 3, 3> J;
  J << 1/v.z(),       0, -v.x() / v.z() / v.z(),
             0, 1/v.z(), -v.y() / v.z() / v.z(),
             0,       0,                      0;
  return J;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> perspectiveJacobian(const Eigen::Matrix<T, 4, 1> &v) {
  Eigen::Matrix<T, 4, 4> J;
  J << 1/v.z(),       0, -v.x() / v.z() / v.z(),       0,
             0, 1/v.z(), -v.y() / v.z() / v.z(),       0,
             0,       0,                      0,       0,
             0,       0, -v.w() / v.z() / v.z(), 1/v.z();
  return J;
}

inline Eigen::Matrix4d flu2optical() {
  Eigen::Matrix4d T;
  T << 0,-1, 0, 0,
       0, 0,-1, 0,
       1, 0, 0, 0,
       0, 0, 0, 1;
  return T;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> axangle2skew( const Eigen::Matrix<T, 3, 1> &x ){
  Eigen::Matrix<T, 3, 3> S;
  S <<   0,-x(2), x(1),
      x(2),   0, -x(0),
     -x(1), x(0),    0;
  return S;
}

template<typename T>
inline Eigen::Matrix<T, 3, 1> skew2axangle( const Eigen::Matrix<T, 3, 3>& S ){
  Eigen::Matrix<T, 3, 1> x;
  x << S(2,1), S(0,2), S(1,0);
  return x;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> axangle2rot( const Eigen::Matrix<T, 3, 1> &a ){
  
  typename Eigen::Matrix<T, 3, 1>::Scalar na = a.norm();
  auto ca = std::cos(na), sa = std::sin(na);
  Eigen::Matrix<T, 3, 1> ana, mc_ana, sa_ana;
  if( na > 0 ){ ana = a/na; mc_ana = ana * (1-ca); sa_ana = ana * sa; }
  else{ ana = a; mc_ana = a; sa_ana = a; }
  
  Eigen::Matrix<T, 3, 3> R;
  R << mc_ana(0)*ana(0) + ca,        mc_ana(0)*ana(1) - sa_ana(2), mc_ana(0)*ana(2) + sa_ana(1),
       mc_ana(0)*ana(1) + sa_ana(2), mc_ana(1)*ana(1) + ca,        mc_ana(2)*ana(1) - sa_ana(0),
       mc_ana(0)*ana(2) - sa_ana(1), mc_ana(1)*ana(2) + sa_ana(0), mc_ana(2)*ana(2) + ca;
  return R;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> skew2rot( const Eigen::Matrix<T, 3, 3>& S ){
  return axangle2rot(skew2axangle(S));
}

template<typename T>
inline Eigen::Matrix<T, 9, 3> axangle2skewJacobian() {
  Eigen::Matrix<T, 9, 3> J;
  J << 0, 0, 0,
       0, 0, 1,
       0,-1, 0,
       0, 0,-1,
       0, 0, 0,
       1, 0, 0,
       0, 1, 0,
      -1, 0, 0,
       0, 0, 0;
  return J;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> SO3RightJacobian(const Eigen::Matrix<T, 3, 1> &theta) {
  auto theta_norm = theta.norm();
  auto theta_norm2 = theta_norm * theta_norm;
  auto theta_norm3 = theta_norm * theta_norm2;
  Eigen::Matrix<T, 3, 3> theta_hat = axangle2skew(theta);
  if( theta_norm > 0 ){
    return Eigen::Matrix<T, 3, 3>::Identity() - (1.0 - std::cos(theta_norm)) / theta_norm2 * theta_hat
      + (theta_norm - std::sin(theta_norm)) / theta_norm3 * theta_hat * theta_hat;
  }else
    return Eigen::Matrix<T, 3, 3>::Identity();
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> SO3RightJacobianInverse(const Eigen::Matrix<T, 3, 1> &theta) {
  auto theta_norm = theta.norm();
  auto theta_norm2 = theta_norm * theta_norm;
  Eigen::Matrix<T, 3, 3> theta_hat = axangle2skew(theta);
  return Eigen::Matrix<T, 3, 3>::Identity() + theta_hat / 2.0
      + (1.0 / theta_norm2 - (1.0 + std::cos(theta_norm)) / 2.0 / theta_norm / std::sin(theta_norm)) * theta_hat
          * theta_hat;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> SO3LeftJacobian(const Eigen::Matrix<T, 3, 1> &theta) {
  auto theta_norm = theta.norm();
  auto theta_norm2 = theta_norm * theta_norm;
  auto theta_norm3 = theta_norm * theta_norm2;
  Eigen::Matrix<T, 3, 3> theta_hat = axangle2skew(theta);
  if( theta_norm > 0 ){
    return Eigen::Matrix<T, 3, 3>::Identity() + (1.0 - std::cos(theta_norm)) / theta_norm2 * theta_hat
        + (theta_norm - std::sin(theta_norm)) / theta_norm3 * theta_hat * theta_hat;
  }else
    return Eigen::Matrix<T, 3, 3>::Identity();
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> SO3LeftJacobianInverse(const Eigen::Matrix<T, 3, 1> &theta) {
  auto theta_norm = theta.norm();
  auto theta_norm2 = theta_norm * theta_norm;
  Eigen::Matrix<T, 3, 3> theta_hat = axangle2skew(theta);
  return Eigen::Matrix3d::Identity() - theta_hat / 2.0
      + (1.0 / theta_norm2 - (1.0 + std::cos(theta_norm)) / 2.0 / theta_norm / std::sin(theta_norm)) * theta_hat
          * theta_hat;
}

inline double deg2rad(double deg) { return (M_PI / 180) * deg; }
inline double rad2deg(double rad) { return (180 / M_PI) * rad; }

template<typename T>
inline Eigen::Matrix<T, 3, 1> sph2cart(const Eigen::Matrix<T, 3, 1> &az_elev_rad) {
  Eigen::Matrix<T, 3, 1> x_y_z;
  x_y_z.x() = az_elev_rad.z() * std::cos(az_elev_rad.y()) * std::cos(az_elev_rad.x());
  x_y_z.y() = az_elev_rad.z() * std::cos(az_elev_rad.y()) * std::sin(az_elev_rad.x());
  x_y_z.z() = az_elev_rad.z() * std::sin(az_elev_rad.y());
  return x_y_z;
}

template<typename T>
inline Eigen::Matrix<T, 3, 1> cart2sph(Eigen::Matrix<T, 3, 1> const &x_y_z) {
  Eigen::Matrix<T, 3, 1> az_elev_rad;
  az_elev_rad.x() = std::atan2(x_y_z.y(), x_y_z.x());
  az_elev_rad.y() = std::atan2(x_y_z.z(), std::hypot(x_y_z.x(), x_y_z.y()));
  az_elev_rad.z() = std::hypot(std::hypot(x_y_z.x(), x_y_z.y()), x_y_z.z());
  return az_elev_rad;
}

inline double restrictAngle(double phi, double minrange = -M_PI, double maxrange = M_PI) {
  // return minrange + std::fmod( (phi - minrange), (maxrange - minrange));
  // NOTE!!: fmod does not behave like MATLAB mod!
  double x = phi - minrange;
  double y = maxrange - minrange;
  return minrange + x - y * std::floor(x / y);
}

// returns the angular distance between two angles a and b in RAD
inline double angularDistance(double phi, double psi) { return std::abs(restrictAngle(phi-psi)); }

inline double angularMean(const Eigen::VectorXd& a, const Eigen::VectorXd& w) {
  return std::atan2( w.dot(a.unaryExpr<double(*)(double)>(&std::sin)), 
                     w.dot(a.unaryExpr<double(*)(double)>(&std::cos)) );
}

inline void smart_plus_SE2(double x1, double y1, double q1,
                           double x2, double y2, double q2,
                           double &x, double &y, double &q) {
  double sq1 = std::sin(q1);
  double cq1 = std::cos(q1);
  x = x1 + cq1 * x2 - sq1 * y2;
  y = y1 + sq1 * x2 + cq1 * y2;
  q = restrictAngle(q1 + q2);
}



template<typename T>
inline Eigen::Matrix<T, 3, 3> rotx(T roll) {
  T cr = std::cos(roll), sr = std::sin(roll);
  Eigen::Matrix<T, 3, 3> R;
  R << 1,  0,   0,
       0, cr, -sr,
       0, sr,  cr;
  return R;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> roty(T pitch) {
  T cp = std::cos(pitch), sp = std::sin(pitch);
  Eigen::Matrix<T, 3, 3> R;
  R << cp, 0, sp,
        0, 1,  0,
      -sp, 0, cp;
  return R;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> rotz(T yaw) {
  T cy = std::cos(yaw), sy = std::sin(yaw);
  Eigen::Matrix<T, 3, 3> R;
  R << cy,-sy, 0,
       sy, cy, 0,
        0,  0, 1;
  return R;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> rotxh(T roll) {
  T cr = std::cos(roll), sr = std::sin(roll);
  Eigen::Matrix<T, 4, 4> P;
  P << 1,  0,  0, 0,
       0, cr,-sr, 0,
       0, sr, cr, 0,
       0,  0,  0, 1;
  return P;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> rotyh(T pitch) {
  T cp = std::cos(pitch), sp = std::sin(pitch);
  Eigen::Matrix<T, 4, 4> P;
  P << cp, 0, sp, 0,
        0, 1,  0, 0,
      -sp, 0, cp, 0,
       0,  0,  0, 1;
  return P;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> rotzh(T yaw) {
  T cy = std::cos(yaw), sy = std::sin(yaw);
  Eigen::Matrix<T, 4, 4> P;
  P << cy,-sy, 0, 0,
       sy, cy, 0, 0,
       0,   0, 1, 0,
       0,   0, 0, 1;
  return P;
}


template<typename T>
inline Eigen::Matrix<T, 2, 2> rot2d(T yaw) {
  T cy = std::cos(yaw), sy = std::sin(yaw);
  Eigen::Matrix<T, 2, 2> R;
  R << cy,-sy, sy, cy;
  return R;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> rot3d(T roll, T pitch, T yaw, std::string axes = "rzyx")
{  
  typename Eigen::Matrix<T, 3, 3>::Index i, j, k;
  bool parity, repetition, frame;
  
  if(axes.compare("sxyz") == 0){ i = 0; parity = false; repetition = false; frame = false; }
  if(axes.compare("sxyx") == 0){ i = 0; parity = false; repetition = true;  frame = false; }
  if(axes.compare("sxzy") == 0){ i = 0; parity = true;  repetition = false; frame = false; }
  
  if(axes.compare("sxzx") == 0){ i = 0; parity = true;  repetition = true;  frame = false; }
  if(axes.compare("syzx") == 0){ i = 1; parity = false; repetition = false; frame = false; }
  if(axes.compare("syzy") == 0){ i = 1; parity = false; repetition = true;  frame = false; }
  
  if(axes.compare("syxz") == 0){ i = 1; parity = true;  repetition = false; frame = false; }
  if(axes.compare("syxy") == 0){ i = 1; parity = true;  repetition = true;  frame = false; }
  if(axes.compare("szxy") == 0){ i = 2; parity = false; repetition = false; frame = false; }
  
  if(axes.compare("szxz") == 0){ i = 2; parity = false; repetition = true;  frame = false; }
  if(axes.compare("szyx") == 0){ i = 2; parity = true;  repetition = false; frame = false; }
  if(axes.compare("szyz") == 0){ i = 2; parity = true;  repetition = true;  frame = false; }
  
  if(axes.compare("rzyx") == 0){ i = 0; parity = false; repetition = false; frame = true; }
  if(axes.compare("rxyx") == 0){ i = 0; parity = false; repetition = true;  frame = true; }
  if(axes.compare("ryzx") == 0){ i = 0; parity = true;  repetition = false; frame = true; }
  
  if(axes.compare("rxzx") == 0){ i = 0; parity = true;  repetition = true;  frame = true; }
  if(axes.compare("rxzy") == 0){ i = 1; parity = false; repetition = false; frame = true; }
  if(axes.compare("ryzy") == 0){ i = 1; parity = false; repetition = true;  frame = true; }
  
  if(axes.compare("rzxy") == 0){ i = 1; parity = true;  repetition = false; frame = true; }
  if(axes.compare("ryxy") == 0){ i = 1; parity = true;  repetition = true;  frame = true; }
  if(axes.compare("ryxz") == 0){ i = 2; parity = false; repetition = false; frame = true; }
  
  if(axes.compare("rzxz") == 0){ i = 2; parity = false; repetition = true;  frame = true; }
  if(axes.compare("rxyz") == 0){ i = 2; parity = true;  repetition = false; frame = true; }
  if(axes.compare("rzyz") == 0){ i = 2; parity = true;  repetition = true;  frame = true; }
  
  switch(i+parity)
  {
    case 0: j = 1; break;
    case 1: j = 2; break;
    case 2: j = 0; break;
    case 3: j = 1; break;
  }
  switch(i-parity+1)
  {
    case 0: k = 1; break;
    case 1: k = 2; break;
    case 2: k = 0; break;
    case 3: k = 1; break;
  }

  if( frame ){ T tmp = yaw; yaw = roll; roll = tmp; } 
  if( parity ){ roll = -roll; pitch = -pitch; yaw = -yaw; }
  
  T sr = std::sin(roll), sp = std::sin(pitch), sy = std::sin(yaw);
  T cr = std::cos(roll), cp = std::cos(pitch), cy = std::cos(yaw);
  T cycr = cy*cr, cysr = cy*sr;
  T sycr = sy*cr, sysr = sy*sr;  
  Eigen::Matrix<T, 3, 3> R;
  if( repetition ){
    R(i,i) = cp,    R(i,j) = sp*sy,          R(i,k) = sp*cy;
    R(j,i) = sp*sr, R(j,j) =-cp*sysr + cycr, R(j,k) =-cp*cysr - sycr;
    R(k,i) =-sp*cr, R(k,j) = cp*sycr + cysr, R(k,k) = cp*cycr - sysr;
  }else{
    R(i,i) = cp*cr, R(i,j) = sp*sycr-cysr, R(i,k) = sp*cycr+sysr;
    R(j,i) = cp*sr, R(j,j) = sp*sysr+cycr, R(j,k) = sp*cysr-sycr;
    R(k,i) =-sp,    R(k,j) = cp*sy,        R(k,k) = cp*cy;    
  }
  return R;
}


/**
 * Returns the rotation matrix R minimizing the error between \f$ S1 - R * S2 \f$.
 * @param S1 A 3xN matrix of corresponding points.
 * @param S2 A 3xN matrix of corresponding points.
 * @return The result of \f$ R = \mathop{argmin}_{R \in SO(3)} S1 - R * S2 \f$.
 */
inline Eigen::Matrix3d findRotation(const Eigen::MatrixXd &S1, const Eigen::MatrixXd &S2) {
  Eigen::Matrix3d M = S1 * S2.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d R1 = svd.matrixU() * svd.matrixV().transpose();

  Eigen::Vector3d reflect;
  reflect << 1, 1, R1.determinant();

  return svd.matrixU() * reflect.asDiagonal() * svd.matrixV().transpose();
}

/**
 *  Aligns the points S2 (3xN) to S1 (3xN). If w is provided as an output, finds a full similarity transform
 *  including scaling. If no w is provided, computes only rotation and translation.
 * @param S1 A matrix of points (3xN)
 * @param S2 A matrix of points (3xN)
 * @param R The rotation output.
 * @param T The translation output.
 * @param w The scaling.
 */
inline void findSimilarityTransform(const Eigen::MatrixXd &S1,
                             const Eigen::MatrixXd &S2,
                             Eigen::Matrix3d &R,
                             Eigen::Vector3d &T,
                             boost::optional<double &> w = boost::none) {
  Eigen::Vector3d T1 = S1.rowwise().mean();
  Eigen::MatrixXd S1_sub = S1.colwise() - T1;

  Eigen::Vector3d T2 = S2.rowwise().mean();
  Eigen::MatrixXd S2_sub = S2.colwise() - T2;

  R = findRotation(S1_sub, S2_sub);

  S2_sub = R * S2_sub;

  if (w) {
    *w = (S1_sub.transpose() * S2_sub).trace() / (S2_sub.transpose() * S2_sub).trace();
    T = T1 - (*w) * R * T2;
  } else {
    T = T1 - R * T2;
  }
}

/**
 * Converts a quaternion to rotation matrix. Assumes the Scalar is the first component.
 * @tparam T The type of the quaternion input.
 * @param q The input quaternion.
 * @return The rotation matrix output.
 */
template<typename T>
inline Eigen::Matrix<T, 3, 3> quat2rot(const Eigen::Matrix<T, 4, 1> &q) {
  Eigen::Matrix<T, 3, 3> R;
  R << 1.0 - 2.0 * (q(2) * q(2) + q(3) * q(3)),
      2.0 * q(1) * q(2) - 2.0 * q(0) * q(3),
      2.0 * q(0) * q(2) + 2.0 * q(1) * q(3),
      2.0 * q(1) * q(2) + 2.0 * q(0) * q(3),
      1.0 - 2.0 * (q(1) * q(1) + q(3) * q(3)),
      2.0 * q(2) * q(3) - 2.0 * q(0) * q(1),
      2.0 * q(1) * q(3) - 2.0 * q(0) * q(2),
      2.0 * q(2) * q(3) + 2.0 * q(0) * q(1),
      1.0 - 2.0 * (q(1) * q(1) + q(2) * q(2));
  return R;
}

/**
 * Converts a quaternion to rotation matrix. Assumes the Scalar is the first component.
 * @tparam T Type of the quaternion input.
 * @param qw The scalar quaternion component.
 * @param qx The x component of the quaternion vector.
 * @param qy The y component of the quaternion vector.
 * @param qz The z component of the quaternion vector.
 * @return The output rotation matrix.
 */
template<typename T>
inline Eigen::Matrix<T, 3, 3> quat2rot(T qw, T qx, T qy, T qz) {
  Eigen::Matrix<T, 3, 3> R;
  R << 1.0 - 2.0 * (qy * qy + qz * qz),
      2.0 * qx * qy - 2.0 * qw * qz,
      2.0 * qw * qy + 2.0 * qx * qz,
      2.0 * qx * qy + 2.0 * qw * qz,
      1.0 - 2.0 * (qx * qx + qz * qz),
      2.0 * qy * qz - 2.0 * qw * qx,
      2.0 * qx * qz - 2.0 * qw * qy,
      2.0 * qy * qz + 2.0 * qw * qx,
      1.0 - 2.0 * (qx * qx + qy * qy);
  return R;
}

/**
 * Converts a rotation matrix to quaternion, with scalar part first.
 * @tparam T The type of the rotation matrix.
 * @param R The input rotation matrix.
 * @return The quaternion output.
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> rot2quat(const Eigen::Matrix<T, 3, 3> &R) {
  Eigen::Quaternion<T> q(R);
  return Eigen::Matrix<T, 4, 1>(q.w(), q.x(), q.y(), q.z());
}

template<typename T>
inline Eigen::Matrix<T, 4, 1> axangle2tangentquat( const Eigen::Matrix<T, 3, 1> &a ){
  Eigen::Matrix<T, 4, 1> q;
  q << 0, a.x()/2, a.y()/2, a.z()/2;
  return q;
}




template<typename T>
inline Eigen::Matrix<T, 4, 4> axangle2twist( const Eigen::Matrix<T, 6, 1> &a ){
  Eigen::Matrix<T, 4, 4> V;
  V <<     0,-a(5), a(4), a(0),
        a(5),    0,-a(3), a(1),
       -a(4), a(3),    0, a(2),
           0,    0,    0,    0;   
  return V;
}

template<typename T>
inline Eigen::Matrix<T, 6, 1> twist2axangle( const Eigen::Matrix<T, 4, 4> &V ){
  Eigen::Matrix<T, 6, 1> a;
  a << V(0,3), V(1,3), V(2,3), V(2,1), V(0,2), V(1,0);
  return a;
}

template<typename T>
inline Eigen::Matrix<T, 6, 6> axangle2adtwist( const Eigen::Matrix<T, 6, 1> &a ){
  Eigen::Matrix<T, 6, 6> A;
  A << 0,   -a(5), a(4), 0,   -a(2), a(1),
       a(5), 0,   -a(3), a(2), 0,   -a(0),
      -a(4), a(3), 0,   -a(1), a(0), 0,
       0,    0,    0,    0,   -a(5), a(4),
       0,    0,    0,    a(5), 0,   -a(3),
       0,    0,    0,   -a(4), a(3), 0;
  return A;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> twist2pose( const Eigen::Matrix<T, 4, 4> &V ) {
  auto rotang2 = V(2,1)*V(2,1) + V(0,2)*V(0,2) + V(1,0)*V(1,0);
  auto rotang  = std::sqrt(rotang2);
  auto rotang3 = rotang * rotang2;
  auto V2 = V*V;
  auto V3 = V*V2;
  
  Eigen::Matrix<T, 4, 4> P;
  if( rotang2 > 0 )
    P = Eigen::Matrix<T, 4, 4>::Identity() + V + (1-std::cos(rotang))/rotang2*V2 + (rotang - std::sin(rotang))/rotang3*V3;
  else
    P = Eigen::Matrix<T, 4, 4>::Identity() + V;  
  return P;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> axangle2pose( const Eigen::Matrix<T, 6, 1> &a )
{ return twist2pose(axangle2twist(a)); }

template<typename T>
inline Eigen::Matrix<T, 6, 6> pose2adpose( const Eigen::Matrix<T, 4, 4> &P ) {
  Eigen::Matrix<T, 6, 6> A;
  A << P.block<3,3>(0,0), axangle2skew(P.block<3,1>(0,3)) * P.block<3,3>(0,0),
       Eigen::Matrix<T,3,3>::Zero(), P.block<3,3>(0,0);
  return A;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> inversePose( const Eigen::Matrix<T, 4, 4>& P ) { 
  Eigen::Matrix<T, 4, 4> iP;
  iP << P(0,0), P(1,0), P(2,0), -P(0,0)*P(0,3) -P(1,0)*P(1,3) -P(2,0)*P(2,3),
        P(0,1), P(1,1), P(2,1), -P(0,1)*P(0,3) -P(1,1)*P(1,3) -P(2,1)*P(2,3),
        P(0,2), P(1,2), P(2,2), -P(0,2)*P(0,3) -P(1,2)*P(1,3) -P(2,2)*P(2,3),
        0,      0,      0,       1;
  return iP;
}

template<typename T>
inline Eigen::Matrix<T, 4, 6> odot( const Eigen::Matrix<T, 4, 1>& p ) {
  Eigen::Matrix<T, 4, 6> A;
  A << p(3)*Eigen::Matrix<T, 3, 3>::Identity(), -axangle2skew(p.template head<3>()),
       Eigen::Matrix<T, 1, 6>::Zero();
  return A;
}

template<typename T>
inline Eigen::Matrix<T, 6, 4> circledCirc( const Eigen::Matrix<T, 4, 1>& p ) {
  Eigen::Matrix<T, 6, 4> A;
  A << Eigen::Matrix<T, 3, 3>::Zero(),      p.template head<3>(),
       -axangle2skew(p.template head<3>()), Eigen::Matrix<T, 3, 1>::Zero();
  return A;
}





// XXX: ASSUMES THE SCALAR IS THE LAST COMPONENT!
/**
 * Converts a quaternion to rotation matrix, with JPL format (scalar last).
 * @tparam T The type of the quaternion.
 * @param q The input quaternion.
 * @return The output rotation matrix.
 */
template<typename T>
inline Eigen::Matrix<T, 3, 3> quat2rotJPL(const Eigen::Matrix<T, 4, 1> &q) {
  Eigen::Matrix<T, 3, 3> R;
  R << q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3),
      2.0 * (q(0) * q(1) + q(2) * q(3)),
      2.0 * (q(0) * q(2) - q(1) * q(3)),
      2.0 * (q(0) * q(1) - q(2) * q(3)),
      -q(0) * q(0) + q(1) * q(1) - q(2) * q(2) + q(3) * q(3),
      2.0 * (q(1) * q(2) + q(0) * q(3)),
      2.0 * (q(0) * q(2) + q(1) * q(3)),
      2.0 * (q(1) * q(2) - q(0) * q(3)),
      -q(0) * q(0) - q(1) * q(1) + q(2) * q(2) + q(3) * q(3);
  return R;
}

// XXX: ASSUMES THE SCALAR IS THE LAST COMPONENT!
/**
 * Converts a rotation matrix to a quaternion in JPL format (scalar last).
 * @tparam T The type of the rotation matrix.
 * @param R The input rotation matrix.
 * @return The output quaternion in JPL format.
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> rot2quatJPL(const Eigen::Matrix<T, 3, 3> &R) {
  T trR = R.trace();
  Eigen::Matrix<T, 4, 1> score;
  score(0) = R(0, 0);
  score(1) = R(1, 1);
  score(2) = R(2, 2);
  score(3) = trR;

  typename Eigen::Matrix<T, 4, 1>::Index maxIndex;
  score.maxCoeff(&maxIndex);

  Eigen::Matrix<T, 4, 1> q;
  if (maxIndex == 0) {
    q(0) = std::sqrt(1 + 2 * R(0, 0) - trR) / 2.0;
    q(1) = (R(0, 1) + R(1, 0)) / (4 * q(0));
    q(2) = (R(0, 2) + R(2, 0)) / (4 * q(0));
    q(3) = (R(1, 2) - R(2, 1)) / (4 * q(0));
  } else if (maxIndex == 1) {
    q(1) = std::sqrt(1 + 2 * R(1, 1) - trR) / 2.0;
    q(0) = (R(0, 1) + R(1, 0)) / (4 * q(1));
    q(2) = (R(1, 2) + R(2, 1)) / (4 * q(1));
    q(3) = (R(2, 0) - R(0, 2)) / (4 * q(1));
  } else if (maxIndex == 2) {
    q(2) = std::sqrt(1 + 2 * R(2, 2) - trR) / 2.0;
    q(0) = (R(0, 2) + R(2, 0)) / (4 * q(2));
    q(1) = (R(1, 2) + R(2, 1)) / (4 * q(2));
    q(3) = (R(0, 1) - R(1, 0)) / (4 * q(2));
  } else {
    q(3) = std::sqrt(1 + trR) / 2.0;
    q(0) = (R(1, 2) - R(2, 1)) / (4 * q(3));
    q(1) = (R(2, 0) - R(0, 2)) / (4 * q(3));
    q(2) = (R(0, 1) - R(1, 0)) / (4 * q(3));
  }

  if (q(3) < 0) q = -q;
  return q / q.norm();
}

/**
 * TODO: Document this.
 * @tparam T
 * @param q
 * @return
 */
template<typename T>
inline Eigen::Matrix<T, 4, 4> quatLeftMatrix(const Eigen::Matrix<T, 4, 1> &q) {
  Eigen::Matrix<T, 4, 4> L;
  L << q(0), -q(1), -q(2), -q(3),
      q(1), q(0), -q(3), q(2),
      q(2), q(3), q(0), -q(1),
      q(3), -q(2), q(1), q(0);
  return L;
}


/**
 * TODO: Document this.
 * @tparam T
 * @param q
 * @return
 */
template<typename T>
inline Eigen::Matrix<T, 4, 4> quatRightMatrix(const Eigen::Matrix<T, 4, 1> &q) {
  Eigen::Matrix<T, 4, 4> R;
  R << q(0), -q(1), -q(2), -q(3),
      q(1), q(0), q(3), -q(2),
      q(2), -q(3), q(0), q(1),
      q(3), q(2), -q(1), q(0);
  return R;
}

// XXX: ASSUMES THE SCALAR IS THE LAST COMPONENT!

/**
 * TODO: Document this.
 * @tparam T
 * @param q
 * @return
 */
template<typename T>
inline Eigen::Matrix<T, 4, 4> quatLeftMatrixJPL(const Eigen::Matrix<T, 4, 1> &q) {
  Eigen::Matrix<T, 4, 4> L;
  L << q(0), q(3), -q(2), q(1),
      -q(3), q(0), q(1), q(2),
      q(2), -q(1), q(0), q(3),
      -q(1), -q(2), -q(3), q(0);
  return L;
}
// XXX: ASSUMES THE SCALAR IS THE LAST COMPONENT!

/**
 * TODO: Document this.
 * @tparam T
 * @param q
 * @return
 */
template<typename T>
inline Eigen::Matrix<T, 4, 4> quatRightMatrixJPL(const Eigen::Matrix<T, 4, 1> &q) {
  Eigen::Matrix<T, 4, 4> R;
  R << q(0), -q(3), q(2), q(1),
      q(3), q(0), -q(1), q(2),
      -q(2), q(1), q(0), q(3),
      -q(1), -q(2), -q(3), q(0);
  return R;
}


/**
 * Computes an SO(3) Metric on two quaternions, q1, q2..
 * @param q1 The first quaternion input.
 * @param q2 The second quaternion input.
 * @return The metric between quaternions.
 */
inline double SO3Metric(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2) {
  double quat_inner_prod = q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z() + q1.w() * q2.w();
  return std::acos(2 * quat_inner_prod * quat_inner_prod - 1);
}

/**
 * Computes the quaternion norm of an input quaternion.
 * @tparam T The data type of the input quaternion.
 * @param quat The input quaternion.
 * @return The quaternion norm.
 */
template<typename T>
inline T quatNorm(const Eigen::Matrix<T, 4, 1> &quat) {
  return quat.norm();
}

/**
 * Computes the normalized quaternion of an input quaternion.
 * @tparam T The data type of the input quaternion.
 * @param quat The quaternion input.
 * @return The normalized quaternion output.
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> quatNormalize(const Eigen::Matrix<T, 4, 1> &quat) {
  return quat / quat.norm();
}

/**
 * Computes the quaternion exponential of a quaternion q.
 * @tparam T The data type of the input quaternion.
 * @param q The quaternion input.
 * @return The quaternion exponential output.
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> quatExp(const Eigen::Matrix<T, 4, 1> &q) {
  T v_norm = q.template tail<3>().norm();
  if (v_norm > 0)
    return std::exp(q(0)) * Eigen::Matrix<T, 4, 1>(std::cos(v_norm), q(1) / v_norm * std::sin(v_norm),
                                                   q(2) / v_norm * std::sin(v_norm), q(3) / v_norm * std::sin(v_norm));
  else
    return Eigen::Matrix<T, 4, 1>(std::exp(q(0)), 0.0, 0.0, 0.0);
}

/**
 * Computes the quaternion log of a quaternion q.
 * @tparam T The data type of the input quaternion.
 * @param q The input quaternion.
 * @return The quaternion logarithm output.
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> quatLog(const Eigen::Matrix<T, 4, 1> &q) {
  T tmp = q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
  T v_norm = std::sqrt(tmp);
  T q_norm = std::sqrt(q.w() * q.w() + tmp);
  if (v_norm > 0) {
    T acos_s_over_norm = std::acos(q.w() / q_norm);
    return Eigen::Matrix<T, 4, 1>(std::log(q_norm), q.x() / v_norm * acos_s_over_norm,
                                  q.y() / v_norm * acos_s_over_norm, q.z() / v_norm * acos_s_over_norm);
  } else
    return Eigen::Matrix<T, 4, 1>(std::log(q_norm), 0.0, 0.0, 0.0);
}

/**
 * Returns the quaternion conjugte of an input quaternion q.
 * @tparam T The data type of the input quaternion.
 * @param q The input quaternion.
 * @return The quaternion conjugate of q.
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> quatConjugate(const Eigen::Matrix<T, 4, 1> &q) {
  return Eigen::Matrix<T, 4, 1>(q(0), -q(1), -q(2), -q(3));
}

/**
 * Computes the quaternion inverse of an input quaternion q.
 * @tparam T The data type of the input quaternion.
 * @param q The input quaternion.
 * @return The quaternion inverse of q.
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> quatInverse(const Eigen::Matrix<T, 4, 1> &q) {
  T q_norm = std::sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  return Eigen::Matrix<T, 4, 1>(q.w() / q_norm, -q.x() / q_norm, -q.y() / q_norm, -q.z() / q_norm);
}

// Assumes ZYX rotation;
/**
 * Converts euler angles in ZYX orientation to a quaternion.
 * @tparam T The datatype of the input angles.
 * @param yaw_pitch_roll The input angles.
 * @return The output quaternion.
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> angle2quat(Eigen::Matrix<T, 3, 1> const &yaw_pitch_roll) {
  T cyaw = std::cos(yaw_pitch_roll.x() / 2);
  T cpitch = std::cos(yaw_pitch_roll.y() / 2);
  T croll = std::cos(yaw_pitch_roll.z() / 2);

  T syaw = std::sin(yaw_pitch_roll.x() / 2);
  T spitch = std::sin(yaw_pitch_roll.y() / 2);
  T sroll = std::sin(yaw_pitch_roll.z() / 2);

  Eigen::Matrix<T, 4, 1> quat;
  quat.x() = cyaw * cpitch * sroll - syaw * spitch * croll;
  quat.y() = cyaw * spitch * croll + syaw * cpitch * sroll;
  quat.z() = syaw * cpitch * croll - cyaw * spitch * sroll;
  quat.w() = cyaw * cpitch * croll + syaw * spitch * sroll;
  return quat;
}

/**
 * Converts a quaternion to Euler angles in ZYX orientation.
 * @tparam T The data type of the quaternion.
 * @param quat The input quaternion.
 * @return The Euler angles in ZYX orientation.
 */
// Assumes ZYX rotation
template<typename T>
inline Eigen::Matrix<T, 3, 1> quat2angle(Eigen::Matrix<T, 4, 1> const &quat) {
  Eigen::Matrix<T, 4, 1> quat_n = quat;
  T r11 = 2 * (quat_n.x() * quat_n.y() + quat_n.w() * quat_n.z());
  T r12 = std::pow(quat_n.w(), 2) + std::pow(quat_n.x(), 2) - std::pow(quat_n.y(), 2) - std::pow(quat_n.z(), 2);
  T r21 = -2 * (quat_n.x() * quat_n.z() - quat_n.w() * quat_n.y());
  T r31 = 2 * (quat_n.y() * quat_n.z() + quat_n.w() * quat_n.x());
  T r32 = std::pow(quat_n.w(), 2) + std::pow(quat_n.z(), 2) - std::pow(quat_n.x(), 2) - std::pow(quat_n.y(), 2);

  // truncate r21 if above the allowed range
  if (abs(r21) >= static_cast<T>(1.0))
    r21 = ((T(0) < r21) - (r21 < T(0))) * static_cast<T>(1.0); // sign of r21

  Eigen::Matrix<T, 3, 1> yaw_pitch_roll;
  yaw_pitch_roll.x() = std::atan2(r11, r12);
  yaw_pitch_roll.y() = std::asin(r21);
  yaw_pitch_roll.z() = std::atan2(r31, r32);
  return yaw_pitch_roll;
}

/**
 * Converts an axis-angle orientation into a quaternion.
 * @tparam T The data type of the axis-angle representation.
 * @param omega The input axis-angle orientation.
 * @return The output quaternion.
 */
template<typename T>
inline Eigen::Matrix<T, 4, 1> axangle2quat(Eigen::Matrix<T, 3, 1> const &omega) {
  T angle = omega.norm();
  if (angle > 0)
    return Eigen::Matrix<T, 4, 1>(std::cos(angle / 2.0), omega(0) / angle * std::sin(angle / 2.0),
                                  omega(1) / angle * std::sin(angle / 2.0), omega(2) / angle * std::sin(angle / 2.0));
  else
    return Eigen::Matrix<T, 4, 1>(1.0, 0.0, 0.0, 0.0);
}

/**
 * Converts a quaternion to an axis-angle representation.
 * @tparam T The data type of the input quaternion.
 * @param q The quaternion input.
 * @return The output axis-angle representation.
 */
template<typename T>
inline Eigen::Matrix<T, 3, 1> quat2axangle(Eigen::Matrix<T, 4, 1> const &q) {
  T n = q.template tail<3>().norm();
  if (n < std::numeric_limits<T>::epsilon())
    n = q.template tail<3>().stableNorm();

  Eigen::Matrix<T, 3, 1> omega;
  if (n != T(0)) {
    T ang = T(2) * std::atan2(n, std::abs(q.w()));
    if (q.w() < T(0))
      n = -n;
    omega = q.template tail<3>() * ang / n;
  } else
    omega = Eigen::Matrix<T, 3, 1>::Zero();
}

// http://stackoverflow.com/a/14143738/292237
// http://www.codeproject.com/Tips/862988/Find-the-Intersection-Point-of-Two-Line-Segments
inline bool lineSegementsIntersect(const Eigen::Vector2d &p1,
                                   const Eigen::Vector2d &p2,
                                   const Eigen::Vector2d &q1,
                                   const Eigen::Vector2d &q2,
                                   Eigen::Vector2d &intersection) {
  Eigen::Vector2d qp(q1 - p1);
  Eigen::Vector2d r(p2 - p1);
  Eigen::Vector2d s(q2 - q1);
  double rxs = r(0) * s(1) - r(1) * s(0);
  double qpxr = qp(0) * r(1) - qp(1) * r(0);
  double eps = 0.0000001;

  // If r x s = 0 and (q - p) x r = 0, then the two lines are collinear.
  if (std::abs(rxs) < eps && std::abs(qpxr) < eps) {
    // 1. If either  0 <= (q - p) * r <= r * r or 0 <= (p - q) * s <= s * s
    // then the two lines are overlapping,
    if ((0.0 <= qp.dot(r) && qp.dot(r) <= r.dot(r)) ||
        (0.0 <= qp.dot(s) && qp.dot(s) <= s.dot(s))) {
      // infinite number of intersections so return closest point to p1
      intersection = qp.norm() < (q2 - p1).norm() ? q1 : q2;
      return true;
    }
    // 2. If neither 0 <= (q - p) * r = r * r nor 0 <= (p - q) * s <= s * s
    // then the two lines are collinear but disjoint.
    // No need to implement this expression, as it follows from the expression above.
    return false;
  }

  // 3. If r x s = 0 and (q - p) x r != 0, then the two lines are parallel and non-intersecting.
  if (std::abs(rxs) < eps && std::abs(qpxr) >= eps)
    return false;

  double qpxs = qp(0) * s(1) - qp(1) * s(0);
  // t = (q - p) x s / (r x s)
  double t = qpxs / rxs;
  // u = (q - p) x r / (r x s)
  double u = qpxr / rxs;

  // 4. If r x s != 0 and 0 <= t <= 1 and 0 <= u <= 1
  // the two line segments meet at the point p + t r = q + u s.
  if ((std::abs(rxs) >= eps) && (0 <= t && t <= 1) && (0 <= u && u <= 1)) {
    // We can calculate the intersection point using either t or u.
    intersection = p1 + t * r;  // An intersection was found.
    return true;
  }

  // 5. Otherwise, the two line segments are not parallel but do not intersect.
  return false;
}

// Returns:
// - corrdinates of the projection of M on AB
// - the distance d0 between M and P
// - the distance d1 between A and P
// - the distance d2 between B and P
inline std::tuple<std::array<double, 3>, double, double, double>
projPoint2Line(const std::array<double, 3> &A,
               const std::array<double, 3> &B,
               const std::array<double, 3> &M) {
  std::tuple<std::array<double, 3>, double, double, double> retv;
  std::array<double, 3> &P = std::get<0>(retv);
  double &d0 = std::get<1>(retv);
  double &d1 = std::get<2>(retv);
  double &d2 = std::get<3>(retv);

  std::array<double, 3> AB = {B[0] - A[0], B[1] - A[1], B[2] - A[2]};
  std::array<double, 3> AM = {M[0] - A[0], M[1] - A[1], M[2] - A[2]};
  double norm = std::sqrt(AB[0] * AB[0] + AB[1] * AB[1] + AB[2] * AB[2]);
  double dot = AB[0] * AM[0] + AB[1] * AM[1] + AB[2] * AM[2];
  d1 = dot / norm;
  d2 = norm - d1;
  std::array<double, 3> AP = {AB[0] / d1, AB[1] / d1, AB[2] / d1};
  P = {AP[0] - A[0], AP[1] - A[1], AP[2] - A[2]};
  std::array<double, 3> MP = {P[0] - M[0], P[1] - M[1], P[2] - M[2]};
  d0 = std::sqrt(MP[0] * MP[0] + MP[1] * MP[1] + MP[2] * MP[2]);

  return retv;
}



template<typename T>
inline Eigen::Matrix<T, 4, 1> point2homo(const Eigen::Matrix<T, 3, 1> & p){
  Eigen::Matrix<T, 4, 1> ph;
  ph << p.x(), p.y(), p.z(), 1;
  return ph;
}

template<typename T>
inline Eigen::Matrix<T, 3, 1> point2homo(const Eigen::Matrix<T, 2, 1> & p){
  Eigen::Matrix<T, 3, 1> ph;
  ph << p.x(), p.y(), 1;
  return ph;
}

template<typename T>
inline Eigen::Matrix<T, 4, 1> normalizeHomo(const Eigen::Matrix<T, 4, 1>& ph) { return ph/ph.w(); }
template<typename T>
inline Eigen::Matrix<T, 3, 1> normalizeHomo(const Eigen::Matrix<T, 3, 1>& ph) { return ph/ph.z(); }

template<typename T>
inline Eigen::Matrix<T, 3, 1> points2line(const Eigen::Matrix<T, 3, 1>& a, const Eigen::Matrix<T, 3, 1>& b)
{ return a.cross(b); }


}
#endif
