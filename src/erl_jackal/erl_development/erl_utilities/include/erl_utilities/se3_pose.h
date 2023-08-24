
#ifndef __ERL_SE3_POSE_H_
#define __ERL_SE3_POSE_H_

#include "erl_utilities/erl_geom_utils.h"

// Define 6D.
namespace Eigen {
using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;
}

namespace erl {

/**
 * SE3Pose stores position and orientation of a rigid body with respect to a world frame. Also contains functions
 * for manipulating 2D and 3D geometric quantities.
 */
struct SE3Pose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * Exception to throw if an invalid orientation is given.
   */
  class InvalidOrientation : public std::exception {
    virtual const char *what() const throw() {
      return "Invalid Orientation. Not in SO(3)!";
    }
  } InvalidOrientationCreated;

  Eigen::Vector3d position{0, 0, 0};
  Eigen::Matrix3d orientation{Eigen::Matrix3d::Identity()};

  /**
   * Default Ctor.
   */
  SE3Pose() {}

  /**
   * Constructs an SE3Pose from a position and orientation matrix.
   * @param position The position.
   * @param orientation The orientation as a matrix.
   */
  SE3Pose(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation) :
      position(position), orientation(orientation) {
    // Ensure valid orientation.
    double eps = 1e-3; // Tolerance for orientation.
    if (std::abs(orientation.determinant() - 1.0) > eps)
      throw InvalidOrientationCreated;
  }

  /**
   * Constructs an SE3Pose from a position and quaternion.
   * @param position The position.
   * @param quaternion The orientation as a quaternion.
   */
  SE3Pose(const Eigen::Vector3d &position, const Eigen::Vector4d &quaternion) : position(position) {
    orientation = quat2rot(quaternion); // Convert quaternion to orientation.
  }

  /**
   * @brief Construct an SE3Pose from an SE2Pose, given a 3 dimensional input. Assumes z is zero.
   * @param se2_pose The SE2Pose to construct the SE3Pose with. (X, Y, Yaw).
   */
  SE3Pose(const Eigen::Vector3d &se2_pose) :
      position(Eigen::Vector3d(se2_pose[0], se2_pose[1], 0.0)), orientation(rotz(se2_pose[2])) {}

  /**
   * Construct an SE(3) pose from an SE(2) pose matrix. Assumes z is zero.
   * @param se2_matrix Input matrix.
   */
  SE3Pose(const Eigen::Matrix3d &se2_matrix) :
      position(Eigen::Vector3d(se2_matrix(0, 2), se2_matrix(1, 2), 0.0)),
      orientation(rotz(std::atan2(se2_matrix(1, 0), se2_matrix(0, 0)))) {}

  /**
   * Construct an SE(3) pose from a 4x4 homogeneous coordinate matrix.
   * @param se3_matrix
   */
  SE3Pose(const Eigen::Matrix4d &se3_matrix) :
      position(se3_matrix.block(0, 3, 3, 1)), orientation(se3_matrix.block(0, 0, 3, 3)) {}

  /**
   * Construct an SE(3) pose from (x, y, z, roll, pitch, yaw)
   * @param se3_vector The 6-D vector containing position and Euler angles.
   */
  SE3Pose(const Eigen::Matrix<double, 6, 1> &se3_vector) :
      position(se3_vector.head(3)),
      orientation(rot3d(se3_vector[3], se3_vector[4], se3_vector[5])) {}

  /**
   * Returns the Yaw of the orientation.
   * @return The yaw.
   */
  double getYaw() const {
    return std::atan2(orientation(1, 0), orientation(0, 0));
  }

  /**
   * Returns the Pitch of the orientation.
   * @return The pitch.
   */
  double getPitch() const {
    return std::atan2(-orientation(2, 0), std::hypot(orientation(2, 1), orientation(2, 2)));
  }

  /**
   * Returns the Roll of the orientation.
   * @return The roll.
   */
  double getRoll() const {
    return std::atan2(orientation(2, 1), orientation(2, 2));
  }

  /**
   * Returns the SE(2) projection of the full SE(3) Pose.
   * @return The SE(2) pose.
   */
  Eigen::Vector3d getSE2() const {
    Eigen::Vector3d result;
    result << position[0], position[1], getYaw();
    return result;
  }

  Eigen::Vector4d getQuaternion() const {
    return rot2quat(orientation);
  }

  /**
   * Returns the homogeneous SE(3) matrix representation of the full SE(3) pose.
   * @return The matrix representation.
   */
  Eigen::Matrix4d getMatrix() const {
    Eigen::Matrix4d result;
    result.setZero();
    result.block(0, 0, 3, 3) = orientation;
    result.block(0, 3, 3, 1) = position;
    result(3, 3) = 1;
    return result;
  }

  /**
   * Generates a homogeneous coordinate representation of a pose projected down to SE(2).
   * @return The SE2 matrix.
   */
  Eigen::Matrix3d getSE2Matrix() const {
    Eigen::Matrix3d result;
    result.setZero();
    result(2, 2) = 1;
    result.block(0, 0, 2, 2) = orientation.block(0, 0, 2, 2);
    result.block(0, 2, 2, 1) = position.head(2);
    return result;
  }

  /**
   * Generate the hatMap matrix in SE(2) of a given pose when projected into SE(2).
   * @return The HatMap matrix of an SE(2) pose.
   */
  Eigen::Matrix3d getSE2HatMap() const {
    // TODO. Is this even right????
    Eigen::Matrix3d result;
    result.setZero();
    result.block(0, 0, 2, 2) = orientation.block(0, 0, 2, 2);
    result.block(0, 2, 2, 1) = position.head(2);
    return result;
  }

  Eigen::Matrix<double, 6, 1> getVectorForm() const {
    Eigen::Matrix<double, 6, 1> state;
    state << position[0], position[1], position[2], getRoll(), getPitch(), getYaw();
    return state;
  }
};

}
#endif


