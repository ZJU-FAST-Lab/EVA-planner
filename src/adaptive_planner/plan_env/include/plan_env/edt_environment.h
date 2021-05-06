#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <plan_env/sdf_map.h>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace adaptive_planner {

class EDTEnvironment {
private:
  /* data */
  double resolution_inv_;
  double costmap_alpha_, costmap_r_, costmap_d_ ;     // parameters of the cost distance
  
  double distToBox(int idx, const Eigen::Vector3d& pos, const double& time);
  double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void getSurroundFirstGrad(Eigen::Vector3d pts[2][2][2], double first_grad[2][2][2][3]);
  void interpolateTrilinearEDT(double values[2][2][2], const Eigen::Vector3d& diff, double& value);
  void interpolateTrilinearFirstGrad(double values[2][2][2], const Eigen::Vector3d& diff, Eigen::Vector3d& grad);
  void interpolateTrilinearSecondGrad(double first_grad[2][2][2][3], const Eigen::Vector3d& diff, Eigen::Vector3d& grad);
  
public:
  EDTEnvironment() {};
  ~EDTEnvironment() {};

  SDFMap::Ptr sdf_map_;

  /* Main API*/
  void setParam(ros::NodeHandle& nh);
  void setMap(SDFMap::Ptr map);

  void evaluateEDT(const Eigen::Vector3d& pos, double& dist);
  void evaluateFirstGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
  void evaluateSecondGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);

  void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
    sdf_map_->getRegion(ori, size);
  }

  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace adaptive_planner

#endif