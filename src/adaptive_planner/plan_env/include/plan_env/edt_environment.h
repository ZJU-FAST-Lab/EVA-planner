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

  void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
  void getSurroundFirstGrad(Eigen::Vector2d pts[2][2], double first_grad[2][2][2]);
  static void interpolateTrilinearEDT(double values[2][2], const Eigen::Vector2d& diff, double& value);
  void interpolateTrilinearFirstGrad(double values[2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad) const;
  void interpolateTrilinearSecondGrad(double first_grad[2][2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad) const;
  
public:
  EDTEnvironment() {};
  ~EDTEnvironment() {};

  SDFMap::Ptr sdf_map_;

  /* Main API*/
  void setParam(ros::NodeHandle& nh);
  void setMap(SDFMap::Ptr& map);

  void evaluateEDT(const Eigen::Vector2d& pos, double& dist);
  void evaluateFirstGrad(const Eigen::Vector2d& pos, Eigen::Vector2d& grad);
  void evaluateSecondGrad(const Eigen::Vector2d& pos, Eigen::Vector2d& grad);

  void getMapRegion(Eigen::Vector2d& ori, Eigen::Vector2d& size) {
    sdf_map_->getRegion(ori, size);
  }

  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace adaptive_planner

#endif