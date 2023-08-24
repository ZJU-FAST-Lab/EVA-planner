#include <plan_env/edt_environment.h>

namespace adaptive_planner {
/* ============================== edt_environment ==============================
 */
void EDTEnvironment::setParam(ros::NodeHandle& nh) {
    
}   

void EDTEnvironment::setMap(shared_ptr<SDFMap>& map) {
  this->sdf_map_ = map;
  resolution_inv_ = 1 / sdf_map_->getResolution();
}

void EDTEnvironment::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
          dists[x][y] = sdf_map_->getDistance(pts[x][y]);
    }
  }
}

void EDTEnvironment::getSurroundFirstGrad(Eigen::Vector2d pts[2][2], double first_grad[2][2][2]){
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
                Eigen::Vector2d grad;
                evaluateFirstGrad(pts[x][y], grad);
                first_grad[x][y][0] = grad(0);
                first_grad[x][y][1] = grad(1);
        }
    }
}

void EDTEnvironment::evaluateEDT(const Eigen::Vector2d& pos,
                                 double& dist) {
  Eigen::Vector2d diff;
  Eigen::Vector2d sur_pts[2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double dists[2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearEDT(dists, diff, dist);
}

void EDTEnvironment::evaluateFirstGrad(const Eigen::Vector2d& pos,
                                              Eigen::Vector2d& grad) {
  Eigen::Vector2d diff;
  Eigen::Vector2d sur_pts[2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double dists[2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearFirstGrad(dists, diff, grad);
}

void EDTEnvironment::evaluateSecondGrad(const Eigen::Vector2d& pos,
                                              Eigen::Vector2d& grad) {
  Eigen::Vector2d diff;
  Eigen::Vector2d sur_pts[2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double first_grad[2][2][2];
  getSurroundFirstGrad(sur_pts, first_grad);

  interpolateTrilinearSecondGrad(first_grad, diff, grad);
}

void EDTEnvironment::interpolateTrilinearEDT(double values[2][2],
                                             const Eigen::Vector2d& diff,
                                             double& value) {
  // trilinear interpolation
  double v0 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0]; // b
  double v1 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1]; // c
  
  value = (1 - diff(1)) * v0 + diff(1) * v1;
}

void EDTEnvironment::interpolateTrilinearFirstGrad(double values[2][2],
                                                   const Eigen::Vector2d& diff,
                                                   Eigen::Vector2d& grad) const {
  // trilinear interpolation
  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];

  grad[1] = (v1 - v0) * resolution_inv_;
  grad[0] = (1 - diff[1]) * (values[1][0] - values[0][0]);
  grad[0] += diff[1] * (values[1][1] - values[0][1]);

  grad[0] *= resolution_inv_;
}

void EDTEnvironment::interpolateTrilinearSecondGrad(double first_grad[2][2][2],
                                                    const Eigen::Vector2d& diff,
                                                    Eigen::Vector2d& grad) const {
    grad[0] =  (1 - diff[1]) * ( (first_grad[1][0][0] - first_grad[0][0][0]));
    grad[0] += diff[1] * ( (first_grad[1][1][0] - first_grad[0][1][0]) );
    grad[0] *= resolution_inv_;

    grad[1] =  ( (1 - diff[0]) * (first_grad[0][1][0] - first_grad[0][0][0]) + diff[0] * (first_grad[1][1][0] - first_grad[1][0][0]) );
    grad[1] *= resolution_inv_;
}

// EDTEnvironment::
}  // namespace adaptive_planner