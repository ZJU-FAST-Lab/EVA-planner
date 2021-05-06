#include <plan_env/edt_environment.h>

namespace adaptive_planner {
/* ============================== edt_environment ==============================
 */
void EDTEnvironment::setParam(ros::NodeHandle& nh) {
    
}   

void EDTEnvironment::setMap(shared_ptr<SDFMap> map) {
  this->sdf_map_ = map;
  resolution_inv_ = 1 / sdf_map_->getResolution();
}

void EDTEnvironment::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
          dists[x][y][z] = sdf_map_->getDistance(pts[x][y][z]);
      }
    }
  }
}

void EDTEnvironment::getSurroundFirstGrad(Eigen::Vector3d pts[2][2][2], double first_grad[2][2][2][3]){
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            for (int z = 0; z < 2; z++) {
                Eigen::Vector3d grad;
                evaluateFirstGrad(pts[x][y][z], grad);
                first_grad[x][y][z][0] = grad(0);
                first_grad[x][y][z][1] = grad(1);
                first_grad[x][y][z][2] = grad(2);
            }
        }
    }
}

void EDTEnvironment::evaluateEDT(const Eigen::Vector3d& pos,
                                 double& dist) {
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearEDT(dists, diff, dist);
}

void EDTEnvironment::evaluateFirstGrad(const Eigen::Vector3d& pos,
                                              Eigen::Vector3d& grad) {
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearFirstGrad(dists, diff, grad);
}

void EDTEnvironment::evaluateSecondGrad(const Eigen::Vector3d& pos,
                                              Eigen::Vector3d& grad) {
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double first_grad[2][2][2][3];
  getSurroundFirstGrad(sur_pts, first_grad);

  interpolateTrilinearSecondGrad(first_grad, diff, grad);
}

void EDTEnvironment::interpolateTrilinearEDT(double values[2][2][2],
                                             const Eigen::Vector3d& diff,
                                             double& value){
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;  // e
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;  // f
  
  value = (1 - diff(2)) * v0 + diff(2) * v1;
}

void EDTEnvironment::interpolateTrilinearFirstGrad(double values[2][2][2],
                                                   const Eigen::Vector3d& diff,
                                                   Eigen::Vector3d& grad) {
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;  // e
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;  // f
  
  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= resolution_inv_;
}

void EDTEnvironment::interpolateTrilinearSecondGrad(double first_grad[2][2][2][3],
                                                          const Eigen::Vector3d& diff,
                                                          Eigen::Vector3d& grad) {
    grad[0] =  (1 - diff[1]) * ( (1 - diff[2]) * (first_grad[1][0][0][0] - first_grad[0][0][0][0]) + diff[2] * (first_grad[1][0][1][0] - first_grad[0][0][1][0]) );
    grad[0] += diff[1] * ( (1 - diff[2]) * (first_grad[1][1][0][0] - first_grad[0][1][0][0]) + diff[2] * (first_grad[1][1][1][0] - first_grad[0][1][1][0]) );
    grad[0] *= resolution_inv_;

    grad[1] =  (1 - diff[2]) * ( (1 - diff[0]) * (first_grad[0][1][0][1] - first_grad[0][0][0][1]) + diff[0] * (first_grad[1][1][0][1] - first_grad[1][0][0][1]) );
    grad[1] += diff[2] * ( (1 - diff[0]) * (first_grad[0][1][1][1] - first_grad[0][0][1][1]) + diff[0] * (first_grad[1][1][1][1] - first_grad[1][0][1][1]) );
    grad[1] *= resolution_inv_;
    
    grad[2] =  (1 - diff[1]) * ( (1 - diff[0]) * (first_grad[0][0][1][2] - first_grad[0][0][0][2]) + diff[0] * (first_grad[1][0][1][2] - first_grad[1][0][0][2]) );
    grad[2] += diff[1] * ( (1 - diff[0]) * (first_grad[0][1][1][2] - first_grad[0][1][0][2]) + diff[0] * (first_grad[1][1][1][2] - first_grad[1][1][0][2]) );
    grad[2] *= resolution_inv_;

}


// EDTEnvironment::
}  // namespace adaptive_planner