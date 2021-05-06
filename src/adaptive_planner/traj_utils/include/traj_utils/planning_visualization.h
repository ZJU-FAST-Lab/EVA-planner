#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <memory>

using std::vector;
namespace adaptive_planner {

class PlanningVisualization {
private:
    /* data */
    /* visib_pub is seperated from previous ones for different info */
    ros::NodeHandle node;
    ros::Publisher path_pub, low_mpc_traj_pub_, local_goal_pub_, high_mpc_traj_pub_, high_mpc_traj_ref_pub_, pos_cmd_pub_, exected_traj_pub_;
    vector<ros::Publisher> pubs_;

public:
    PlanningVisualization(){}
    ~PlanningVisualization(){}
    PlanningVisualization(ros::NodeHandle& nh);

    // draw basic shapes
    void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                           const Eigen::Vector4d& color, int id, int pub_id = 0);
    void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);
    void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                         double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);
    void displayLineStrip(const vector<Eigen::Vector3d>& list, double line_width,
                          const Eigen::Vector4d& color, int id, int pub_id = 0);
    void drawPosCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                    const Eigen::Vector4d& color);

    // draw trajectory: Goal,
    void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
    void drawLocalGoal(Eigen::Vector3d local_goal, double resolution, const Eigen::Vector4d& color, int id = 0);
    void drawPath(const vector<Eigen::Vector3d>& path, double resolution,const Eigen::Vector4d& color, int id = 0);
    void drawLowMpcTraj(const vector<Eigen::Vector3d>& traj, double resolution,const Eigen::Vector4d& color, int id = 0);
    void drawHighMpccTraj(const vector<Eigen::Vector3d>& traj, double resolution,const Eigen::Vector4d& color, int id = 0);
    void drawHighMpccRefTraj(const vector<Eigen::Vector3d>& traj, double resolution,const Eigen::Vector4d& color, int id = 0);
    void drawExectedTraj(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double resolution,const Eigen::Vector4d& color, int id = 0);


    typedef std::shared_ptr<PlanningVisualization> Ptr;
};

}	// namespace adaptive_planner
#endif