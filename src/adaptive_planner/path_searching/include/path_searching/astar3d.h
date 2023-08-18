#ifndef _ASTAR_H
#define _ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include "plan_env/sdf_map.h"
#include <math.h>
#include <limits>

//#define inf 1>>20 bug zhl

namespace adaptive_planner {

    struct GridNode;
    typedef GridNode* GridNodePtr; // a type alias (GridNodePtr) for GridNode pointer,

    struct GridNode {
        int rounds = 0;
        int id = 0;        // 1--> open set, -1 --> closed set, // zhl, 0 maybe unvisited

        Eigen::Vector3i index;
        Eigen::Vector3d coord;

        double gScore, fScore;
        GridNodePtr cameFrom; // parent node
        std::multimap<double, GridNodePtr>::iterator nodeMapIt;

        GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord) {
            rounds = 0;
            id = 0;
            index = _index;
            coord = _coord;

            gScore = std::numeric_limits<double>::infinity();
            fScore = std::numeric_limits<double>::infinity();

            cameFrom = NULL;
        }

        GridNode() {};

        ~GridNode() {};
    };

    class Astar3d {
    private:
        /* ---------- main data structure ---------- */
        uint8_t *data;
        GridNodePtr ***GridNodeMap; // like 3d array holding GridNodePtr
        Eigen::Vector3i goalIdx;
        GridNodePtr terminatePtr;
        // using double as the key type for fval of A* nodes, which means the keys will be sorted in ascending order by default.
        // this is our priority queue
        std::multimap<double, GridNodePtr> openSet;  // dictionary holding (fval, nodeptr)

        /* ---------- record data ---------- */
        SDFMap::Ptr sdf_map;
        bool has_path = false;

        /* ---------- parameter ---------- */
        /* search */
        double margin_; // zhl inflation radius; safe distance over sdf map obstacle space, less than this margin to obstacle is considered occupied, unsafe
        double local_margin_;
        double resolution_;         // grid map resolution in meter m/cell
        double inv_resolution_;     // graph map resolution cell/m
        double tie_breaker_;
        std::vector<Eigen::Vector3d> simplifyPath;
        int rounds_{0};
        int margin_num_;
        std::mutex lock;

        /* map */
        Eigen::Vector3d origin_, map_size_3d_;
        Eigen::Vector3d map_min, map_max;
        Eigen::Vector3d local_min_, local_max_;

        int GLX_SIZE, GLY_SIZE, GLZ_SIZE;   // cell map size in x, y, z dimension
        int GLXYZ_SIZE, GLYZ_SIZE;
        double gl_xl, gl_yl, gl_zl;         // cell map origin in meter
        double gl_xu, gl_yu, gl_zu;

        /* heuristic function */
        double getHeu(GridNodePtr node1, GridNodePtr node2);

        /* helper */
        void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> &neighborPtrSets, std::vector<double> &edgeCostSets);

        bool isOccupied(const Eigen::Vector3i &index);

        Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index);

        Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);

        ros::Publisher visited_nodes_vis_pub_;

    public:
        Astar3d() {};

        ~Astar3d();

        enum PlanningStatus{SUCCESS, FAILED};

        /* main API */
        void setParam(ros::NodeHandle &nh);

        void init();

        PlanningStatus search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

        void setEnvironment(const SDFMap::Ptr &env);

        Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

        std::vector<Eigen::Vector3d> getPath();

        std::vector<Eigen::Vector3d> getLocalPath(const vector<Eigen::Vector3d> &path);

        std::vector<Eigen::Vector3d> getVisitedNodes();

        void visVisitedNode();

        std::vector<Eigen::Vector3d> pathSimplify(const std::vector<Eigen::Vector3d> &path);

        typedef shared_ptr<Astar3d> Ptr;
    };

}  // namespace adaptive_planner

#endif
