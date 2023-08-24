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
        int id = 0;        // 1--> open set, -1 --> closed set, 0 unvisited

        Eigen::Vector2i index;
        Eigen::Vector2d coord;

        double gScore, fScore;
        GridNodePtr cameFrom; // parent node
        std::multimap<double, GridNodePtr>::iterator nodeMapIt;

        GridNode(Eigen::Vector2i _index, Eigen::Vector2d _coord) {
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

    class Astar {
    private:
        /* ---------- main data structure ---------- */
        uint8_t *data;
        GridNodePtr **GridNodeMap; // like 2d array holding GridNodePtr
        Eigen::Vector2i goalIdx;
        GridNodePtr terminatePtr;
        // using double as the key type for fval of A* nodes, which means the keys will be sorted in ascending order by default.
        // this is our priority queue
        std::multimap<double, GridNodePtr> openSet;  // dictionary holding (f-val, nodeptr)

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
        std::vector<Eigen::Vector2d> simplifyPath;
        int rounds_{0};
        int margin_num_;
        std::mutex lock;

        /* map */
        Eigen::Vector2d origin_, map_size_2d_;
        Eigen::Vector2d map_min, map_max;
        Eigen::Vector2d local_min_, local_max_;

        int GLX_SIZE, GLY_SIZE;   // cell map size in x, y

        double gl_xl, gl_yl;         // cell map origin in meter
        double gl_xu, gl_yu;

        /* heuristic function */
        double getHeu(GridNodePtr node1, GridNodePtr node2);

        /* helper */
        void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> &neighborPtrSets, std::vector<double> &edgeCostSets);

        bool isOccupied(const Eigen::Vector2i &index);

        Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i &index);

        Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt);

        ros::Publisher visited_nodes_vis_pub_;

    public:

        Astar() {};

        ~Astar();

        /* main API */
        void setParam(ros::NodeHandle &nh);

        void init();

        int search(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);

        void setEnvironment(const SDFMap::Ptr &env);

        Eigen::Vector2d coordRounding(const Eigen::Vector2d &coord);

        std::vector<Eigen::Vector2d> getPath();

        std::vector<Eigen::Vector2d> getLocalPath(const vector<Eigen::Vector2d> &path);

        std::vector<Eigen::Vector2d> getVisitedNodes();

        void visVisitedNode();

        std::vector<Eigen::Vector2d> pathSimplify(const std::vector<Eigen::Vector2d> &path);

        typedef shared_ptr<Astar> Ptr;
    };

}  // namespace adaptive_planner

#endif
