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

#define inf 1>>20
namespace adaptive_planner {

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int rounds;
    int id;        // 1--> open set, -1 --> closed set
    
    Eigen::Vector3i index;
    Eigen::Vector3d coord;
	
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		rounds = 0;
    id = 0;
		index = _index;
		coord = _coord;

		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};

class Astar
{	
	private:
		/* ---------- main data structure ---------- */
    uint8_t * data;
		GridNodePtr *** GridNodeMap;
		Eigen::Vector3i goalIdx;
    GridNodePtr terminatePtr;
		std::multimap<double, GridNodePtr> openSet;

    /* ---------- record data ---------- */
    SDFMap::Ptr sdf_map;
    bool has_path = false;

    /* ---------- parameter ---------- */
    /* search */
    double margin_;
    double local_margin_;
		double resolution_, inv_resolution_;
    double tie_breaker_;
    std::vector<Eigen::Vector3d> simplifyPath;
    int rounds_{0};
    int margin_num_;

    /* map */
    Eigen::Vector3d origin_, map_size_3d_;
    Eigen::Vector3d map_min, map_max;
    Eigen::Vector3d local_min_,local_max_;
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;

		/* heuristic function */
		double getHeu(GridNodePtr node1, GridNodePtr node2);
		
    /* helper */
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		
		bool isOccupied(const Eigen::Vector3i & index);
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

    ros::Publisher visited_nodes_vis_pub_;

	public:
		Astar(){};
		~Astar();

    enum { REACH_END = 1, NO_PATH = 2 };

    /* main API */
    void setParam(ros::NodeHandle& nh);
    void init();

		int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    void setEnvironment(const SDFMap::Ptr& env);
		
		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		std::vector<Eigen::Vector3d> getPath();
    std::vector<Eigen::Vector3d> getLocalPath();
		std::vector<Eigen::Vector3d> getVisitedNodes();
    void visVisitedNode();

		std::vector<Eigen::Vector3d> pathSimplify(const std::vector<Eigen::Vector3d> &path);

    typedef shared_ptr<Astar> Ptr;
};

}  // namespace adaptive_planner

#endif