#include <path_searching/astar.h>
#include <Eigen/Core> // L1-norm, manhattanDistance
#include <sstream>
#include <mutex>

using namespace std;
using namespace Eigen;

namespace adaptive_planner {

    Astar::~Astar() {
        for (int i = 0; i < GLX_SIZE; i++) {
            for (int j = 0; j < GLY_SIZE; j++) {
                    delete GridNodeMap[i][j];
            }
            delete GridNodeMap[i];
        }
        delete GridNodeMap;
    }

    void Astar::init() {
        inv_resolution_ = 1.0 / resolution_;
        // global map region (origin and size in 3d)
        sdf_map->getRegion(origin_, map_size_2d_);
        map_min = origin_;
        map_max = origin_ + map_size_2d_;
        cout << "global map min: " << map_min.transpose() << endl;
        cout << "global map max: " << map_max.transpose() << endl;

        // map boundary lower
        gl_xl = map_min(0);
        gl_yl = map_min(1);

        // map boundary upper
        gl_xu = map_max(0);
        gl_yu = map_max(1);

        // graph size, used to create graph container
        GLX_SIZE = (int) (map_size_2d_(0) * inv_resolution_);
        GLY_SIZE = (int) (map_size_2d_(1) * inv_resolution_);

        cout << "grid map size: " << GLX_SIZE * GLY_SIZE << endl;

        GridNodeMap = new GridNodePtr *[GLX_SIZE];
        for (int i = 0; i < GLX_SIZE; i++) {
            GridNodeMap[i] = new GridNodePtr [GLY_SIZE];
            for (int j = 0; j < GLY_SIZE; j++) {
                    Vector2i tmpIdx(i, j);
                    Vector2d pos = gridIndex2coord(tmpIdx);
                    GridNodeMap[i][j] = new GridNode(tmpIdx, pos);
                }
            }
        }


    void Astar::setParam(ros::NodeHandle &nh) {
        nh.param("astar/resolution_astar", resolution_, 0.1);

        // zhl obstacle inflation radius
        nh.param("astar/margin", margin_, 0.3);

        // truncated path outside box center at start with x,y radius = local_margin, z [0, 3.0] hard-coded
        nh.param("astar/local_margin", local_margin_, 10.0);

        visited_nodes_vis_pub_ = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis", 1);
        tie_breaker_ = 1.0 + 1.0 / 1000.0;  //check ref at http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html

    }

    void Astar::setEnvironment(const SDFMap::Ptr &env) {
        this->sdf_map = env;
    }

    Vector2d Astar::gridIndex2coord(const Vector2i &index) {
        Vector2d pt;

        pt(0) = ((double) index(0) + 0.5) * resolution_ + gl_xl;
        pt(1) = ((double) index(1) + 0.5) * resolution_ + gl_yl;

        return pt;
    }

    Vector2i Astar::coord2gridIndex(const Vector2d &pt) {
        Vector2i idx;
        idx << min(max(int((pt(0) - gl_xl) * inv_resolution_), 0), GLX_SIZE - 1),
                min(max(int((pt(1) - gl_yl) * inv_resolution_), 0), GLY_SIZE - 1);
//                min(max(int((pt(2) - gl_zl) * inv_resolution_), 0), GLZ_SIZE - 1);

        return idx;
    }

    Eigen::Vector2d Astar::coordRounding(const Eigen::Vector2d &coord) {
        return gridIndex2coord(coord2gridIndex(coord));
    }

    bool Astar::isOccupied(const Eigen::Vector2i &index) {
        double dist = sdf_map->getDistance(gridIndex2coord(index));
        // zhl: cell within in margin_ distance to obstacle are considered as occupied
        if (dist <= margin_) {
            return true;
        } else {
            return false;
        }
    }

    inline void Astar::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets) {
        neighborPtrSets.clear();
        edgeCostSets.clear();
        Vector2i neighborIdx;
        for (int dx = -1; dx < 2; dx++) {
            for (int dy = -1; dy < 2; dy++) {
                // zhl skip currentNode itself just add neighbors
                if (dx == 0 && dy == 0) continue;

                // adding 26 neighbors for 3d, 8 for 2d
                neighborIdx(0) = (currentPtr->index)(0) + dx;
                neighborIdx(1) = (currentPtr->index)(1) + dy;

                // check if neighbors are within the cell map
                if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE|| neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE) {
                    continue;
                }

                // check if neighbors are in inflated obstacle space outside when updating gvals
                neighborPtrSets.push_back(GridNodeMap[neighborIdx(0)][neighborIdx(1)]);
                edgeCostSets.push_back(sqrt(dx * dx + dy * dy));

            }
        }
    }

    double Astar::getHeu(GridNodePtr node1, GridNodePtr node2) {
        // choose possible heuristic function you want
        // Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
        // http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
        int dx = abs(node1->index(0) - node2->index(0));
        int dy = abs(node1->index(1) - node2->index(1));

        double h = 1.0 * (dx + dy);

        // this is strange, if none of dx, dy, dz equal 0, then h = 0.0? //zhl
        // correct way? https://stackoverflow.com/questions/53116475/calculating-diagonal-distance-in-3-dimensions-for-a-path-finding-heuristic
        return tie_breaker_ * h;
    }


    int Astar::search(Vector2d start_pt, Vector2d end_pt) {
        std::lock_guard<std::mutex> guard(lock);
        ros::Time time_1 = ros::Time::now();
        // must rest node graph status
        for (int i = 0; i < GLX_SIZE; i++) {
            for (int j = 0; j < GLY_SIZE; j++) {
                GridNodeMap[i][j]->id = 0;
            }
        }

        //index of start_point and end_point
        Vector2i start_idx = coord2gridIndex(start_pt);
        Vector2i end_idx = coord2gridIndex(end_pt);
        goalIdx = end_idx;

        //position of start_point and end_point
        start_pt = gridIndex2coord(start_idx);
        end_pt = gridIndex2coord(end_idx);

        //Initialize the pointers of struct GridNode which represent start node and goal node
        GridNodePtr startPtr = new GridNode(start_idx, start_pt);
        GridNodePtr endPtr = new GridNode(end_idx, end_pt);

        //openSet is the open_list implemented through multimap in STL library
        openSet.clear();
        // currentPtr represents the node with lowest f(n) in the open_list
        GridNodePtr currentPtr = NULL;
        GridNodePtr neighborPtr = NULL;

        //put start node in open set
        startPtr->gScore = 0;
        startPtr->fScore = getHeu(startPtr, endPtr);
        startPtr->id = 1;
        startPtr->coord = start_pt;
        startPtr->nodeMapIt = openSet.insert(make_pair(startPtr->fScore, startPtr));

        /*
        *
        STEP 2 :  some preparatory works to be done before while loop
        please write your code below
        *
        *
        */
        double tentative_gScore;
        vector<GridNodePtr> neighborPtrSets;
        vector<double> edgeCostSets;


        while (!openSet.empty()) {
            /*
            *
            *
            step 3: Remove the node with the lowest cost function from open set and put it into closed set
            please write your code below

            IMPORTANT NOTE!!!
            This part you should use the C++ STL: multimap, more details can be found in Homework description
            *
            *
            */
            // get the min f-val node from open set as currentNode
            currentPtr = openSet.begin()->second;
            openSet.erase(openSet.begin());
            currentPtr->id = -1; // put current node into closed set
            // if the current node is the goal
            if (currentPtr->index == goalIdx) {
                //ros::Time time_2 = ros::Time::now();
                terminatePtr = currentPtr;
                // cout << "goal " << endl;
                // ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
                // cout << "[A*]{sucess}  Time in A*  is" << (time_2 - time_1).toSec() * 1000.0 << " ms, path cost is " << currentPtr->gScore * resolution << " m." << endl;
                return 1;
            }
            //get successors/children, and put their point and edge cost into containers defined above
            AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

            /*
            *
            *
            STEP 4:  For all unexpanded neighbors "m" of node "n", please finish this for loop please write your code below
            *
            */
            for (int i = 0; i < (int) neighborPtrSets.size(); i++) {
                /*
                *
                *
                STEP 5:  Judge if the neighbors have been expanded please write your code below

                IMPORTANT NOTE!!!
                neighborPtrSets[i]->id = 1 : unexpanded, openset
                neighborPtrSets[i]->id = -1 : expanded, node with id=-1 are in closed set
                *
                */
                neighborPtr = neighborPtrSets[i];

                // TODO ZHL, why not just use id == 0 to determine whether the node has been explored
                // TODO ZHL, here rounds_ == 1 is a const and new add neighbors with rounds = 0//
                //  so unvisited new neighbors are not explored

                // TODO ZHL, the logic here can be simplified as neighborPtr -> id == -1
                if (neighborPtr->id == -1) {
                    continue;    //in closed set
                }

                // remove neighbors in inflated obstacle space
                if (isOccupied(neighborPtr->index)) continue;

                double edge_cost = edgeCostSets[i];
                tentative_gScore = currentPtr->gScore + edge_cost;

                //TODO ZHL double check if this logic is correct
                if (neighborPtr->id == 0) { //discover a new node
                    /*
                    *
                    *
                    STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                    please write your code below
                    *
                    */
                    neighborPtr->id = 1;
                    neighborPtr->cameFrom = currentPtr;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr)); //put neighbor in open set and record it using iterator.
                    continue;
                } else if (tentative_gScore < neighborPtr->gScore) { //in open set and need update
                    /*
                    *
                    *
                    STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                    please write your code below
                    *
                    */
                    neighborPtr->cameFrom = currentPtr;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr)); //put neighbor in open set and record it.
                    continue;
                }
            }
        }
        //if search fails
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.1) {
            cout << "open set empty, no path!" << endl;
            // ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
            cout << "Timeout > 0.1, consumed time: " << (time_2 - time_1).toSec() << endl;
        }
        ROS_ERROR_THROTTLE(1, "Astar failed");


        // add by zhl
        delete startPtr;
        delete endPtr;
//    delete GridNodeMap; ?


        return 2;
    }

/*
*
*
STEP 8:  Finish the Astar::retrievePath and Astar::getPath
please write your code below
*      
*/
    vector<Vector2d> Astar::getPath() {
        vector<Vector2d> path;
        vector<GridNodePtr> gridPath;

        // trace back from the curretnt nodePtr to get all nodes along the path
        gridPath.push_back(terminatePtr);
        auto currentPtr = terminatePtr;
        while (currentPtr->cameFrom != NULL) {
            currentPtr = currentPtr->cameFrom;
            gridPath.push_back(currentPtr);
        }

        for (auto ptr: gridPath)
            path.push_back(ptr->coord);

        reverse(path.begin(), path.end());

        // simplifyPath = pathSimplify(path); // not used
        // return simplifyPath;
        return path;
    }

    vector<Vector2d> Astar::getLocalPath(const vector<Vector2d>& path) {
        /* Get truncated normalized path from A* planner
         * 1. rewire the path, removing waypoint within straight line that is collision free
         * 2. normalize path so that consecutive segment has equal length
         * */

        // simplify the path by reducing the number of waypoints while still maintaining the path's feasibility.
        auto path_simple = pathSimplify(path);

        // get the normalized path, with new consecutive waypoints separated by equal distance
        vector<Vector2d> path_normalized;
        path_normalized.push_back(path_simple[0]);

        for (auto i = 1; i < path_simple.size(); i++) {
            Vector2d pos_L, pos_R, direction;
            pos_L = path_simple[i - 1];
            pos_R = path_simple[i];
            direction = pos_R - pos_L;
            double dist = direction.norm();
            int num = round(dist / resolution_); // new path len?
            double delta_x, delta_y;
            delta_x = direction(0) / num;  // normalized step size in each direction
            delta_y = direction(1) / num;
            Vector2d delta;
            delta << delta_x, delta_y;
            // Vector2d delta = direction / num;
            for (int j = 0; j < num; j++) {
                Vector2d pos;
                pos = pos_L + (j + 1) * delta;
                path_normalized.push_back(pos);
            }
            path_normalized.push_back(pos_R);
        }

        // get the local path
        Vector2d start = path_normalized[0];
        Vector2d local_min, local_max;
        // new local path region box lower and upper bounds, center at (start_x, start_y, 1.5),
        local_min(0) = start(0) - local_margin_;
        local_min(1) = start(1) - local_margin_;
        local_max(0) = start(0) + local_margin_;
        local_max(1) = start(1) + local_margin_;

        vector<Vector2d> local_path;
        local_path.push_back(path_normalized[0]);
        for (auto i = 1; i < path_normalized.size(); i++) {
            Vector2d pos = path_normalized[i];
            if (pos(0) < local_min(0) || pos(0) > local_max(0) ||
                pos(1) < local_min(1) || pos(1) > local_max(1)) {   // terminate search when first outside box point detected
                break;
            } else {
                local_path.push_back(pos);
            }
        }

        return local_path;
    }

    vector<Vector2d> Astar::pathSimplify(const vector<Vector2d> &path) {
        // This algorithm iterates through the input path, attempting to simplify it by removing intermediate points while avoiding obstacles.
        vector<Vector2d> subPath;
        int path_node_num = path.size();
        if (path_node_num == 0) {
            return path;
        }
        int flag = 0;
        Vector2d cut_start = path[0];
        subPath.push_back(cut_start);

        while (flag < (path_node_num - 1)) {
            // for (int i = flag;i < path_node_num ; i++){
            for (int i = flag + 1; i < path_node_num; i++) {
                bool isSafe = true;
                Vector2d check_pt = path[i];
                Vector2d delta_pt = check_pt - cut_start;
                int check_num = ceil(delta_pt.norm() / resolution_);

                // check intermediate points within straight line starting at cut_start end at check_pt
                for (int j = 0; j <= check_num; j++) {
                    Vector2d mid_pt;
                    double t = 1.0 / check_num * j;
                    mid_pt = (1 - t) * cut_start + t * check_pt;

                    if (isOccupied(coord2gridIndex(mid_pt))) {
                        isSafe = false;
                        break;
                    }
                }

                // end point handling
                if (isSafe && i == (path_node_num - 1)) {
                    flag = i;
                    subPath.push_back(check_pt);
                }

                // skip current point, move pointer to next waypoint
                if (isSafe) {
                    continue;
                }

                    // current examine straight line is colliding,
                    // use last iteration's end point as new starting point
                else {
                    flag = i;
                    cut_start = path[flag - 1];
                    subPath.push_back(cut_start);
                    break;
                }
            }
        }
        return subPath;
    }

    vector<Vector2d> Astar::getVisitedNodes() {
        vector<Vector2d> visited_nodes;
        for (int i = 0; i < GLX_SIZE; i++)
            for (int j = 0; j < GLY_SIZE; j++)
                if(GridNodeMap[i][j]->id != 0) // visualize all nodes in open or closed set
                    visited_nodes.push_back(GridNodeMap[i][j]->coord);
        // ROS_WARN("visited_nodes size : %d", visited_nodes.size());
        cout << "visited_nodes size : " << visited_nodes.size() << endl;
        return visited_nodes;
    }

    void Astar::visVisitedNode() {
        vector<Vector2d> nodes = getVisitedNodes();
        visualization_msgs::Marker node_vis;
        node_vis.header.frame_id = "world";
        node_vis.header.stamp = ros::Time::now();
        node_vis.ns = "astar/expanded_nodes";
        node_vis.type = visualization_msgs::Marker::CUBE_LIST;
        node_vis.action = visualization_msgs::Marker::ADD;
        node_vis.id = 0;

        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;
        node_vis.color.a = 0.5;
        node_vis.color.r = 0.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 1.0;

        node_vis.scale.x = resolution_;
        node_vis.scale.y = resolution_;
        node_vis.scale.z = resolution_;

        geometry_msgs::Point pt;
        for (int i = 0; i < int(nodes.size()); i++) {
            Vector2d coord = nodes[i];
            pt.x = coord(0);
            pt.y = coord(1);
            //  pt.z = coord(2); TODO ZHL default height for visualziation

            node_vis.points.push_back(pt);
        }

        visited_nodes_vis_pub_.publish(node_vis);
    }

}  // namespace adaptive_planner
