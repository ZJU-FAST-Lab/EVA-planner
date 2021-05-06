#include <path_searching/astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace adaptive_planner {

Astar::~Astar()
{
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for( int k = 0; k < GLZ_SIZE;k++)
                delete GridNodeMap[i][j][k];
}

void Astar::init()
{   
    inv_resolution_ = 1.0 / resolution_;
    // global map region
    sdf_map->getRegion(origin_, map_size_3d_);
    map_min = origin_;
    map_max = origin_ + map_size_3d_; 
    cout << "global map min: " << map_min.transpose() << endl;
    cout << "global map max: " << map_max.transpose() << endl;
    
    gl_xl = map_min(0);
    gl_yl = map_min(1);
    gl_zl = map_min(2);

    gl_xu = map_max(0);
    gl_yu = map_max(1);
    gl_zu = map_max(2);

    GLX_SIZE = (int)(map_size_3d_(0) * inv_resolution_);
    GLY_SIZE = (int)(map_size_3d_(1) * inv_resolution_);
    GLZ_SIZE = (int)(map_size_3d_(2) * inv_resolution_);
    
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE; 

    cout << "grid map size: " << GLXYZ_SIZE << endl;
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void Astar::setParam(ros::NodeHandle& nh){
  nh.param("astar/resolution_astar", resolution_, -1.0);
  nh.param("astar/margin", margin_, -1.0);
  nh.param("astar/local_margin", local_margin_, -1.0);

  visited_nodes_vis_pub_ = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);

  margin_num_ = ceil(margin_/resolution_);
  tie_breaker_ = 1 + 1/10000;
}

void Astar::setEnvironment(const SDFMap::Ptr& env) {
  this->sdf_map = env;
}

Vector3d Astar::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution_ + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution_ + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution_ + gl_zl;

    return pt;
}

Vector3i Astar::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution_), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution_), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution_), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d Astar::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

bool Astar::isOccupied(const Eigen::Vector3i & index)
{
    Eigen::Vector3d pro_pos;
    pro_pos = gridIndex2coord(index);
    double dist = sdf_map->getDistance(pro_pos);
    if (dist <= margin_){
      return true;
    }else {
      return false;
    }
    // int is_occupied = 0;
    // for (int i=-margin_num_;i<=margin_num_;i++)
    //     for (int j=-margin_num_;j<=margin_num_;j++)
    //         for (int k=-margin_num_;k<=margin_num_;k++){
    //             Eigen::Vector3i idx;
    //             Eigen::Vector3d pos;
    //             idx << index(0)+i, index(1)+j, index(2)+k;
    //             pos = gridIndex2coord(idx);
    //             int occupied = sdf_map->getInflateOccupancy(pos);
    //             if (occupied == 1){
    //                 is_occupied = 1;
    //             }
    //         }
    
    // if (is_occupied == 1){
    //     return true;
    // } else{
    //     return false;
    // }
}

inline void Astar::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    Vector3i neighborIdx;
    for(int dx = -1; dx < 2; dx++){
        for(int dy = -1; dy < 2; dy++){
            for(int dz = -1; dz < 2; dz++){
                
                if( dx == 0 && dy == 0 && dz ==0 )
                    continue; 

                neighborIdx(0) = (currentPtr -> index)(0) + dx;
                neighborIdx(1) = (currentPtr -> index)(1) + dy;
                neighborIdx(2) = (currentPtr -> index)(2) + dz;

                if(    neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE
                    || neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE
                    || neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE){
                    continue;
                }

                neighborPtrSets.push_back(GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
                edgeCostSets.   push_back(sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }
}

double Astar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    // choose possible heuristic function you want
    // Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    // Remember tie_breaker learned in lecture, add it here ?
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return tie_breaker_ * h;
    //return 0;
}

int Astar::search(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();  

    ++ rounds_;
    // cout << "astar num:  " << rounds_ << endl;  

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> rounds = rounds_;
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr); 
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );

    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    double tentative_gScore;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    // cout << "in while loop " << endl;
    int dnum = 1;
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        currentPtr = openSet.begin() ->second;
        openSet.erase(openSet.begin());
        currentPtr -> id = -1; 
        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            //ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            // cout << "goal " << endl;
            // ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );  
            // cout << "[A*]{sucess}  Time in A*  is" << (time_2 - time_1).toSec() * 1000.0 << " ms, path cost is " << currentPtr->gScore * resolution << " m." << endl;
            return REACH_END;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);        

        /*
        *
        *
        STEP 4:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            STEP 5:  Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = 1 : unexpanded, openset
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            *        
            */
            neighborPtr = neighborPtrSets[i];
            
            bool flag_explored = (neighborPtr->rounds == rounds_);
            if (flag_explored && neighborPtr -> id == -1){
                continue;    //in closed set
            }

            neighborPtr->rounds = rounds_;

            if( isOccupied(neighborPtr->index))
                continue;
            
            double edge_cost = edgeCostSets[i];            
            tentative_gScore = currentPtr -> gScore + edge_cost;

            if( ! flag_explored){ //discover a new node
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                neighborPtr -> id        = 1;
                neighborPtr -> cameFrom  = currentPtr;
                neighborPtr -> gScore    = tentative_gScore;
                neighborPtr -> fScore    = neighborPtr -> gScore + getHeu(neighborPtr, endPtr); 
                neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
                continue;
            }
            else if(tentative_gScore < neighborPtr-> gScore){ //in open set and need update
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                neighborPtr -> cameFrom = currentPtr;
                neighborPtr -> gScore = tentative_gScore;
                neighborPtr -> fScore = tentative_gScore + getHeu(neighborPtr, endPtr); 
                // cout << "1" << endl;
                openSet.erase(neighborPtr -> nodeMapIt);
                // cout << "2" << endl;
                neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1){
        cout << "open set empty, no path!" << endl;
        // ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
        cout << "Time consume in Astar path finding is " << (time_2 - time_1).toSec() << endl; 
    }
    return NO_PATH;
}

/*
*
*
STEP 8:  Finish the Astar::retrievePath and Astar::getPath
please write your code below
*      
*/
vector<Vector3d> Astar::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    // trace back from the curretnt nodePtr to get all nodes along the path
    gridPath.push_back(terminatePtr);
    auto currentPtr = terminatePtr;
    while(currentPtr->cameFrom != NULL){
        currentPtr = currentPtr ->cameFrom;
        gridPath.push_back(currentPtr);
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    simplifyPath = pathSimplify(path);
    // return simplifyPath;
    return path;
}

vector<Vector3d> Astar::getLocalPath()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    // trace back from the curretnt nodePtr to get all nodes along the path
    gridPath.push_back(terminatePtr);
    auto currentPtr = terminatePtr;
    while(currentPtr->cameFrom != NULL){
        currentPtr = currentPtr ->cameFrom;
        gridPath.push_back(currentPtr);
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());
    // cout <<"get path , the path size : " << path.size() << endl;

    // simplify the path
    vector<Vector3d> path_simple;
    path_simple = pathSimplify(path);

    // get the normalized path
    vector<Vector3d> local_path_normalized;
    Vector3d pos;
    pos = path_simple[0]; 
    local_path_normalized.push_back(pos);
    int simple_size = path_simple.size();
    for (int i=1; i<simple_size; i++){
        Vector3d pos_L, pos_R,direction;
        pos_L = path_simple[i-1];
        pos_R = path_simple[i];
        direction = pos_R - pos_L;
        double dist = direction.norm();
        int num = round(dist / resolution_);
        double delta_x, delta_y, delta_z;
        delta_x = direction(0) / num;
        delta_y = direction(1) / num;
        delta_z = direction(2) / num;
        Vector3d delta;
        delta << delta_x, delta_y, delta_z; 
        for (int i=0; i<num; i++){
            Vector3d pos;
            pos = pos_L + (i+1) * delta;
            local_path_normalized.push_back(pos);
        }
        local_path_normalized.push_back(pos_R);      
    }

    // get the local path
    Vector3d start = local_path_normalized[0];
    Vector3d local_min, local_max;
    vector<Vector3d> local_path;
    local_min(0) = start(0) - local_margin_;
    local_min(1) = start(1) - local_margin_;
    local_min(2) = 0.0;
    local_max(0) = start(0) + local_margin_;
    local_max(1) = start(1) + local_margin_;
    local_max(2) = 3.0;

    int num = local_path_normalized.size();
    local_path.push_back(local_path_normalized[0]);
    for (int i = 1; i < num ; i++){
        Vector3d pos = local_path_normalized[i];
        if ( pos(0) < local_min(0) || pos(0) > local_max(0) ||
             pos(1) < local_min(1) || pos(1) > local_max(1) ||
             pos(2) < local_min(2) || pos(2) > local_max(2) )
        {
            break;
        }else
        {
            local_path.push_back(pos);
        }
    }
    
    return local_path;

    // get the local path
    // Vector3d start = path[0];
    // Vector3d local_min, local_max;
    // vector<Vector3d> local_path;
    // local_min(0) = start(0) - local_margin_;
    // local_min(1) = start(1) - local_margin_;
    // local_min(2) = 0.0;
    // local_max(0) = start(0) + local_margin_;
    // local_max(1) = start(1) + local_margin_;
    // local_max(2) = 3.0;

    // int num = path.size();
    // local_path.push_back(path[0]);
    // for (int i = 1; i < num ; i++){
    //     Vector3d pos = path[i];
    //     if ( pos(0) < local_min(0) || pos(0) > local_max(0) ||
    //          pos(1) < local_min(1) || pos(1) > local_max(1) ||
    //          pos(2) < local_min(2) || pos(2) > local_max(2) )
    //     {
    //         break;
    //     }else
    //     {
    //         local_path.push_back(pos);
    //     }
    // }
    // // cout <<"get local path , the local path size : " << local_path.size() << endl;

    // // simplify the local path
    // vector<Vector3d> local_path_simple;
    // local_path_simple = pathSimplify(local_path);
    // // cout <<"get local simple path , the local simple path size : " << local_path_simple.size() << endl;
    

    // // get the normalized path
    // vector<Vector3d> local_path_normalized;
    // Vector3d pos;
    // pos = local_path_simple[0]; 
    // local_path_normalized.push_back(pos);
    // int simple_size = local_path_simple.size();
    // for (int i=1; i<simple_size; i++){
    //     Vector3d pos_L, pos_R,direction;
    //     pos_L = local_path_simple[i-1];
    //     pos_R = local_path_simple[i];
    //     direction = pos_R - pos_L;
    //     double dist = direction.norm();
    //     int num = round(dist / resolution_);
    //     double delta_x, delta_y, delta_z;
    //     delta_x = direction(0) / num;
    //     delta_y = direction(1) / num;
    //     delta_z = direction(2) / num;
    //     Vector3d delta;
    //     delta << delta_x, delta_y, delta_z; 
    //     for (int i=0; i<num; i++){
    //         Vector3d pos;
    //         pos = pos_L + (i+1) * delta;
    //         local_path_normalized.push_back(pos);
    //     }
    //     local_path_normalized.push_back(pos_R);      
    // }

    // return local_path_normalized;
} 

vector<Vector3d> Astar::pathSimplify(const vector<Vector3d> &path)
{
    vector<Vector3d> subPath;
    int path_nodes = path.size();
    if (path_nodes == 0){
        return path;
    }
    // int flag = 1;
    int flag = 0;
    Vector3d cut_start = path[0];
    subPath.push_back(cut_start);

    while(flag < (path_nodes-1)){
        // for (int i = flag;i < path_nodes ; i++){
        for (int i = flag+1;i < path_nodes ; i++){
            bool isSafe = true;
            Vector3d check_pt = path[i];
            Vector3d delta_pt = check_pt-cut_start ; 
            int check_num = ceil(delta_pt.norm() / resolution_);
            for (int j = 0;j <= check_num; j++){
                Vector3d check_safe_pt;
                double t = 1.0/check_num*j;
                check_safe_pt = (1-t) * cut_start + t * check_pt;

                if (isOccupied(coord2gridIndex(check_safe_pt)))
                {
                    isSafe = false;
                    break;
                }
            }
            
            if (isSafe && i == (path_nodes-1))
            {
                flag = i;
                subPath.push_back(check_pt);
            }
            
            if (isSafe){
                continue;
            } 
            // else if (flag == i){
            //     // error situation
            //     flag = path_nodes;
            //     break;
            // }
            else {
                flag = i;
                cut_start = path[flag-1];
                subPath.push_back(cut_start);
                break;
            }
        }             
    }
    return subPath;
}

vector<Vector3d> Astar::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1 && GridNodeMap[i][j][k]->rounds == rounds_)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    // ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    cout << "visited_nodes size : " << visited_nodes.size() << endl;
    return visited_nodes;
}

void Astar::visVisitedNode(){
    vector<Vector3d> nodes = getVisitedNodes();
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
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    visited_nodes_vis_pub_.publish(node_vis);
}

}  // namespace adaptive_planner
