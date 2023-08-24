#ifndef ERL_COSTMAP_H
#define ERL_COSTMAP_H

#include <cstring>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
#include <vector>
#include <nav_msgs/OccupancyGrid.h> // occupancy grid msg
// #include <fstream>

namespace erl {

/**
 * @brief The CostMap2D class implements costmap a 2D Grid environment.
 */
struct CostMap2D {
    struct Setting {
        // hector map,
        // lower and upper bounds of obstacle characterization, and unknown grid value
        int8_t kUnknown = -1;
        int8_t kObsLb = 80;
        int8_t kObsUb = 100;
        
        // clearpath jackal robot
        double kCostRadius1 = 0.430 / 2.0; // insribed radius

        // a hack for shortest path, gamma >= 15.0, will trigger this special hack
        // circumscribed radius round up to map resolution 0.33-> 0.4,
        // this kCostRadius2 is like inflation for binary A*
        double kCostRadius2 = 0.4; 


        double kCostGamma = 7.0;
        
        int8_t kCostUnknown = 3;
        int8_t kCostLethal = 19;
        int8_t kCostInscribed = 18;
        
        // JACKAL_CIRCUMSCRIBED_RADIUS + 0.1 = 0.354

        // int8_t kCostCutoff = 10; // 0.30
        // int8_t kCostCutoff = 9; // 0.31
        // int8_t kCostCutoff = 8; // 0.33
        // int8_t kCostCutoff = 7; // 0.35 meter
        int8_t kCostCutoff = 6; // 0.37 meter

        int8_t kVisUnknown = -1;
        int8_t kVisLethal = 100;
        int8_t kVisInscribed = 90;
        int8_t kVisCutoff = -3;
        int8_t kVisZero = 110;
    };

    // containers
    std::shared_ptr<Setting> setting = nullptr;
    std::vector<uint8_t> pos_bw_vec = {};
    std::vector<uint8_t> neg_bw_vec = {};
    std::vector<int8_t> planning_costmap_data = {};
    std::vector<int8_t> rviz_costmap_data = {};

    // constructor
    explicit CostMap2D(std::shared_ptr<Setting> in_setting): setting(std::move(in_setting)) {} 

    // Given ros occupancy grid, compute costmap using cv2
    void compute(const nav_msgs::OccupancyGrid::ConstPtr &gridmap_msg, cv::Mat &dist_mat_32f) {
        auto gm_data_1d_ptr = gridmap_msg->data.data();
        int map_width = gridmap_msg->info.width;
        int map_height = gridmap_msg->info.height;

        int n_elements = map_width * map_height;
        double map_resolution = gridmap_msg->info.resolution;
        // double kRadius0 = map_resolution / 2.0;

        // note that occupancy grid map is row-major, cv Mat is row-major, c++ is stl row-major
        // init 1d vector containers
        if (planning_costmap_data.empty()) {
            pos_bw_vec.resize(n_elements);
            neg_bw_vec.resize(n_elements);
            planning_costmap_data.resize(n_elements);
            rviz_costmap_data.resize(n_elements);
        }

        // put hector map to binary map, 0 value for obstacle, 255 for non obstacle
        for (int i = 0; i < n_elements; ++i) {
            if (gm_data_1d_ptr[i] >= setting->kObsLb && gm_data_1d_ptr[i] <= setting->kObsUb) {
                pos_bw_vec[i] = 0;
                neg_bw_vec[i] = 255;
            } else {
                pos_bw_vec[i] = 255;
                neg_bw_vec[i] = 0;
            }
        }

        // wrap 1d binary container in 2d cv Mat for further computation without copying the data
        cv::Mat pos_bw_mat_8u(map_height, map_width, CV_8UC1, pos_bw_vec.data());
        cv::Mat neg_bw_mat_8u(map_height, map_width, CV_8UC1, neg_bw_vec.data());
        // be cautious, cv::DIST_L2 return type is CV_32F
        cv::Mat pos_dist_mat_32f;
        cv::Mat neg_dist_mat_32f;
        // https://docs.opencv.org/3.4/d7/d1b/group__imgproc__misc.html#ga8a0b7fdfcb7a13dde018988ba3a43042
        cv::distanceTransform(pos_bw_mat_8u, pos_dist_mat_32f, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        cv::distanceTransform(neg_bw_mat_8u, neg_dist_mat_32f, cv::DIST_L2, cv::DIST_MASK_PRECISE);

        dist_mat_32f = pos_dist_mat_32f - neg_dist_mat_32f;
        dist_mat_32f *= static_cast<float>(map_resolution);

        // assign costmap value by distance map, handle unknown type separately
        for (int i = 0; i < n_elements; ++i) {
            if ((gm_data_1d_ptr[i]) == setting->kUnknown) {
                planning_costmap_data[i] = setting->kCostUnknown;
                rviz_costmap_data[i] = setting->kVisUnknown;
            } else{
              assignCostmapValueByDistance(dist_mat_32f, map_resolution, i);
            }
        }
    }


    // for known type cell, we assign costmap cell value by distance to obstacle
    void assignCostmapValueByDistance(const cv::Mat &dist_mat, double map_res, int i){
      double dist_metric = dist_mat.at<float>(i) * map_res;

      // real obstacle
      if (dist_metric <= map_res) {
          planning_costmap_data[i] = setting->kCostLethal;
          rviz_costmap_data[i] = setting->kVisUnknown;
      } else if (dist_metric <= setting->kCostRadius1) {
          planning_costmap_data[i] = setting->kCostInscribed;
          rviz_costmap_data[i] = setting->kVisInscribed;
      } else {
          double cell_cost = 0.0;
          // special hack to achieve shortest path with min clearance for Jackal 0.4 circumscribed radius
          if (setting->kCostGamma >= 15.0){
            cell_cost = 1.0 * setting->kCostInscribed - 1.0;
            if (dist_metric > setting->kCostRadius2) cell_cost = 0.0;
          }else{
            // using C-style convert
            cell_cost = double(setting->kCostInscribed) * std::exp(-setting->kCostGamma * (dist_metric - setting->kCostRadius1));
          }

          planning_costmap_data[i] = int8_t(std::round(cell_cost));


          if (planning_costmap_data[i] == 0){
            rviz_costmap_data[i] = setting->kVisZero;
          // due to map resolution limitation, to show inflated planning boundary in [cutoff, cutoff+2]
          }else if (planning_costmap_data[i] >= setting->kCostCutoff && planning_costmap_data[i] <= setting->kCostCutoff + 2){
            rviz_costmap_data[i] = setting->kVisCutoff;
          }else{
            // be cautious, the final result should less than 127, in our case, the max is kCostInscribed * 5 = 19 * 5 = 95
            rviz_costmap_data[i] = planning_costmap_data[i] * 5;
          }
      } // end for non-real obstacle
    }

}; // end of class
} // end of erl namespace

#endif