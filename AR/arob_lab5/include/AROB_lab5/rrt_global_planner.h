/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>

#include <time.h> 
#include <random>
#include "AROB_lab5/TreeNode.h"

#ifndef RRT_PLANNER_CPP
#define RRT_PLANNER_CPP

namespace rrt_planner {

class RRTPlanner : public nav_core::BaseGlobalPlanner {
    
public:

    RRTPlanner();
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // overridden classes from interface nav_core::BaseGlobalPlanner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

private:
    ros::Publisher vis_pub_;

    costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
    std::string global_frame_id_;
	bool initialized_;

    double max_samples_;
    int cell_width_, cell_height_;

    double max_dist_;
    double resolution_;

    // functions to compute the plan
    bool obstacleFree(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1);
    bool computeRRT(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol);
    void getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);

};

};
 #endif
