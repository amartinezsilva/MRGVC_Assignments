#include <pluginlib/class_list_macros.h>
#include "AROB_lab5/rrt_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace rrt_planner {

double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}

RRTPlanner::RRTPlanner() : costmap_ros_(NULL), initialized_(false),
                            max_samples_(0.0){}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    if (!initialized_){
        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle nh_local("~/local_costmap/");
        ros::NodeHandle nh_global("~/global_costmap/");

        nh.param("maxsamples", max_samples_, 0.0);

        //to make sure one of the nodes in the plan lies in the local costmap
        double width, height;
        nh_local.param("width", width, 3.0);
        nh_local.param("height", height, 3.0);
        max_dist_ = (std::min(width, height)/6.0);  //or any other distance within local costmap
        nh_global.param("resolution", resolution_, 0.05);

        cell_width_ = int(16.0/resolution_);
        cell_height_ = int(16.0/resolution_);


        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros_->getGlobalFrameID();

        initialized_ = true;
    }
	else{
	    ROS_WARN("This planner has already been initialized... doing nothing.");
    }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan ){

    std::cout << "RRTPlanner::makePlan" << std::endl;
    
    if (!initialized_){
        ROS_ERROR("The planner has not been initialized.");
        return false;
    }

	if (start.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The start pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), start.header.frame_id.c_str());
		return false;
	}

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The goal pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), goal.header.frame_id.c_str());
		return false;
	}
    
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();  // Update information from costmap
    
    // Get start and goal poses in map coordinates
    unsigned int goal_mx, goal_my, start_mx, start_my;
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)){
        ROS_WARN("Goal position is out of map bounds.");
        return false;
    }    
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);

    std::vector<int> point_start{(int)start_mx,(int)start_my};
    std::vector<int> point_goal{(int)goal_mx,(int)goal_my};    
  	std::vector<std::vector<int>> solRRT;
    bool computed = computeRRT(point_start, point_goal, solRRT);
    if (computed){        
        getPlan(solRRT, plan);
        // add goal
        plan.push_back(goal);
    }else{
        ROS_WARN("No plan computed");
    }

    return computed;
}

bool RRTPlanner::computeRRT(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol){
    bool finished = false;
    int maxIterations = 40000;
    int count = 0;
    //Initialize random number generator
    srand(time(NULL));
        
    std::cout << "Start x: " << start[0] << "  y: " << start[1] << "\n";
    std::cout << "Goal x: " << goal[0] << "  y: " << goal[1] << "\n";

    
    // Initialize the tree with the starting point in map coordinates (root)
    TreeNode *itr_node = new TreeNode(start); 

    int max_cell_dist = int(max_dist_ / resolution_);

    std::cout << "Max distance in cells: " << max_cell_dist << "\n";

    std::vector<std::vector<int>> evaluated_points;

    //While goal is not reached
    
    while(!finished && count < maxIterations){
        
        int x_samp = int(rand() % cell_width_);
        int y_samp = int(rand() % cell_height_);
        std::vector<int> x_rand_point{(int)x_samp,(int)y_samp};
        
        TreeNode *random_node = new TreeNode(x_rand_point);
        TreeNode *x_near_node = random_node -> neast(itr_node);

        //x_near_node -> setParent(itr_node);
        std::vector<int> x_near_point = x_near_node->getNode();

        double ev_distance = distance(x_rand_point[0], x_rand_point[1], x_near_point[0], x_near_point[1]);

        std::vector<int> x_new_point = x_rand_point;

        if(ev_distance > max_cell_dist){
            double theta = atan2((x_rand_point[1] - x_near_point[1]),(x_rand_point[0] - x_near_point[0]));
            x_new_point[0] = x_near_point[0] + max_cell_dist*cos(theta);
            x_new_point[1] = x_near_point[1] + max_cell_dist*sin(theta);
        }

        if(std::find(evaluated_points.begin(), evaluated_points.end(), x_new_point) != evaluated_points.end()) {
            /* v contains x */
            std::cout<< "x_new already evaluated"<<std::endl;
            continue;
        } else {
            /* v does not contain x */
            evaluated_points.push_back(x_new_point);
        }


        TreeNode *x_new_node = new TreeNode(x_new_point);

        if(obstacleFree(x_near_point[0], x_near_point[1], x_new_point[0], x_new_point[1])){
            //Debug
            std::cout << "Sampled node x_rand: " << "\n";
            random_node->printNode();
            std::cout << "Nearest node in tree x_near: " << "\n";
            x_near_node -> printNode();
            x_near_node -> appendChild(x_new_node);
            std::cout << "Adding node xnew to the tree: " << "\n";
            x_new_node->printNode();
            //itr_node = x_new_node;

            ev_distance = distance(x_new_point[0], x_new_point[1], goal[0], goal[1]);
            std::cout << "Iteration: " << count << ", Distance to goal: " << ev_distance << std::endl; 
            if(obstacleFree(x_new_point[0], x_new_point[1], goal[0], goal[1]) && ev_distance <= max_cell_dist){
                sol = x_new_node->returnSolution();
                finished = true;
                std::cout << "finished plan!" << std::endl;
            }
        }

        //std::cin.get(); 
    
        count++;
    }

    //sol = itr_node->returnSolution();

    ROS_WARN("Finished after %d iterations", count);

    itr_node->~TreeNode();

    return finished;
}

bool RRTPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
                            const unsigned int x1, const unsigned int y1){
    //Bresenham algorithm to check if the line between points (x0,y0) - (x1,y1) is free of collision

    int dx = x1 - x0;
    int dy = y1 - y0;

    int incr_x = (dx > 0) ? 1.0 : -1.0;
    int incr_y = (dy > 0) ? 1.0 : -1.0;

    unsigned int da, db, incr_x_2, incr_y_2;
    if (abs(dx) >= abs(dy)){
        da = abs(dx); db = abs(dy);
        incr_x_2 = incr_x; incr_y_2 = 0;
    }else{
        da = abs(dy); db = abs(dx);
        incr_x_2 = 0; incr_y_2 = incr_y;
    }

    int p = 2*db - da;
    unsigned int a = x0; 
    unsigned int b = y0;
    unsigned int end = da;
    for (unsigned int i=0; i<end; i++){
        if (costmap_->getCost(a, b) != costmap_2d::FREE_SPACE){  // to include cells with inflated cost
            return false;
        }else{
            if (p >= 0){
                a += incr_x;
                b += incr_y;
                p -= 2*da;
            }else{
                a += incr_x_2;
                b += incr_y_2;
            }
            p += 2*db;
        }
    }

    return true;
}

void RRTPlanner::getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan){

    for (auto it = sol.rbegin(); it != sol.rend(); it++){
        std::vector<int> point = (*it);
        geometry_msgs::PoseStamped pose;

        costmap_->mapToWorld((unsigned int)point[0], (unsigned int)point[1], 
                            pose.pose.position.x, pose.pose.position.y);
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_id_;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
        //std::cout << "Wp x: " << pose.pose.position.x << " y: " << pose.pose.position.y << std::endl;
    }
}

};
