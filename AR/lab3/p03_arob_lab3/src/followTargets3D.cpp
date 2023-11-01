#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <ros/package.h>

using namespace std;

class FollowTargetsClass {

	ros::NodeHandle nh_;
	ros::Subscriber position_sub_;
	ros::Publisher goal_pub_;
	geometry_msgs::PoseStamped Goal;
    ifstream inFile;
	std::vector<std::vector<float> > targets;
	int currentTarget = 0; //index with the next target to reach
	float distance_to_target;
	float minimum_distance = 0.25;

	std::string pkg_path = ros::package::getPath("p03_arob_lab3");
	std::string targets_file;

public:

	FollowTargetsClass() { //in the contructor you can read the targets from the text file

    	nh_.getParam("targets_file", targets_file);
		std::cout << "Reading targets file: " << targets_file << '\n';

		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/command/pose", 1, true);
		position_sub_ = nh_.subscribe("/ground_truth_to_tf/pose", 1, &FollowTargetsClass::positionCb, this);

		inFile.open(pkg_path + "/src/" + targets_file + ".txt");

		std::string target_string;
		if ( inFile.is_open() ) {
			while ( std::getline (inFile, target_string) ) { // equivalent to myfile.good()
				std::cout << "Read point: " << target_string << '\n';

				std::vector<float> target;

				std::istringstream ss(target_string);
				std::string token;

				while(std::getline(ss, token, ';')) {
					target.push_back(stof(token));
				}

				targets.push_back(target);
				
			}
			
			inFile.close();

		}
		else {
			std::cout << "Couldn't open file\n";
		}

		//Publish first goal
		std::vector<float> first_target = targets.front();
		Goal.pose.position.x = first_target.at(0);
		Goal.pose.position.y = first_target.at(1);
		Goal.pose.position.z = first_target.at(2);

		goal_pub_.publish(Goal);
		std::cout << "Publishing first goal x -> " << Goal.pose.position.x << " y -> " << Goal.pose.position.y  << " z -> " << Goal.pose.position.z << '\n';


	}

	~FollowTargetsClass() {
	}

	//complete the class by adding the functio that you need

	void positionCb(const geometry_msgs::PoseStamped& msg) {

		float ex = msg.pose.position.x - Goal.pose.position.x;
		float ey = msg.pose.position.y - Goal.pose.position.y;
		float ez = msg.pose.position.z - Goal.pose.position.z;

		distance_to_target = sqrt(ex*ex+ey*ey);

		if (distance_to_target < minimum_distance){

			if (currentTarget == targets.size() - 1){
				ROS_INFO_STREAM_ONCE("Final goal reached!");
				return;
			}
	
			currentTarget++;
			
			//fetch new target
			std::vector<float> target = targets.at(currentTarget);
			
			//Publish new goal
			Goal.pose.position.x = target.at(0);
			Goal.pose.position.y = target.at(1);
			Goal.pose.position.z = target.at(2);

			goal_pub_.publish(Goal);

			std::cout << "Publishing x -> " << Goal.pose.position.x << " y -> " << Goal.pose.position.y << " z -> " << Goal.pose.position.z << '\n';

		}

	}


};


int main(int argc, char** argv) {


	ros::init(argc, argv, "followTargets");
	ros::NodeHandle nh("~");

	FollowTargetsClass FT;

	ros::spin();
	return 0;
}

