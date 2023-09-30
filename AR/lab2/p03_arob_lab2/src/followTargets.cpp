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

	std::string pkg_path = ros::package::getPath("p03_arob_lab2");


public:

	FollowTargetsClass() { //in the contructor you can read the targets from the text file

		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 1, true);
		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &FollowTargetsClass::positionCb, this);

		inFile.open(pkg_path + "/src/targets.txt");

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
		goal_pub_.publish(Goal);
		std::cout << "Publishing first goal x -> " << Goal.pose.position.x << " y -> " << Goal.pose.position.y << '\n';


	}

	~FollowTargetsClass() {
	}

	//complete the class by adding the functio that you need

	void positionCb(const nav_msgs::Odometry& msg) {

		float ex = msg.pose.pose.position.x - Goal.pose.position.x;
		float ey = msg.pose.pose.position.y - Goal.pose.position.y;
		distance_to_target = sqrt(ex*ex+ey*ey);

		if (distance_to_target < minimum_distance){

			if (currentTarget == targets.size() - 1){
				cout << "Final goal reached!" << '\n';
				return;
			}
	
			currentTarget++;
			
			//fetch new target
			std::vector<float> target = targets.at(currentTarget);
			
			//Publish new goal
			Goal.pose.position.x = target.at(0);
			Goal.pose.position.y = target.at(1);
			goal_pub_.publish(Goal);

			std::cout << "Publishing x -> " << Goal.pose.position.x << " y -> " << Goal.pose.position.y << '\n';

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

