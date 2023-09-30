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
	ros::Publisher goal_pub_;
	ros::Subscriber position_sub_;
	geometry_msgs::PoseStamped Goal;
        ifstream inFile;
	std::vector<std::vector<float> > targets;
	int currentTarget; //index with the next target to reach

	std::string pkg_path = ros::package::getPath("p03_arob_lab2");

	//variables to read the txt
	std::vector<float> inputTarget = {0,0}; //auxiliar vector to read each target
	std::stringstream stringStream;
	float charFound;
	std::string temp;

	//variables to read vectors
	int targetsNumber = 0;
	int vectorIndex = 0;
	int targetsIndex = 0;


public:
	FollowTargetsClass() { 
		
		//in the contructor you can read the targets from the text file
		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &FollowTargetsClass::positionCb, this);
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);

		inFile.open(pkg_path + "/src/targets.txt");
		if (inFile.is_open()){
			std::cout << "File is open."<<std::endl;
			while(std::getline(inFile, temp,';')) {
				if(std::stringstream(temp) >> charFound)
				{
					if(vectorIndex < 1) {
						inputTarget[vectorIndex] = charFound;
						vectorIndex = vectorIndex + 1;
					} else {
						inputTarget[vectorIndex] = charFound;
						vectorIndex = 0;
						targets.push_back(inputTarget);
						targetsNumber = targetsNumber + 1;
					}
				}
			}
			inFile.close();
		} else std::cout << "Unable to open the file"; 
	}

	~FollowTargetsClass() {
	}

	//complete the class by adding the functio that you need

	void positionCb(const nav_msgs::Odometry& msg) {
		float ex = targets[targetsIndex][0] - msg.pose.pose.position.x;
		float ey = targets[targetsIndex][1]  - msg.pose.pose.position.y;
		float rho = sqrt(ex*ex+ey*ey);

		if(rho <= 0.1 && targetsIndex < targetsNumber-1) {
			targetsIndex = targetsIndex + 1;
		}

		Goal.pose.position.x = targets[targetsIndex][0]; 
		Goal.pose.position.y = targets[targetsIndex][1]; 
		Goal.pose.position.z = 0;
		Goal.pose.orientation.w = 0;
		goal_pub_.publish(Goal);
	}
};


int main(int argc, char** argv) {

	ros::init(argc, argv, "followTargets");
	ros::NodeHandle nh("~");
	FollowTargetsClass FT;

	ros::spin();
	return 0;
}

