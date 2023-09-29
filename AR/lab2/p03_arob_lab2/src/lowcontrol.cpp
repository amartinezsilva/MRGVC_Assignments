#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <stdio.h> 
#include <math.h>
#include <fstream>
#include <tf/transform_broadcaster.h>

using namespace std;


class Lowlevelcontrol {
	ros::NodeHandle nh_;
	ros::Publisher velocity_pub_;
	ros::Subscriber position_sub_;
	ros::Subscriber goal_sub_;
	geometry_msgs::PoseStamped Goal;
	float krho, kalpha, kbeta;
public:
	Lowlevelcontrol() {

		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &Lowlevelcontrol::positionCb, this);
		goal_sub_ = nh_.subscribe("/goal", 1, &Lowlevelcontrol::goalCb, this);
		velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		
		// Goal.pose.position.x = -6.0; //update the initial values of these variables
		// Goal.pose.position.y = -6.0;
		Goal.pose.position.x = 0.0; //update the initial values of these variables
		Goal.pose.position.y = 0.0;
		
		kalpha = 2.0; // the values of these parameters should be obtained by you alpha = 4.0 rho = 1.0 beta = -0.25
		krho = 0.1;
		kbeta = -0.01;
	}

	~Lowlevelcontrol() {
	}

	void goalCb(const geometry_msgs::PoseStamped& msg) {

		std::cout << " Goal Update: "<< msg.pose.position.x << endl;
		std::cout << " Goal Update: "<< msg.pose.position.y << endl;
	//	upadte the goal
		Goal.pose.position.x = msg.pose.position.x;
		Goal.pose.position.y = msg.pose.position.y;
	}

	void positionCb(const nav_msgs::Odometry& msg) {

		
		float ex = Goal.pose.position.x - msg.pose.pose.position.x;
		float ey = Goal.pose.position.y - msg.pose.pose.position.y;
		float rho = sqrt(ex*ex+ey*ey);
		float alpha = atan2(ey,ex) - tf::getYaw(msg.pose.pose.orientation);
		if (alpha < -M_PI) alpha = 2*M_PI - abs(alpha);
		if (alpha > M_PI) alpha = -2*M_PI + alpha;
		float beta = - alpha - tf::getYaw(msg.pose.pose.orientation);
		if (beta < -M_PI) beta = 2*M_PI - abs(beta);
		if (beta > M_PI) beta = -2*M_PI + beta;

		std::cout << "ex: "<< ex << " ";
		std::cout << "ey: "<< ey << " ";

		std::cout << "Rho: "<< rho << " ";
		std::cout << "Alpha: "<< alpha << " ";
		std::cout << "Beta: "<< beta << endl;

		std::cout << "X: "<< msg.pose.pose.position.x << " ";
		std::cout << "Y: "<< msg.pose.pose.position.y << " ";
		std::cout << "Th: "<< tf::getYaw(msg.pose.pose.orientation) << endl;

		geometry_msgs::Twist input; //to send the velocities

		//here you have to implement the controller

		if(rho < 0.2){
			input.linear.x = 0;
			input.angular.z = 0;
			return;
		}

		input.linear.x = krho * rho;
		input.angular.z = kalpha * alpha + kbeta * beta;

		std::cout << "vx: "<< input.linear.x << " ";
		std::cout << "wz: "<< input.angular.z << endl;


		velocity_pub_.publish(input);
	}

	
};

int main(int argc, char** argv) {


	ros::init(argc, argv, "lowcontrol");
	ros::NodeHandle nh("~");
	Lowlevelcontrol llc;

	ros::spin();
	return 0;
}
