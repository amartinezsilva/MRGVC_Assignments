/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/tilting_mc_desired_angles.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>


#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {

	float min_interp_distance_ = 0.05;
	float minimum_distance_ = 0.25;
	float look_ahead_ = 0.5;
	int pos_look_ahead_ = 0;
	float c1_ = 0.6321, c2_ = 0.3679;
	std::vector<std::vector<double>> Goal_, prevGoal_;
	std::vector<std::vector<double>> targets_;
	std::ifstream inFile;
	Eigen::Vector3f current_point_;
	int currentTarget = 0; //index with the next target to reach
	int prevTarget = 0;
	TrajectorySetpoint prev_setpoint_{};
	TrajectorySetpoint prev_filtered_setpoint_{};
	TiltingMcDesiredAngles prev_tilt_setpoint_{};
	TiltingMcDesiredAngles prev_filtered_tilt_setpoint_{};


public:
	OffboardControl(const std::string &targets_file) : Node("offboard_control") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
		tilt_angle_publisher_ =
			this->create_publisher<TiltingMcDesiredAngles>("/fmu/tilting_mc_desired_angles/in", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
		tilt_angle_publisher_ =
			this->create_publisher<TiltingMcDesiredAngles>("/fmu/tilting_mc_desired_angles/in");

#endif

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});
		// get odometry and compute errors
		odometry_sub_ = 
			this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/vehicle_odometry/out", 10,
				[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
					
					//std::cout << "x: " << msg->x << " y: " << msg->y << " z: " << msg->z  << std::endl;
					// std::cout << "vx: " << msg->vx << " vy: " << msg->vy << " vz: " << msg->vz  << std::endl;
		
					// float q[4];
					// for (int i = 0; i < 4; ++i) {
					// 	q[i] = msg->q[i];
					// }
					// float roll, pitch, yaw;
					// quaternionToEulerAngles(q, roll, pitch, yaw);
					// float roll_degrees, pitch_degrees, yaw_degrees;
					// roll_degrees = radiansToDegrees(roll);
					// pitch_degrees = radiansToDegrees(pitch);
					// yaw_degrees = radiansToDegrees(yaw);

					// std::cout << "roll(º): " << roll_degrees
					// << "pitch(º): " << pitch_degrees
					// << "yaw(º): " << yaw_degrees  << std::endl;

					//Update control
					current_point_ = Eigen::Vector3f(msg->x, msg->y, msg->z);

				});

		//Read targets file

		this->declare_parameter("targets_file", targets_file);
        std::string targets_file_param = this->get_parameter("targets_file").as_string();
        std::cout << "Reading targets file: " << targets_file_param << '\n';

        //std::string pkg_path = ament_index_cpp::get_package_share_directory("px4_ros_com");
		std::string pkg_path = "/home/adren/px4_ros_com_ros2/src/px4_ros_com";
        inFile.open(pkg_path + "/src/examples/offboard_tilted/" + targets_file_param + ".txt");
        std::cout << "Path: " << pkg_path + "/src/" + targets_file_param + ".txt" << '\n';

		std::string target_string;
		std::vector<double> list_x, list_y, list_z, list_r, list_p, list_yaw;
		if ( inFile.is_open() ) {
			while ( std::getline (inFile, target_string) ) { // equivalent to myfile.good()
				std::cout << "Read point: " << target_string << '\n';

				std::vector<double> target;

				std::istringstream ss(target_string);
				std::string token;

				while(std::getline(ss, token, ';')) {
                    target.push_back(std::atof(token.c_str()));
				}

				list_x.push_back(target.at(0));
				list_y.push_back(target.at(1));
				list_z.push_back(target.at(2));
				list_r.push_back(target.at(3));
				list_p.push_back(target.at(4));
				list_yaw.push_back(target.at(5));

				//targets_.push_back(target);
				
			}
			
			inFile.close();

		}
		else {
			std::cout << "Couldn't open file\n";
		}

		// Calculate total distance
        int total_distance = 0;
        for (int i = 0; i < list_x.size() - 1; i++) {
			Eigen::Vector3f point_1, point_2;
            point_1 = Eigen::Vector3f(list_x[i], list_y[i], list_z[i]);
            point_2 = Eigen::Vector3f(list_x[i + 1], list_y[i + 1], list_z[i + 1]);
            total_distance = total_distance + (point_2 - point_1).norm();
        }

		//Perform interpolation
		int new_path_size = int(total_distance/min_interp_distance_);
		std::cout << "Requesting interpolation of: " << new_path_size << " waypoints" << std::endl;

		std::vector<double> x_interp =  interpWaypointList(list_x, new_path_size);
		std::vector<double> y_interp =  interpWaypointList(list_y, new_path_size);
		std::vector<double> z_interp =  interpWaypointList(list_z, new_path_size);
		std::vector<double> r_interp =  interpWaypointList(list_r, new_path_size);
		std::vector<double> p_interp =  interpWaypointList(list_p, new_path_size);
		std::vector<double> yaw_interp =  interpWaypointList(list_yaw, new_path_size);

		//DEBUG: write interpolated trajectory to a .txt
		// Open the file for writing
		// Specify the filename
    	std::string output_filename = "interpolated_trajectory.csv";
		std::ofstream outputFile(output_filename);

		// Check if the file is open
		if (!outputFile.is_open()) {
			std::cerr << "Error opening the file: " << output_filename << std::endl;
			return;
		}

		// Write column headers
    	outputFile << "x,y,z,roll,pitch,yaw" << std::endl;
		// Write the vectors to the file
		for (size_t i = 0; i < x_interp.size(); ++i) {
			outputFile << x_interp[i] << "," << y_interp[i] << "," << z_interp[i] << ","
					<< r_interp[i] << "," << p_interp[i] << "," << yaw_interp[i] << std::endl;

			std::vector<double> target;
			target.push_back(x_interp[i]);
			target.push_back(y_interp[i]);
			target.push_back(z_interp[i]);
			target.push_back(r_interp[i]);
			target.push_back(p_interp[i]);
			target.push_back(yaw_interp[i]);
			targets_.push_back(target);

		}

		// Close the file
		outputFile.close();

		std::cout << "Interpolated trajectory wrote to " << output_filename << std::endl;

        
        std::vector<double> first_target = targets_.front();
        Goal_.resize(2, std::vector<double>(3, 0.0));

        Goal_[0][0] = first_target.at(0);
        Goal_[0][1] = first_target.at(1);
        Goal_[0][2] = first_target.at(2);
        Goal_[1][0] = first_target.at(3);
        Goal_[1][1] = first_target.at(4);
        Goal_[1][2] = first_target.at(5);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            		// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
			publish_tilt_setpoint();

           		 // stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm() const;
	void disarm() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<TiltingMcDesiredAngles>::SharedPtr tilt_angle_publisher_;

	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;


	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	float_t distance_to_goal_;

	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint() ;
	void publish_tilt_setpoint() ;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
	//angle conversion functions
	void quaternionToEulerAngles(const float q[4], float& roll, float& pitch, float& yaw) const;
    void eulerAnglesToQuaternion(float roll, float pitch, float yaw, float q[4]) const;
    float degreesToRadians(const float degrees) const;
    float radiansToDegrees(const float radians) const;
	// interpolation functions
	int nearestNeighbourIndex(std::vector<double> &_x, double &_value) const;
	std::vector<double> linealInterp1(std::vector<double> &_x, std::vector<double> &_y, std::vector<double> &_x_new) const;
	std::vector<double> interpWaypointList(std::vector<double> &_list_pose_axis, int _amount_of_points) const;
	int calculatePosLookAhead(int _pos_on_path) const;
	int calculatePosOnPath(Eigen::Vector3f &_current_point, double _search_range, int _prev_normal_pos_on_path) const;
	int calculateDistanceOnPath(int _prev_normal_pos_on_path, double _meters) const;


};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);

}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint() {


	//Update control
	currentTarget = calculatePosOnPath(current_point_, 2.0, prevTarget);
	pos_look_ahead_ = calculatePosLookAhead(currentTarget);

	std::cout << "current pos index: " << currentTarget << "look ahead index: " << pos_look_ahead_ << std::endl;
	
	Eigen::Vector3f target_p, unit_vec;
	target_p = Eigen::Vector3f(targets_.at(pos_look_ahead_).at(0), targets_.at(pos_look_ahead_).at(1), targets_.at(pos_look_ahead_).at(2));
    //double distance = (target_p - current_point_).norm();

	unit_vec = target_p - current_point_;
	unit_vec = unit_vec / unit_vec.norm();
	float mod_vel = 0.5;

	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = NAN;
	msg.y = NAN;
	msg.z = NAN;
	msg.vx = unit_vec(0) * mod_vel;
	msg.vy = unit_vec(1) * mod_vel;
	msg.vz = unit_vec(2) * mod_vel;
	
	//msg.yaw = NAN; // [-PI:PI]
	msg.yaw = degreesToRadians(targets_.at(pos_look_ahead_).at(5)); // [-PI:PI]
	//trajectory_setpoint_publisher_->publish(msg);

	std::cout << "comm_vx: " << msg.vx << " comm_vy: " << msg.vy << " comm_vz: " << msg.vz  << std::endl;


	//First order filter
	TrajectorySetpoint filtered_msg = msg;

	filtered_msg.vx = c1_*prev_setpoint_.vx + c2_*prev_filtered_setpoint_.vx;
	filtered_msg.vy = c1_*prev_setpoint_.vy + c2_*prev_filtered_setpoint_.vy;
	filtered_msg.vz = c1_*prev_setpoint_.vz + c2_*prev_filtered_setpoint_.vz;
	filtered_msg.yaw = c1_*prev_setpoint_.yaw + c2_*prev_filtered_setpoint_.yaw;

	trajectory_setpoint_publisher_->publish(filtered_msg);
	prev_setpoint_ = msg;
	prev_filtered_setpoint_ = filtered_msg;
	
	prevTarget = currentTarget;

}

/**
 * @brief Publish a tilt angle setpoint
 * 
 */

void OffboardControl::publish_tilt_setpoint() {
	TiltingMcDesiredAngles msg{};
	msg.timestamp = timestamp_.load();

	std::vector<double> target = targets_.at(pos_look_ahead_);

	msg.roll_body = degreesToRadians(target.at(3));
	msg.pitch_body = degreesToRadians(target.at(4));

	//tilt_angle_publisher_->publish(msg);

	//First order filter
	TiltingMcDesiredAngles filtered_msg = msg;

	filtered_msg.roll_body = c1_*prev_tilt_setpoint_.roll_body + c2_*prev_filtered_tilt_setpoint_.roll_body;
	filtered_msg.pitch_body = c1_*prev_tilt_setpoint_.pitch_body + c2_*prev_filtered_tilt_setpoint_.pitch_body;

	tilt_angle_publisher_->publish(filtered_msg);
	prev_tilt_setpoint_ = msg;
	prev_filtered_tilt_setpoint_ = filtered_msg;

}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::quaternionToEulerAngles(const float q[4], float& roll, float& pitch, float& yaw) const
{
    // Quaternion to Euler angles conversion
    // Adapted from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    float sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    float siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void OffboardControl::eulerAnglesToQuaternion(float roll, float pitch, float yaw, float q[4]) const
{
    // Euler angles to Quaternion conversion
    // Adapted from: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    float cy = std::cos(yaw * 0.5);
    float sy = std::sin(yaw * 0.5);
    float cp = std::cos(pitch * 0.5);
    float sp = std::sin(pitch * 0.5);
    float cr = std::cos(roll * 0.5);
    float sr = std::sin(roll * 0.5);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

float OffboardControl::degreesToRadians(const float degrees) const
{
    return degrees * M_PI / 180.0;
}

float OffboardControl::radiansToDegrees(const float radians) const
{
    return radians * 180.0 / M_PI;
}




int OffboardControl::nearestNeighbourIndex(std::vector<double> &_x, double &_value) const
{
    double dist = std::numeric_limits<double>::max();
    double newDist = dist;
    size_t idx = 0;

    for (size_t i = 0; i < _x.size(); ++i) {
        newDist = std::abs(_value - _x[i]);
        if (newDist <= dist) {
            dist = newDist;
            idx = i;
        }
    }

    return idx;
}

std::vector<double> OffboardControl::linealInterp1(std::vector<double> &_x, std::vector<double> &_y, std::vector<double> &_x_new) const
 {
    std::vector<double> y_new;
    double dx, dy, m, b;
    size_t x_max_idx = _x.size() - 1;
    size_t x_new_size = _x_new.size();

    y_new.reserve(x_new_size);

    for (size_t i = 0; i < x_new_size; ++i) {
        size_t idx = nearestNeighbourIndex(_x, _x_new[i]);

        if (_x[idx] > _x_new[i]) {
            dx = idx > 0 ? (_x[idx] - _x[idx - 1]) : (_x[idx + 1] - _x[idx]);
            dy = idx > 0 ? (_y[idx] - _y[idx - 1]) : (_y[idx + 1] - _y[idx]);
        } else {
            dx = idx < x_max_idx ? (_x[idx + 1] - _x[idx]) : (_x[idx] - _x[idx - 1]);
            dy = idx < x_max_idx ? (_y[idx + 1] - _y[idx]) : (_y[idx] - _y[idx - 1]);
        }

        m = dy / dx;
        b = _y[idx] - _x[idx] * m;

        y_new.push_back(_x_new[i] * m + b);
    }

    return y_new;
}

std::vector<double> OffboardControl::interpWaypointList(std::vector<double> &_list_pose_axis, int _amount_of_points) const
{
    std::vector<double> aux_axis;
    std::vector<double> new_aux_axis;
    for (int i = 0; i < _list_pose_axis.size(); i++) {
        aux_axis.push_back(i);
    }
    double portion = (aux_axis.back() - aux_axis.front()) / (_amount_of_points);
    double new_pose = aux_axis.front();
    new_aux_axis.push_back(new_pose);
    for (int i = 1; i < _amount_of_points; i++) {
        new_pose = new_pose + portion;
        new_aux_axis.push_back(new_pose);
    }
    auto interp1_path = linealInterp1(aux_axis, _list_pose_axis, new_aux_axis);
    return interp1_path;
}


// nav_msgs::Path OffboardControl::createPathInterp1(std::vector<double> &_list_x, std::vector<double> &_list_y, std::vector<double> &_list_z, int _path_size, int _new_path_size) {
//     nav_msgs::Path interp1_path;
//     std::vector<double> interp1_list_x, interp1_list_y, interp1_list_z;
//     if (_path_size > 1) {
//         // Lineal interpolation
//         interp1_list_x = interpWaypointList(_list_x, _new_path_size);
//         interp1_list_y = interpWaypointList(_list_y, _new_path_size);
//         interp1_list_z = interpWaypointList(_list_z, _new_path_size);
//         // Construct path
//         interp1_path = constructPath(interp1_list_x, interp1_list_y, interp1_list_z);
//     }
//     return interp1_path;
// }

int OffboardControl::calculatePosLookAhead(int _pos_on_path) const {
    int pos_look_ahead;
    std::vector<double> vec_distances;
    double temp_dist = 0.0;
    for (_pos_on_path; _pos_on_path < targets_.size() - 1; _pos_on_path++) {
        Eigen::Vector3f p1 = Eigen::Vector3f(targets_.at(_pos_on_path).at(0),targets_.at(_pos_on_path).at(1), targets_.at(_pos_on_path).at(2));
        Eigen::Vector3f p2 = Eigen::Vector3f(targets_.at(_pos_on_path + 1).at(0), targets_.at(_pos_on_path + 1).at(1), targets_.at(_pos_on_path + 1).at(2));
        temp_dist = temp_dist + (p2 - p1).norm();
        if (temp_dist < look_ahead_) {
            pos_look_ahead = _pos_on_path;
        } else {
            _pos_on_path = targets_.size();
        }
    }

    return pos_look_ahead;
}

int OffboardControl::calculatePosOnPath(Eigen::Vector3f &_current_point, double _search_range, int _prev_normal_pos_on_path) const {
    std::vector<double> vec_distances;
    int start_search_pos_on_path = calculateDistanceOnPath(_prev_normal_pos_on_path, -_search_range);
    int end_search_pos_on_path = calculateDistanceOnPath(_prev_normal_pos_on_path, _search_range);
    for (int i = start_search_pos_on_path; i < end_search_pos_on_path; i++) {
        Eigen::Vector3f target_path_point;
        target_path_point = Eigen::Vector3f(targets_.at(i).at(0), targets_.at(i).at(1), targets_.at(i).at(2));
        vec_distances.push_back((target_path_point - _current_point).norm());
    }
    auto smallest_distance = std::min_element(vec_distances.begin(), vec_distances.end());
    int pos_on_path = smallest_distance - vec_distances.begin();

    return pos_on_path + start_search_pos_on_path;
}


int OffboardControl::calculateDistanceOnPath(int _prev_normal_pos_on_path, double _meters) const{
    int pos_equals_dist;
    double dist_to_front, dist_to_back, temp_dist;
    std::vector<double> vec_distances;
    Eigen::Vector3f p_prev = Eigen::Vector3f(targets_.at(_prev_normal_pos_on_path).at(0), targets_.at(_prev_normal_pos_on_path).at(1), targets_.at(_prev_normal_pos_on_path).at(2));
    Eigen::Vector3f p_front = Eigen::Vector3f(targets_.front().at(0), targets_.front().at(1), targets_.front().at(2));
    Eigen::Vector3f p_back = Eigen::Vector3f(targets_.back().at(0), targets_.back().at(1), targets_.back().at(2));
    dist_to_front = (p_prev - p_front).norm();
    dist_to_back = (p_prev - p_back).norm();
    temp_dist = 0.0;
    if (_meters > 0) {
        if (_meters < dist_to_back) {
            for (int i = _prev_normal_pos_on_path; i < targets_.size() - 1; i++) {
                Eigen::Vector3f p1 = Eigen::Vector3f(targets_.at(i).at(0), targets_.at(i).at(1), targets_.at(i).at(2));
                Eigen::Vector3f p2 = Eigen::Vector3f(targets_.at(i+1).at(0), targets_.at(i+1).at(1), targets_.at(i+1).at(2));
                temp_dist = temp_dist + (p2 - p1).norm();
                if (temp_dist < _meters) {
                    pos_equals_dist = i;
                } else {
                    i = targets_.size();
                }
            }
        } else {
            pos_equals_dist = targets_.size() - 1;
        }
    } else {
        if (_meters < dist_to_front) {
            pos_equals_dist = 0;
            for (int i = _prev_normal_pos_on_path; i >= 1; i--) {
                Eigen::Vector3f p1 = Eigen::Vector3f(targets_.at(i).at(0), targets_.at(i).at(1), targets_.at(i).at(2));
                Eigen::Vector3f p0 = Eigen::Vector3f(targets_.at(i-1).at(0), targets_.at(i-1).at(1), targets_.at(i-1).at(2));
                temp_dist = temp_dist + (p1 - p0).norm();
                if (temp_dist < fabs(_meters / 2)) {
                    pos_equals_dist = i;
                } else {
                    i = 0;
                }
            }
        } else {
            pos_equals_dist = 0;
        }
    }

    return pos_equals_dist;
}

int main(int argc, char* argv[]) {

	
	
	std::string targets_file = argv[1];
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>(targets_file));

	rclcpp::shutdown();
	return 0;
}
