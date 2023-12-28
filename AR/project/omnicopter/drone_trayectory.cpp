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
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>


#include <chrono>
#include <iostream>
#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using std::placeholders::_1;

class DroneTrayectory : public rclcpp::Node
{

float minimum_distance = 0.25;
float minimum_angle = 6.5;
std::vector<std::vector<float>> Goal;
std::vector<std::vector<float>> targets;
std::ifstream inFile;
int currentTarget = 0; //index with the next target to reach

public:
	DroneTrayectory(const std::string &targets_file) : Node("drone_trayectory")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        vehicle_attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
		std::bind(&DroneTrayectory::positionCb, this, _1));
        // subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        // "/fmu/out/vehicle_odometry", qos,
        // [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
        //     RCLCPP_INFO(this->get_logger(), "Received VehicleOdometry message");
        //     // Add additional debug prints if needed
        //     //this->positionCb(msg);
        // });

        this->declare_parameter("targets_file", targets_file);
        std::string targets_file_param = this->get_parameter("targets_file").as_string();
        std::cout << "Reading targets file: " << targets_file_param << '\n';

        //std::string pkg_path = ament_index_cpp::get_package_share_directory("px4_ros_com");
		std::string pkg_path = "/home/luis/trayectory/src/px4_ros_com";
        inFile.open(pkg_path + "/src/" + targets_file_param + ".txt");
        std::cout << "Path: " << pkg_path + "/src/" + targets_file_param + ".txt" << '\n';

		std::string target_string;
		if ( inFile.is_open() ) {
			while ( std::getline (inFile, target_string) ) { // equivalent to myfile.good()
				std::cout << "Read point: " << target_string << '\n';

				std::vector<float> target;

				std::istringstream ss(target_string);
				std::string token;

				while(std::getline(ss, token, ';')) {
                    target.push_back(std::atof(token.c_str()));
				}

				targets.push_back(target);
				
			}
			
			inFile.close();

		}
		else {
			std::cout << "Couldn't open file\n";
		}
        
        std::vector<float> first_target = targets.front();
        Goal.resize(2, std::vector<float>(3, 0.0));

        Goal[0][0] = first_target.at(0);
        Goal[0][1] = first_target.at(1);
        Goal[0][2] = first_target.at(2);
        Goal[1][0] = first_target.at(3);
        Goal[1][1] = first_target.at(4);
        Goal[1][2] = first_target.at(5);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
            
            //std::cout << "offboard_setpoint_counter_:" << offboard_setpoint_counter_ << std::endl;
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;
    float x_drone_, y_drone_, z_drone_;
    float ex_, ey_, ez_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void positionCb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void quaternionToEulerAngles(const float q[4], float& roll, float& pitch, float& yaw);
    void eulerAnglesToQuaternion(float roll, float pitch, float yaw, float q[4]);
    float degreesToRadians(float degrees);
    float radiansToDegrees(float radians);

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void DroneTrayectory::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void DroneTrayectory::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void DroneTrayectory::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void DroneTrayectory::publish_trajectory_setpoint()
{
    // std::cout << "publish"  << std::endl;
	TrajectorySetpoint msg_position{};
    VehicleAttitudeSetpoint msg_angle{};
	//msg.position = {0.0, 0.0, -5.0};
	//msg.yaw = Goal[1][2]; // [-PI:PI]

    // std::cout << "Drone Orientation: Roll=" << Goal[1][0]  << " degrees, "
    // << "Pitch=" << Goal[1][1]  << " degrees, "
    // << "Yaw=" << Goal[1][2]  << " degrees" << std::endl;
        
    msg_position.position = {Goal[0][0], Goal[0][1], Goal[0][2]};
    //msg_position.yaw = Goal[1][2];

    // msg_angle.roll_body = degreesToRadians(Goal[1][0]);
    // msg_angle.pitch_body = degreesToRadians(Goal[1][1]);
    // msg_angle.yaw_body = degreesToRadians(Goal[1][2]);

    //msg_angle.yaw_sp_move_rate = 0.2;

    msg_angle.thrust_body = {ex_/10.0, ey_/10.0, ez_/10.0};
    // msg_angle.reset_integral =true;
    float q[4];
    eulerAnglesToQuaternion(Goal[1][0], Goal[1][1], Goal[1][2], q);

    std::array<float, 4> q_array;
    std::copy(std::begin(q), std::end(q), std::begin(q_array));

    //msg_angle.q_d = q_array;

    //msg.yaw = degreesToRadians(Goal[1][2]); // [-PI:PI]
    float time = this->get_clock()->now().nanoseconds() / 1000;
	msg_position.timestamp = time;
	msg_angle.timestamp = time;
	//trajectory_setpoint_publisher_->publish(msg_position);
    vehicle_attitude_setpoint_publisher_->publish(msg_angle);

}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void DroneTrayectory::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void DroneTrayectory::positionCb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Position X: %.2f, Y: %.2f and Z: %.2f",
    //             msg->position[0], msg->position[1], msg->position[2]);
    //std::cout << "GOT POSITION"  << std::endl;
    x_drone_ = msg->position[0];
    y_drone_ = msg->position[1];
    z_drone_ = msg->position[2];

    float q[4];
    for (int i = 0; i < 4; ++i) {
        q[i] = msg->q[i];
    }

    float roll, pitch, yaw;
    quaternionToEulerAngles(q, roll, pitch, yaw);
    float roll_degrees, pitch_degrees, yaw_degrees;
    roll_degrees = radiansToDegrees(roll);
    pitch_degrees = radiansToDegrees(pitch);
    yaw_degrees = radiansToDegrees(yaw);

    std::cout << "goal desired x: " << Goal[0][0] << " y: " <<
    Goal[0][1] << " z: " << Goal[0][2] << std::endl;

    std::cout << "Drone Orientation: Roll=" << roll_degrees << " degrees, "
        << "Pitch=" << pitch_degrees << " degrees, "
        << "Yaw=" << yaw_degrees << " degrees" << std::endl;
    

    ex_ = Goal[0][0] - x_drone_;
	ey_ = Goal[0][1] - y_drone_;
	ez_ = Goal[0][2] - z_drone_;

	float distance_to_target = sqrt(ex_*ex_+ey_*ey_+ez_*ez_);

    float eRoll = roll_degrees - Goal[1][0];
	float ePitch = pitch_degrees - Goal[1][1];
	float eYaw = yaw_degrees - Goal[1][2];

	float angle_to_target = sqrt(eRoll*eRoll+ePitch*ePitch+eYaw*eYaw);


    
    std::cout << "Roll desired: " << Goal[1][0] << " degrees, "
    << "Pitch desired: " << Goal[1][1] << " degrees, "
    << "Yaw desired:" << Goal[1][2] << " degrees" << std::endl;

    std::cout << "distance to target: " << distance_to_target << std::endl;
    std::cout << "angle to target: " << angle_to_target << std::endl;



    if(distance_to_target < minimum_distance && (angle_to_target < minimum_angle || (360 - angle_to_target) < minimum_angle)) {
        if (currentTarget == targets.size() - 1){
            printf("Final goal reached!");
            return;
        }

        currentTarget++;

        std::vector<float> target = targets.at(currentTarget);

        Goal[0][0] = target.at(0);
        Goal[0][1] = target.at(1);
        Goal[0][2] = target.at(2);
        Goal[1][0] = target.at(3);
        Goal[1][1] = target.at(4);
        Goal[1][2] = target.at(5);


    }

    // TrajectorySetpoint msg_publish{};
    // msg_publish.position = {Goal[0][0], Goal[0][1], Goal[0][2]};
    // msg_publish.yaw = Goal[1][2]; // [-PI:PI]
    // msg_publish.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // trajectory_setpoint_publisher_->publish(msg_publish);

}

void DroneTrayectory::quaternionToEulerAngles(const float q[4], float& roll, float& pitch, float& yaw)
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

void DroneTrayectory::eulerAnglesToQuaternion(float roll, float pitch, float yaw, float q[4])
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

float DroneTrayectory::degreesToRadians(float degrees)
{
    return degrees * M_PI / 180.0;
}

float DroneTrayectory::radiansToDegrees(float radians)
{
    return radians * 180.0 / M_PI;
}



int main(int argc, char *argv[])
{
     if (argc < 2) {
        std::cerr << "Usage: ros2 run px4_ros_com drone_trayectory <targets_file>" << std::endl;
        return 1;
    }

    std::string targets_file = argv[1];
    
	std::cout << "Starting drone trayectory node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DroneTrayectory>(targets_file));

	rclcpp::shutdown();
	return 0;
}
