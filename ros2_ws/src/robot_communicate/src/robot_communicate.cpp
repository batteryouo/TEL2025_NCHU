#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial/serial.h"

#include "communicate_msg/msg/mecanum.hpp"
#include "communicate_msg/msg/imu.hpp"
#include "communicate_msg/msg/int32.hpp"
#include "robot_communicate.hpp"
using namespace std::chrono_literals;



float uint8Vector2Float(std::vector<uint8_t> data, int bias = 0){
	uint8_t num[4] = {0};
	for(int i = 0; i< 4; ++i){
		num[i] = data[i+bias];
	}
	return ((float *)num)[0];
}

SerialObj::SerialObj():Node("test_serial_node"){
	_joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,	
		std::bind(&SerialObj::_joy_callback, this, std::placeholders::_1) );
	_move_subscription = this->create_subscription<communicate_msg::msg::Mecanum>("/move", 10,
		std::bind(&SerialObj::_move_callback, this, std::placeholders::_1));
	
	_imuPublisher = this->create_publisher<communicate_msg::msg::Imu>("launch_imu", 10);
	_launchPublisher = this->create_publisher<communicate_msg::msg::Int32>("launch", 10);
	timer_ = this->create_wall_timer(1ms, std::bind(&SerialObj::_timer_callback, this)) ;
	
	this->declare_parameter("port", "/dev/ttyACM0");
	_port = this->get_parameter("port").as_string();

	serialCommunicate.init(_port);
}

SerialObj::~SerialObj(){

}

void SerialObj::_timer_callback(){
	std::vector<uint8_t> data;
	cmd::Command_Type command = serialCommunicate.read(data);

	if(command == cmd::Command_Type::IMU_YPR){
		communicate_msg::msg::Imu launch_imu;
		float y, p, r;
		y = uint8Vector2Float(data, 0);
		// p = uint8Vector2Float(data, 4);
		// r = uint8Vector2Float(data, 8);
		p = 0;
		r = 0;
		launch_imu.header.stamp = this->get_clock()->now();
		launch_imu.header.frame_id = "launch_imu";
		launch_imu.y = y;
		launch_imu.p = p;
		launch_imu.r = r;

		_imuPublisher->publish(launch_imu);
	}
	if(command == cmd::Command_Type::LAUNCH){

		int launch = 0;
		communicate_msg::msg::Int32 launch_data;
		launch = (int)data[0];

		launch_data.header.stamp = this->get_clock()->now();
		launch_data.header.frame_id = "launch";
		launch_data.data = launch;
		

		_launchPublisher->publish(launch_data);

	}
}

void SerialObj::_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
	float v_x = msg->axes[0];
	float v_y = msg->axes[1];
	float w = msg->axes[2];

	float knob = msg->axes[3];

	int32_t launch = msg->buttons[0];
	int32_t mode = msg->buttons[1];
	int32_t start = msg->buttons[2];

	float speed = std::sqrt(v_x*v_x + v_y*v_y);
	float theta = std::atan2(v_y, v_x);
	
	printf("speed: %f, theta: %f", speed, theta);
	printf(", w: %f\n", w);
	if(start != 1){
		restart_flag = true;
	}
	if(start == 1 && restart_flag){
		std::vector<uint8_t> data;
		for(int i = 0; i< 4; ++i){
			uint8_t tmp = ((uint8_t *)&speed)[i];
			data.push_back(tmp);
		}
		for(int i = 0; i< 4; ++i){
			uint8_t tmp = ((uint8_t *)&theta)[i];
			data.push_back(tmp);
		}
		for(int i = 0; i< 4; ++i){
			uint8_t tmp = ((uint8_t *)&w)[i];
			data.push_back(tmp);
		}

		serialCommunicate.write(data, cmd::Command_Type::MOVE_POLAR);

		data.clear();
		for(int i = 0; i< 4; ++i){
			// knob data has already normalized to 0~1
			uint8_t tmp = ((uint8_t*)&knob)[i];
			data.push_back(tmp);
		}
		serialCommunicate.write(data, cmd::Command_Type::LAUNCH_ANGLE_NORMALIZE);

		data.clear();
		data.push_back((uint8_t)launch);
		serialCommunicate.write(data, cmd::Command_Type::LAUNCH);

	}
	
}

void _move_callback(const communicate_msg::msg::Mecanum::SharedPtr msg){
	float speed = msg->speed;
	float theta = msg->angle;
	float w = msg->w;

	std::vector<uint8_t> data;
	for(int i = 0; i< 4; ++i){
		uint8_t tmp = ((uint8_t *)&speed)[i];
		data.push_back(tmp);
	}
	for(int i = 0; i< 4; ++i){
		uint8_t tmp = ((uint8_t *)&theta)[i];
		data.push_back(tmp);
	}
	for(int i = 0; i< 4; ++i){
		uint8_t tmp = ((uint8_t *)&w)[i];
		data.push_back(tmp);
	}

	serialCommunicate.write(data, cmd::Command_Type::MOVE_POLAR);
}