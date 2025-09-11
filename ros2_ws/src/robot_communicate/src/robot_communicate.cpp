#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial/serial.h"

#include "communicate_msg/msg/mecanum.hpp"
#include "robot_communicate.hpp"
using namespace std::chrono_literals;

SerialObj::SerialObj():Node("test_serial_node"){
	_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,	
		std::bind(&SerialObj::_joy_callback, this, std::placeholders::_1) );

	this->declare_parameter("port", "/dev/ttyACM0");
	_port = this->get_parameter("port").as_string();

	serialCommunicate.init(_port);
}

SerialObj::~SerialObj(){

}

void SerialObj::_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
	float v_x = msg->axes[0];
	float v_y = msg->axes[1];
	float w = msg->axes[2];

	int32_t launch = msg->buttons[0];
	int32_t mode = msg->buttons[1];
	int32_t start = msg->buttons[2];

	float speed = std::sqrt(v_x*v_x + v_y*v_y);
	float theta = std::atan2(v_y, v_x);
	
	printf("speed: %f, theta: %f", speed, theta);
	printf(", w: %f\n", w);

	if(start == 1){
		std::vector<uint8_t> data;
		for(int i = 0; i< 4; ++i){
			uint8_t tmp = ((uint8_t *)&v_x)[i];
			data.push_back(tmp);
		}
		for(int i = 0; i< 4; ++i){
			uint8_t tmp = ((uint8_t *)&v_y)[i];
			data.push_back(tmp);
		}
		for(int i = 0; i< 4; ++i){
			uint8_t tmp = ((uint8_t *)&w)[i];
			data.push_back(tmp);
		}

		serialCommunicate.write(data, cmd::Command_Type::MOVE_POLAR);

	}
	
}
