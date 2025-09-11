#ifndef ROS2_COMMUNICATE_H
#define ROS2_COMMUNICATE_H

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial/serial.h"

#include "SerialCommunicate.h"

class SerialObj: public rclcpp::Node{
	public:
		SerialObj();
		~SerialObj();

	private:   

		std::string _port;
		serial::Serial _ser;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscription;

		void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
		// rclcpp::TimerBase::SharedPtr timer_;
		// rclcpp::Publisher<communicate_msg::msg::Mecanum>::SharedPtr publisher_; 

		SerialCommunicate serialCommunicate;


};

#endif