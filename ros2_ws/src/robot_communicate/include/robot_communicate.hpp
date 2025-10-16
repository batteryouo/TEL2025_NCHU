#ifndef ROS2_COMMUNICATE_H
#define ROS2_COMMUNICATE_H

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial/serial.h"

#include "SerialCommunicate.h"
#include "communicate_msg/msg/mecanum.hpp"
#include "communicate_msg/msg/imu.hpp"

class SerialObj: public rclcpp::Node{
	public:
		SerialObj();
		~SerialObj();

	private:   

		std::string _port;
		serial::Serial _ser;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscription;

		void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
		void _timer_callback();
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<communicate_msg::msg::Mecanum>::SharedPtr publisher_;
		rclcpp::Publisher<communicate_msg::msg::Imu>::SharedPtr _imuPublisher;

		SerialCommunicate serialCommunicate;


};

#endif