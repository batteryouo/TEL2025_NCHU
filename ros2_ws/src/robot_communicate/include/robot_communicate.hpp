#ifndef ROS2_COMMUNICATE_H
#define ROS2_COMMUNICATE_H

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial/serial.h"

#include "SerialCommunicate.h"
#include "communicate_msg/msg/mecanum.hpp"
#include "communicate_msg/msg/imu.hpp"
#include "communicate_msg/msg/int32.hpp"

class SerialObj: public rclcpp::Node{
	public:
		SerialObj();
		~SerialObj();

	private:   

		std::string _port;
		serial::Serial _ser;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscription;
		rclcpp::Subscription<communicate_msg::msg::Mecanum>::SharedPtr _move_subscription;
		rclcpp::Subscription<communicate_msg::msg::Int32>::SharedPtr _launch_subscription;
		rclcpp::Subscription<communicate_msg::msg::Imu>::SharedPtr _launch_angle_subscription;

		void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
		void _move_callback(const communicate_msg::msg::Mecanum::SharedPtr msg);
		void _launch_callback(const communicate_msg::msg::Int32::SharedPtr msg);
		void _launch_angle_callback(const communicate_msg::msg::Imu::SharedPtr msg);
		void _timer_callback();
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<communicate_msg::msg::Imu>::SharedPtr _imuPublisher;
		rclcpp::Publisher<communicate_msg::msg::Int32>::SharedPtr _launchPublisher;

		SerialCommunicate serialCommunicate;
		bool restart_flag = false;


};

#endif