#ifndef AT9S_H
#define AT9S_H

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "at9s/SerialCommunicate.h"

class AT9S: public rclcpp::Node{
	public:
		AT9S();
		~AT9S();
	private:

		std::string _port;
        SerialCommunicate _serialCommunicate;
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_; 

        void _timer_callback();

};

#endif