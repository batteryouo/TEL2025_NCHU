#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "at9s/at9s.hpp"
#include "at9s/SerialCommunicate.h"

#include <stdio.h>

using namespace std::chrono_literals;

AT9S::AT9S(): Node("at9s"){

    publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&AT9S::_timer_callback, this)) ;
    this->declare_parameter("port", "/dev/ttyACM1");
    
    _port = this->get_parameter("port").as_string();
    std::cout << "waiting for init" << std::endl;
    _serialCommunicate.init(_port);
}

AT9S::~AT9S(){
}

void AT9S::_timer_callback(){
    std::vector<uint8_t> data;
    cmd::Command_Type command = _serialCommunicate.read(data);
    if(command == cmd::Command_Type::RC){ 

        uint8_t num[4] = {0};
        float vx, vy, knob, w;
        int launch, mode, start;
        
        for(int i = 0; i< 4; ++i){
            num[i] = data[i + 0];
        }
        vy = -1.0 * ((float *)num)[0]; // convert to left is positive

        for(int i = 0; i< 4; ++i){
            num[i] = data[i + 4];
        }
        vx = -1.0 * ((float *)num)[0]; // convert to forward is positive

        for(int i = 0; i< 4; ++i){
            num[i] = data[i + 8];
        }
        w = -1.0 * ((float *)num)[0]; // convert to counterclockwise is positive

        for(int i = 0; i< 4; ++i){
            num[i] = data[i + 12];
        }
        knob = ( ((float *)num)[0] + 1.0 ) / 2.0; // convert knob to 0~1

        launch = 1 - (int)data[16]; // convert launch to 1
        mode = (int)data[17];
        start = (int)data[18];
		
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = this->get_clock()->now();  // Timestamp
        joy_msg.header.frame_id = "joy_frame";            // Optional frame ID
        joy_msg.axes = {vx, vy, w, knob};
        joy_msg.buttons = {launch, mode, start};
        
	publisher_->publish(joy_msg);
        std::cout << ": publish succesfully!\n";
    }
}
