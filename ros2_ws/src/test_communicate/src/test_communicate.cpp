#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial/serial.h"

#include "communicate_msg/msg/mecanum.hpp"

using namespace std::chrono_literals;
serial::Serial ser;
std::string port = "/dev/ttyACM0";


class SerialObj: public rclcpp::Node{
	public: 
		SerialObj(): Node("test_serial_node"){
			// publisher_ = this->create_publisher<communicate_msg::msg::Mecanum>("control", 10);
			// timer_ = this->create_wall_timer(500ms, std::bind(&SerialObj::timer_callback, this));		
			
			subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
			std::bind(&SerialObj::joy_callback, this, std::placeholders::_1) );
		}
	private:
		// left x-axis(joy): 1, y-axis(joy): 0, LT: 2, RT: 5
		void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
			float v_x = msg->axes[1];
			float v_y = -msg->axes[0];

			float w = (((1- msg->axes[2]) - (1 - msg->axes[5])) / 2);

			float speed = std::sqrt(v_x*v_x + v_y*v_y);
			float theta = std::atan2(v_y, v_x);
			// std::fabs();
			printf("speed: %f, theta: %f", speed, theta);
			printf(", w: %f\n", w);
			try{
				if(ser.isOpen()){
					int counter = 2;
					uint8_t dataArray[16] = {0};


					dataArray[0] = '$';
					dataArray[1] = 0x14;
					dataArray[14] = '\r';
					dataArray[15] = '\n';
						
					for(int _i = 0; _i< (int)sizeof(float); ++_i){
						dataArray[counter] = ((uint8_t *)&speed)[_i];
						++counter;
					}
					
					for(int _i = 0; _i< (int)sizeof(float); ++_i){
						dataArray[counter] = ((uint8_t *)&theta)[_i];
						++counter;
					}
					
					for(int _i = 0; _i< (int)sizeof(float); ++_i){
						dataArray[counter] = ((uint8_t *)&w)[_i];
						++counter;
					}

					ser.write(dataArray, 16);
				}
				else{
					try{
						ser.setPort(port);
						ser.setBaudrate(115200);
						serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
						ser.setTimeout(timeout);
						ser.open();
						std::this_thread::sleep_for(std::chrono::seconds(3));
					}
					catch(serial::IOException &e){
						std::cout << "Unable to open serial port: " << ser.getPort() << std::endl;
					}
					if(ser.isOpen()){
						std::cout << "Serial port: " << ser.getPort() << " initialized and opened." << std::endl;
					}
				}
			}
			catch(serial::IOException &e){
				std::cout << "error reading from the serial port: " << ser.getPort() << ". Closing connection." << std::endl;
				ser.close();
			}	
		}

		// rclcpp::TimerBase::SharedPtr timer_;
		// rclcpp::Publisher<communicate_msg::msg::Mecanum>::SharedPtr publisher_;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

};

int main(int argc, char **argv){
	
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<SerialObj>());
	rclcpp::shutdown();
	
	return 0;
}
