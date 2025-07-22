#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

#include "communicate_msg/msg/mecanum.hpp"

using namespace std::chrono_literals;

serial::Serial ser;
std::string port = "/dev/ttyACM0";
class SerialObj: public rclcpp::Node{
	public: 
		SerialObj(): Node("test_serial_node"){
			publisher_ = this->create_publisher<communicate_msg::msg::Mecanum>("control", 10);
			timer_ = this->create_wall_timer(500ms, std::bind(&SerialObj::timer_callback, this));		
		}
	private:
		void timer_callback(){
			auto message = communicate_msg::msg::Mecanum();
			message.speed = 5;
			message.angle = 3;
			message.w = 10;

			publisher_->publish(message);

			try{
				if(ser.isOpen()){
					std::cout << "qq" << std::endl;	

				}
				else{
					try{
						ser.setPort(port);
						ser.setBaudrate(115200);
						serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
						ser.setTimeout(timeout);
						ser.open();
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

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<communicate_msg::msg::Mecanum>::SharedPtr publisher_;
};

int main(int argc, char **argv){
	std::cout << "hi" << std::endl;
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<SerialObj>());
	rclcpp::shutdown();
	return 0;
}
