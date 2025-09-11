#include "rclcpp/rclcpp.hpp"
#include "robot_communicate.hpp"


int main(int argc, char **argv){
	
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<SerialObj>());
	rclcpp::shutdown();
	
	return 0;
}
