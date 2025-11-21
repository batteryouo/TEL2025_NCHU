#include "rclcpp/rclcpp.hpp"
#include "at9s/at9s.hpp"


int main(int argc, char **argv){
	
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<AT9S>());
	rclcpp::shutdown();
	
	return 0;
}
