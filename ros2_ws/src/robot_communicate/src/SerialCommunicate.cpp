#include <iostream>
#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "SerialCommunicate.h"

cmd::CommandProtocol::CommandProtocol(){

	/*
		1	  2	  3		   4   5		  6		 +N
		0	  1	  2		   3   4		  5		 +N
		Start1 Start2 packet_size CMD checkSum_1 checkSum2 data
		'$'	0x14   6+N		 (a) (b)		(c)	   ...
	*/ 
	// check_sum_1 = (N^CMD^data[0]^data[1]^...) & 0xFE
	// check_sum_2 = (~check_sum_1) & 0xFE

	// dataLength smaller than minumum required(header length). also avoid for invalid data input(e.g. {'$', 0x14} 0xFF ... )
	//																								 ^size is 2  ^data[2]

}

cmd::CommandProtocol::~CommandProtocol(){

}

bool cmd::CommandProtocol::packetValid(const std::vector<uint8_t> &packet){
	if(packet.size() < HeaderLength){
		return false;
	}
	if(packet[HEADER_INFO::Start1] != '$' || packet[HEADER_INFO::Start2] != 0x14){
		return false;
	}
	if(packet[HEADER_INFO::Packet_SIZE] != packet.size()){
		return false;
	}
	if(packet[HEADER_INFO::Packet_SIZE] != _expectPacketSize((Command_Type)packet[HEADER_INFO::Command])){
		return false;
	}
	uint8_t checkSum1, checkSum2;
	std::vector<uint8_t> inputData;
	for(size_t i = HeaderLength; i< packet[Packet_SIZE]; ++i){
		inputData.push_back(packet[i]);
	}
	_computeCheckSum(packet[HEADER_INFO::Packet_SIZE], (Command_Type)packet[HEADER_INFO::Command], inputData, checkSum1, checkSum2);

	if(checkSum1 != packet[HEADER_INFO::CheckSum_1] || checkSum2 != packet[HEADER_INFO::CheckSum_2]){
		return false;
	}
	return true;
}

void cmd::CommandProtocol::buildPacket(Command_Type inputCommand, const std::vector<uint8_t> &inputData, std::vector<uint8_t> &outputPacket){
	if(outputPacket.size()){
		outputPacket.clear();
	}

	uint8_t packetLength = HeaderLength + inputData.size();
	uint8_t checkSum1, checkSum2;

	_computeCheckSum(packetLength, inputCommand, inputData, checkSum1, checkSum2);

	outputPacket.push_back('$');
	outputPacket.push_back(0x14);
	outputPacket.push_back(packetLength);
	outputPacket.push_back((uint8_t)inputCommand);
	outputPacket.push_back(checkSum1);
	outputPacket.push_back(checkSum2);

	for(size_t i = 0; i< inputData.size(); ++i){
		outputPacket.push_back(inputData[i]);
	}
}

bool cmd::CommandProtocol::parsePacket(const std::vector<uint8_t> &packet){
	_data.clear();
	if(!packetValid(packet)){
		_command = Command_Type::None;
		return false;
	}

	_command = (Command_Type)packet[HEADER_INFO::Command];
	for(size_t i = HeaderLength; i< packet[Packet_SIZE]; ++i){
		_data.push_back(packet[i]);
	}
	return true;
}

cmd::Command_Type cmd::CommandProtocol::command(){
	return _command;
}

std::vector<uint8_t> cmd::CommandProtocol::data(){
	return _data;
}

void cmd::CommandProtocol::_computeCheckSum(uint8_t packetLength, Command_Type inputCommand, const std::vector<uint8_t> &inputData, uint8_t &checkSum1, uint8_t &checkSum2){
	checkSum1 = packetLength ^ (uint8_t)inputCommand;
	for(size_t i = 0; i< inputData.size(); ++i){
		checkSum1 ^= inputData[i];
	}
	checkSum1 = checkSum1 & 0xFE;
	checkSum2 = (~checkSum1) & 0xFE;
}

size_t cmd::CommandProtocol::_expectPacketSize(cmd::Command_Type inputCommand){
	size_t expectSize = HeaderLength;
	switch (inputCommand){
		case Command_Type::MOVE_CARTESIAN :
			expectSize = HeaderLength + 12; // (float)vx, (float)vy, (float)w
			break;
		case Command_Type::MOVE_POLAR :
			expectSize = HeaderLength + 12; // (float)vx, (float)vy, (float)w
			break;
		case Command_Type::IMU_YPR :
			expectSize = HeaderLength + 12; // (float)y, (float)p, (float)r
			break;
		
		case Command_Type::SWITCH_STATE :
			expectSize = HeaderLength + 1; // (uint8_t)state
			break;
		case Command_Type::RC:
			expectSize = HeaderLength + 19; // (float)right_up_down_axis, (float)right_left_right_axis, (float)left_left_right_axis
											// (float)left_up_down_axis(right knob on controller)
											// (char) launch, (char) mode, (char) open
			break;
		case Command_Type::LAUNCH_ANGLE_NORMALIZE:
			expectSize = HeaderLength + 4; // (float) normalize_angle
			break;
		case Command_Type::LAUNCH:
			expectSize = HeaderLength + 1; // (int) launch
			break;
		case Command_Type::None :
			expectSize = 0;
			break;
		default:
			expectSize = 0;
			break;
	}

	return expectSize;
}

SerialCommunicate::SerialCommunicate(){
}

SerialCommunicate::~SerialCommunicate(){
	_ser.close();
}

void SerialCommunicate::init(std::string port){
	_port = port;

	_ser.setPort(_port);
	_ser.setBaudrate(115200);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
	_ser.setTimeout(timeout);
	try{
		_ser.open();
		std::this_thread::sleep_for(std::chrono::seconds(3));
	
		if(_ser.isOpen()){
			std::cout << "Serial port: " << _ser.getPort() << " initialized and opened." << std::endl;
		}
	}
	catch(serial::IOException &e){
		std::cout << "Unable to open serial port: " << _ser.getPort() << std::endl;
		_ser.close();
	}
	
}

cmd::Command_Type SerialCommunicate::read(std::vector<uint8_t> &outputData){
	
	cmd::Command_Type returnCMD = cmd::Command_Type::None;

	if(_ser.available()){
		
		std::string readString = _ser.read(_ser.available());
		
		_input += readString;
		// _input = readString;	
		
	}
	uint8_t c;
	int _i = 0;
	for(_i = 0; _i< _input.size(); ++_i){	
		c = _input[_i];
		_packet.push_back(c);
		
		bool startFlag = _startFlag();
		if(!startFlag){
			_packet.clear();
			continue;
		}
		if(_packet.size() >= HEADER_INFO::Packet_SIZE + 1 && _packet.size() != _packet[HEADER_INFO::Packet_SIZE]){
			continue;
		}
		else if(parsePacket(_packet)){
			outputData = data();
			returnCMD = command();
			_packet.clear();
			break;
		}
		if(_packet.size() > MAX_BUFFER_SIZE){
			_packet.clear();
			break;
		}
	}
	if(_input.size() >= 0){
		_input.erase(0, _i+1);
	}
	return returnCMD;
}

void SerialCommunicate::write(const std::vector<uint8_t> &inputData, cmd::Command_Type inputCommand){
	std::vector<uint8_t> outputPacket;
	buildPacket(inputCommand, inputData, outputPacket);

	_ser.write(outputPacket.data(), outputPacket.size());
}

bool SerialCommunicate::_startFlag(){

	if(_packet.size() >= CMD_HEADER_LENGTH){
		return true;
	}
	if(_packet.size() == 1 && _packet[HEADER_INFO::Start1] != '$'){
		return false;
	}
	if(_packet.size() == 2 && _packet[HEADER_INFO::Start2] != 0x14){
		return false;
	}
	if(_packet.size() >= 4){
		return (_packet[HEADER_INFO::Packet_SIZE] == _expectPacketSize((cmd::Command_Type)_packet[HEADER_INFO::Command]));
	}

	return true;
}
