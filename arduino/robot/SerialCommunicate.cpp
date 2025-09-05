#include <Arduino.h>

#include "SerialCommunicate.h"
#include "vector.h"

cmd::CommandProtocol::CommandProtocol(){

	/*
		1      2      3           4   5          6         +N
		0      1      2           3   4          5         +N
		Start1 Start2 packet_size CMD checkSum_1 checkSum2 data
		'$'    0x14   6+N         (a) (b)        (c)       ...
	*/ 
	// check_sum_1 = (N^CMD^data[0]^data[1]^...) & 0xFE
	// check_sum_2 = (~check_sum_1) & 0xFE

	// dataLength smaller than minumum required(header length). also avoid for invalid data input(e.g. {'$', 0x14} 0xFF ... )
	//                                                                                                 ^size is 2  ^data[2]

}

cmd::CommandProtocol::~CommandProtocol(){

}

bool cmd::CommandProtocol::packetValid(const vector<uint8_t> &packet){
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
	vector<uint8_t> inputData;
	for(size_t i = HeaderLength; i< packet[Packet_SIZE]; ++i){
		inputData.push_back(packet[i]);
	}
	_computeCheckSum(packet[HEADER_INFO::Packet_SIZE], (Command_Type)packet[HEADER_INFO::Command], inputData, checkSum1, checkSum2);

	if(checkSum1 != packet[HEADER_INFO::CheckSum_1] || checkSum2 != packet[HEADER_INFO::CheckSum_2]){
		return false;
	}
	return true;
}

void cmd::CommandProtocol::buildPacket(Command_Type inputCommand, const vector<uint8_t> &inputData, vector<uint8_t> &outputPacket){
	if(!outputPacket.size()){
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

bool cmd::CommandProtocol::parsePacket(const vector<uint8_t> &packet){
	_data.clear();
	if(!packetValid(packet)){
		_command = Command_Type::ERROR;
		return false;
	}

	_command = (Command_Type)packet[HEADER_INFO::Command];
	for(size_t i = HeaderLength; i< packet[Packet_SIZE]; ++i){
		_data.push_back(packet[i]);
	}
}

cmd::Command_Type cmd::CommandProtocol::command(){
	return _command;
}

vector<uint8_t> cmd::CommandProtocol::data(){
	return _data;
}

void cmd::CommandProtocol::_computeCheckSum(uint8_t packetLength, Command_Type inputCommand, const vector<uint8_t> &inputData, uint8_t &checkSum1, uint8_t &checkSum2){
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
		
		case Command_Type::SWITCH_STATE :
			expectSize = HeaderLength + 1; // (uint8_t)state

		case Command_Type::ERROR :
			expectSize = HeaderLength;

		default:
			expectSize = HeaderLength;
			break;
	}

	return expectSize;
}

SerialCommunicate::SerialCommunicate(int bufferSize):_bufferSize(bufferSize){
	
	_commandBuffer = new uint8_t[bufferSize];

}

SerialCommunicate::~SerialCommunicate(){
	delete[] _commandBuffer;
}

void SerialCommunicate::_flush(){
	_usage = 0;
	_bufferStart = 0;
}
