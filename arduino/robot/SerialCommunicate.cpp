#include <Arduino.h>

#include "SerialCommunicate.h"
#include "vector.hpp"

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

}

vector<uint8_t> cmd::CommandProtocol::buildPacket(Command_Type inputCommand, const vector<uint8_t> &inputData){

}

bool cmd::CommandProtocol::parsePacket(const vector<uint8_t> &packet){

}

cmd::Command_Type cmd::CommandProtocol::command(){
	return _command;
}

vector<uint8_t> cmd::CommandProtocol::data(){
	return _data;
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
