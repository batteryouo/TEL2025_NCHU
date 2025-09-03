#include <Arduino.h>

#include "SerialCommunicate.h"

cmd::CommandProtocol::CommandProtocol(){

}

cmd::CommandProtocol::~CommandProtocol(){

}

bool cmd::CommandProtocol::computeCheckSum(const uint8_t *packet, uint8_t packetLength, uint8_t *checkSum_1, uint8_t *checkSum_2){
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
	if(!_validPacket(packet, packetLength)){
		return false;
	}

	uint8_t tmp_checkSum_1 = 0, tmp_checkSum_2 = 0;
	uint8_t packet_size = packet[Packet_SIZE];
	uint8_t CMD = packet[Command];

	tmp_checkSum_1 = packet_size ^ CMD;
	for(uint8_t i = HeaderLength; i< packet_size; ++i){
		tmp_checkSum_1 ^= packet[i];
	}
	tmp_checkSum_1 &= 0xFE;
	tmp_checkSum_2 = (~tmp_checkSum_1) & 0xFE;

	*checkSum_1 = tmp_checkSum_1;
	*checkSum_2 = tmp_checkSum_2;

	return true;
}

bool cmd::CommandProtocol::isCheckSumValid(const uint8_t *packet, uint8_t packetLength){
	
	if(!_validPacket(packet, packetLength)){
		return false;
	}
	uint8_t checkSum_1, checkSum_2;

	computeCheckSum(packet, packetLength, &checkSum_1, &checkSum_2);

	return packet[CheckSum_1] == checkSum_1 && packet[CheckSum_2] == checkSum_2;	
}

bool cmd::CommandProtocol::_validPacket(const uint8_t *packet, uint8_t packetLength){
	if(packetLength < HeaderLength){
		return false;
	}
	if(packet[Packet_SIZE] != packetLength){
		return false;	
	}
	if(packet[Start1] != '$' || packet[Start2] != 0x14){
		return false;
	}

	return true;
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
