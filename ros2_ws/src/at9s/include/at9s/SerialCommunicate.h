#ifndef SERIALCOMMUNICATE_H
#define SERIALCOMMUNICATE_H

#define CMD_HEADER_LENGTH 6
#define MAX_BUFFER_SIZE 10000

#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"



namespace cmd{

enum Command_Type{
	MOVE_CARTESIAN,
	MOVE_POLAR,
	IMU_YPR,
	SWITCH_STATE,
	RC,
	None
};

class CommandProtocol{
	
	/*
		1      2      3           4   5          6         +N
		0      1      2           3   4          5         +N
		Start1 Start2 packet_size CMD checkSum_1 checkSum2 data
		'$'    0x14   6+N         (a) (b)        (c)       ...
	*/ 
	// check_sum_1 = (N^CMD^data[0]^data[1]^...) & 0xFE
	// check_sum_2 = (~check_sum_1) & 0xFE

	public:
		CommandProtocol();
		~CommandProtocol();

		bool packetValid(const std::vector<uint8_t> &packet);
		void buildPacket(Command_Type inputCommand, const std::vector<uint8_t> &inputData, std::vector<uint8_t> &outputPacket);	
		bool parsePacket(const std::vector<uint8_t> &packet);
		Command_Type command();
		std::vector<uint8_t> data();
	protected:
		size_t _expectPacketSize(Command_Type inputCommand);
		enum HEADER_INFO{
			Start1, Start2, Packet_SIZE, Command, CheckSum_1, CheckSum_2
		};
	private:
		static constexpr uint8_t HeaderLength = CMD_HEADER_LENGTH;
	
		Command_Type _command;
		std::vector<uint8_t> _data;		

		void _computeCheckSum(uint8_t packetLength, Command_Type inputCommand, const std::vector<uint8_t> &inputData
			, uint8_t &checkSum1, uint8_t &checkSum2);

};

};

class SerialCommunicate: private cmd::CommandProtocol{
	public:
		SerialCommunicate();
		~SerialCommunicate();
		void init(std::string port);
		cmd::Command_Type read(std::vector<uint8_t> &outputData);
		void write(const std::vector<uint8_t> &inputData, cmd::Command_Type inputCommand);

	private:
		std::string _port;
		serial::Serial _ser;
		std::vector<uint8_t> _packet;

		bool _startFlag();
		std::string _input;

};

#endif