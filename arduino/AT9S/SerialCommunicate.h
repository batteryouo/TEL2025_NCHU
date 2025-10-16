#ifndef SERIALCOMMUNICATE_H
#define SERIALCOMMUNICATE_H

#include "vector.h"

#define CMD_HEADER_LENGTH 6

namespace cmd{

enum Command_Type{
	MOVE_CARTESIAN,
	MOVE_POLAR,
	IMU_YPR,
	SWITCH_STATE,
	LAUNCH_ANGLE_NORMALIZE,
	LAUNCH,
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

		bool packetValid(const vector<uint8_t> &packet);
		void buildPacket(Command_Type inputCommand, const vector<uint8_t> &inputData, vector<uint8_t> &outputPacket);	
		bool parsePacket(const vector<uint8_t> &packet);
		Command_Type command();
		vector<uint8_t> data();
	protected:
		size_t _expectPacketSize(Command_Type inputCommand);
		enum HEADER_INFO{
			Start1, Start2, Packet_SIZE, Command, CheckSum_1, CheckSum_2
		};
	private:
		static constexpr uint8_t HeaderLength = CMD_HEADER_LENGTH;
	
		Command_Type _command;
		vector<uint8_t> _data;		

		void _computeCheckSum(uint8_t packetLength, Command_Type inputCommand, const vector<uint8_t> &inputData
			, uint8_t &checkSum1, uint8_t &checkSum2);

};

};

class SerialCommunicate: private cmd::CommandProtocol{
	public:
		SerialCommunicate(HardwareSerial *serial);
		~SerialCommunicate();
		void init();
		cmd::Command_Type read(vector<uint8_t> &outputData);
		void write(const vector<uint8_t> &inputData, cmd::Command_Type inputCommand);
		
		void float2char();
		void char2float();
	private:
		HardwareSerial *_serial;	
		vector<uint8_t> _packet;

		bool _startFlag();

};

#endif