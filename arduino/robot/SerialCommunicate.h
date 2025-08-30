#ifndef SERIALCOMMUNICATE
#define SERIALCOMMUNICATE

#define CMD_HEADER_LENGTH 6

#include <Arduino.h>

namespace cmd{

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
		bool computeCheckSum(const uint8_t *packet, uint8_t packetLength, uint8_t *checkSum_1, uint8_t *checkSum_2);
		bool isCheckSumValid(const uint8_t *packet, uint8_t packetLength);
	private:
		static constexpr uint8_t HeaderLength = CMD_HEADER_LENGTH;
		bool _validPacket(const uint8_t *packet, uint8_t packetLength);

		enum HEADER_INFO{
			Start1, Start2, Packet_SIZE, Command, CheckSum_1, CheckSum_2
		};

};
};

class SerialCommunicate: private cmd::CommandProtocol{
	public:
		SerialCommunicate(int bufferSize = 100);
		~SerialCommunicate();
		
	private:
		uint8_t *_commandBuffer;
		int _bufferSize = 1;
		int _usage = 0;
		int _bufferStart = 0;
		void _flush();
};

#endif