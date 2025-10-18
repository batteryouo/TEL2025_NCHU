#include "SBUS.h"
#include "SerialCommunicate.h"

SBUS sbus(Serial3);
SerialCommunicate serialCommunicate(&Serial);

float normalized_SBUS2Serial(int value);
void float2uint8_t(float num, uint8_t *outputArray);

void setup(){
	sbus.begin(false);
	serialCommunicate.init();
	Serial.begin(115200);
}

void loop(){
	uint8_t tmp_data[4] = {0};
	vector<uint8_t> data;
	float value = 0;

	sbus.process();
	value = normalized_SBUS2Serial( sbus.getNormalizedChannel(1) ); // right_left_right_axis
	float2uint8_t(value, tmp_data);
	for(int i = 0; i< 4; ++i){
		data.push_back(tmp_data[i]);
	}
	
	value = normalized_SBUS2Serial( sbus.getNormalizedChannel(2) ); // right_up_down_axis
	float2uint8_t(value, tmp_data);
	for(int i = 0; i< 4; ++i){
		data.push_back(tmp_data[i]);
	}

	value = normalized_SBUS2Serial( sbus.getNormalizedChannel(4) ); // left_left_right_aixs
	float2uint8_t(value, tmp_data);
	for(int i = 0; i< 4; ++i){
		data.push_back(tmp_data[i]);
	}
	
	value = normalized_SBUS2Serial( sbus.getNormalizedChannel(8) ); // left_up_down_aixs(knob)
	float2uint8_t(value, tmp_data);
	for(int i = 0; i< 4; ++i){
		data.push_back(tmp_data[i]);
	}

	value = normalized_SBUS2Serial( sbus.getNormalizedChannel(7) ); // launch
	uint8_t launch = 0;
	if(value > 0.9){
		launch = 0;
	}
	else{
		launch = 1;
	}
	data.push_back(launch);

	value = normalized_SBUS2Serial( sbus.getNormalizedChannel(5) ); // mode
	uint8_t mode = 0;
	if(value > 0.9){
		mode = 0;
	}
	else if(value > -0.1){
		mode = 1;
	}
	else{
		mode = 2;
	}
	data.push_back(mode);

	
	value = normalized_SBUS2Serial( sbus.getNormalizedChannel(9) ); // start
	uint8_t is_start = 0;
	if(value > 0.9){
		is_start = 0;
	}
	else{
		is_start = 1;
	}
	data.push_back(is_start);	

	serialCommunicate.write(data, cmd::Command_Type::RC);

	delay(50);
}

float normalized_SBUS2Serial(int value){
	value = constrain(value, -80, 80);

	return ( (float)value - (-80) )/( 80 - (-80) ) * 2.0 - 1.0;
};

void float2uint8_t(float num, uint8_t *outputArray){
	for(int i = 0; i< 4; ++i){
		outputArray[i] = ((uint8_t *)&num)[i];
	}
}
