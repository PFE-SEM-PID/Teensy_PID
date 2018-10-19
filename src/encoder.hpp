#ifndef TEENSYLC_PID_ENCODER_HPP
#define TEENSYLC_PID_ENCODER_HPP
#include <Arduino.h>

//ENCODER SETTINGS
#define CHANA 2
#define CHANB 3
#define ticks_per_turn 100

volatile int32_t encoder_pos = 0;

//ENCODER READ FUNCTION
void readCodeuse()
{
	if (digitalRead(CHANB) == LOW) {
		encoder_pos--;
	} else {
		encoder_pos++;
	}
}

//INITIALISATION AND START READING
void encoder_init(){
	pinMode(CHANA,INPUT);
	pinMode(CHANB,INPUT);
	attachInterrupt(digitalPinToInterrupt(CHANA), readCodeuse, RISING);
}


#endif //TEENSYLC_PID_ENCODER_HPP
