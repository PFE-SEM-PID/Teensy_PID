#ifndef TEENSYLC_PID_ENCODER_HPP
#define TEENSYLC_PID_ENCODER_HPP
#include <Arduino.h>
#include <Encoder.h>

//ENCODER SETTINGS
#define CHANA 2
#define CHANB 3
#define ticks_per_turn 400

volatile uint8_t last_a=0;
volatile uint8_t last_b=0;
volatile uint8_t curr_a=0;
volatile uint8_t curr_b=0;

Encoder encoder(CHANA, CHANB);;

//INITIALISATION AND START READING
void encoder_init(){
	//pinMode(CHANA,INPUT_PULLUP);
	//pinMode(CHANB,INPUT_PULLUP);
	//attachInterrupt(digitalPinToInterrupt(CHANA), readChanA, RISING);
	//attachInterrupt(digitalPinToInterrupt(CHANB), readChanB, CHANGE);
}

#endif //TEENSYLC_PID_ENCODER_HPP