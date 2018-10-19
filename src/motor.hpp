#ifndef TEENSYLC_PID_MOTOR_HPP
#define TEENSYLC_PID_MOTOR_HPP


#include <Arduino.h>
#include <IntervalTimer.h>
#include "encoder.hpp"
//PINS & LIMITS
#define INA 		4
#define INB 		5
#define PWM 	6
#define SEL0 	11
#define CS 		12
#define MAX_PWM	255
#define FREQ_ASSERV	100 //Hz
#define PERIODE_ASSERV	1000000/FREQ_ASSERV	//us
//ASSERVISSEMENT
//CONSTANTES & PID
float kp=4.8;
float ki=0.8;
float kd=6;

volatile double output=0;
volatile double setPoint=0;
volatile double error=0;
volatile double last_error=0;
volatile double derivative_error=0;
volatile double integral_error=0;
volatile int32_t last_input=0;
int32_t espilon_output=0;

int32_t PID_compute(int32_t input){
	error=setPoint-input;
	derivative_error=error-last_error;
	integral_error+=ki*error;
	last_input=input;
	last_error=error;
	//Anti windup
	if(integral_error>MAX_PWM) integral_error=MAX_PWM;
	else if(integral_error<-MAX_PWM) integral_error=-MAX_PWM;
	if(error==0 && last_error==0){
		integral_error=0;
	}
	output=kp*error+integral_error+kd*derivative_error;
	//Limitation à des valeurs 8bits
	if(output>MAX_PWM) output=MAX_PWM;
	else if(output<-MAX_PWM) output=-MAX_PWM;
	return (int32_t)output;
}

void set_tuning(float* constant){
	Serial.println("Enter constant value");
	Serial.setTimeout(5000);
	*constant=Serial.parseFloat();
	Serial.setTimeout(50);
	Serial.print("New value:");Serial.println(*constant);
}
//CONTROL MOTEUR

enum DIRECTION{
	CLOCKWISE,
	COUNTER_CLOCKWISE
};
void motor_set_direction(DIRECTION dir){
	if(dir==CLOCKWISE){
		digitalWrite(LED_BUILTIN, HIGH);
		digitalWrite(INA, HIGH);
		digitalWrite(INB, LOW);
	}
	else{
		digitalWrite(LED_BUILTIN, LOW);
		digitalWrite(INA, LOW);
		digitalWrite(INB, HIGH);
	}
}

void motor_run(int32_t pwm){
	if(pwm<0){
		motor_set_direction(CLOCKWISE);
	}
	else{
		motor_set_direction(COUNTER_CLOCKWISE);
	}
	pwm=abs(pwm);
	pwm=map(pwm,0,127,0,MAX_PWM);
//	if(pwm>MAX_PWM) pwm=MAX_PWM;
	analogWrite(PWM, (uint16_t)pwm);
}

IntervalTimer timer;
void asservissement(){
	motor_run(PID_compute(encoder_pos));
}

void motor_init(){
	pinMode(INA,OUTPUT);
	pinMode(INB,OUTPUT);
	pinMode(PWM,OUTPUT);
	pinMode(SEL0,OUTPUT);
	pinMode(CS,INPUT);
	digitalWrite(INA,LOW);
	digitalWrite(INB,LOW);
	digitalWrite(SEL0,LOW);
	digitalWrite(PWM,LOW);
	analogWriteRes(8);
	analogWriteFrequency(PWM, 20000);
	analogWrite(PWM, 0);
	timer.begin(asservissement, PERIODE_ASSERV);
}


//Utilitaires
void motor_print_status(){
	Serial.print(millis());	Serial.print(' ');
	Serial.print(setPoint);Serial.print(' ');
	Serial.print(encoder_pos);Serial.print(' ');
	Serial.print(output);Serial.print(' ');
	Serial.print(derivative_error);Serial.print(' ');
	Serial.println(integral_error);
}

void motor_reset_pos(){
	Serial.println("Reseting position and PID");
	noInterrupts();
	encoder_pos=0;
	setPoint = 0;
	last_error = 0;
	derivative_error = 0;
	integral_error = 0;
	interrupts();
}

void PID_set_setpoint(int32_t new_setpoint){
	setPoint=new_setpoint;
}


void PID_read_setpoint(){
	Serial.println("Enter SetPoint");
	Serial.setTimeout(5000);
	PID_set_setpoint(Serial.parseInt());
	Serial.setTimeout(50);
	Serial.println(setPoint);
}





#endif //TEENSYLC_PID_MOTOR_HPP
