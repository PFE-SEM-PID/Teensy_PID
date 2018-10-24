#ifndef TEENSYLC_PID_MOTOR_HPP
#define TEENSYLC_PID_MOTOR_HPP


#include <Arduino.h>
#include <IntervalTimer.h>
#include "encoder.hpp"
#include "pid.hpp"
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
float pos_kp=4;
float pos_ki=1;
float pos_kd=7;
float speed_kp=0.2;
float speed_ki=0.08;
float speed_kd=0.01;

bool speed_controlled=false;

PID position_pid(&pos_kp, &pos_ki, &pos_kd);
PID speed_pid(&speed_kp, &speed_ki, &speed_kd);

void set_tuning(float* constant){
//	Serial.println("Enter constant value");
	Serial.setTimeout(5000);
	*constant=Serial.parseFloat();
	Serial.setTimeout(50);
//	Serial.print("New value:");Serial.println(*constant);
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
//	pwm=map(pwm,0,127,0,MAX_PWM);
//	if(pwm>MAX_PWM) pwm=MAX_PWM;
	analogWrite(PWM, (uint16_t)pwm);
}


IntervalTimer timer;
void asservissement(){
	if(speed_controlled) {
		static int32_t last_pos=encoder_pos;
		double speed=(encoder_pos-last_pos)*FREQ_ASSERV;
		last_pos=encoder_pos;
		motor_run(static_cast<int32_t>(speed_pid.compute(speed)));
	}
	else{
		motor_run(static_cast<int32_t>(position_pid.compute(encoder_pos)));
	}
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
	if(speed_controlled) {
		Serial.print(speed_pid.get_setpoint());
		Serial.print(' ');
		Serial.print(encoder_pos);
		Serial.print(' ');
		Serial.print(speed_pid.get_output());
		Serial.print(' ');
		Serial.print(speed_pid.get_derivative_error());
		Serial.print(' ');
		Serial.println(speed_pid.get_integral_error());
	}
	else{
		Serial.print(position_pid.get_setpoint());
		Serial.print(' ');
		Serial.print(encoder_pos);
		Serial.print(' ');
		Serial.print(position_pid.get_output());
		Serial.print(' ');
		Serial.print(position_pid.get_derivative_error());
		Serial.print(' ');
		Serial.println(position_pid.get_integral_error());
	}
}

void motor_reset_pos(){
//	Serial.println("Reseting position and PID");
	noInterrupts();
	encoder_pos=0;
	if(speed_controlled){
		speed_pid.reset_errors();
		speed_pid.set_set_point(0);
	}
	else{
		position_pid.reset_errors();
		position_pid.set_set_point(0);
	}
	interrupts();
}

void PID_set_position_setpoint(int32_t new_setpoint){
	position_pid.set_set_point(new_setpoint);
}


void PID_read_setpoint(){
	Serial.setTimeout(5000);
	PID_set_position_setpoint(Serial.parseInt());
	Serial.setTimeout(50);
}





#endif //TEENSYLC_PID_MOTOR_HPP
