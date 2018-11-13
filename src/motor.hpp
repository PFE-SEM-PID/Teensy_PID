#ifndef TEENSYLC_PID_MOTOR_HPP
#define TEENSYLC_PID_MOTOR_HPP

#include <Arduino.h>
#include <IntervalTimer.h>
#include "encoder.hpp"
#include "pid.hpp"


//ASSERVISSEMENT
//CONSTANTES & PID
float pos_kp=1.63;
float pos_ki=0.2;
float pos_kd=1.4;
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
	analogWrite(PWM, (uint16_t)pwm);
}


IntervalTimer timer;
void asservissement(){
	motor_run(static_cast<int32_t>(position_pid.compute(encoder.read())));
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
		Serial.print(encoder.read());
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
		Serial.print(encoder.read());
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
	encoder.write(0);
	speed_pid.reset_errors();
	speed_pid.set_set_point(0);
	position_pid.reset_errors();
	position_pid.set_set_point(0);
	interrupts();
}

void PID_set_position_setpoint(int32_t new_setpoint){
	position_pid.set_set_point(new_setpoint);
}

void PID_increment_setpoint(){
	Serial.setTimeout(5000);
	PID_set_position_setpoint(static_cast<int32_t>(position_pid.get_setpoint())+Serial.parseInt());
	Serial.setTimeout(50);
}

void PID_read_setpoint(){
	Serial.setTimeout(5000);

	PID_set_position_setpoint(Serial.parseInt());
	Serial.setTimeout(50);
}

#endif //TEENSYLC_PID_MOTOR_HPP