//
// Created by tic-tac on 10/23/18.
//

#ifndef TEENSYLC_PID_PID_HPP
#define TEENSYLC_PID_PID_HPP
#include "defines.hpp"
#include "Arduino.h"

class PID{
	volatile double output=0;
	volatile double setpoint=0;
	volatile double error=0;
	volatile double last_error=0;
	volatile double derivative_error=0;
	volatile double integral_error=0;
	volatile double last_input=0;
public:
	PID(float p_kp, float p_ki, float p_kd):kp(p_kp), ki(p_ki), kd(p_kd){}

	double compute(double input){
		error=setpoint-input;
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
		return output;
	}

	void reset_errors(){
		error=derivative_error=last_error=integral_error=0;
	}

	void set_set_point(double value){
		setpoint=value;
	}

	float kp=4;
	float ki=1;
	float kd=7;

	double get_setpoint() {
		return setpoint;
	}

	double get_output() {
		return output;
	}

	double get_derivative_error() {
		return derivative_error;
	}

	double get_integral_error() {
		return integral_error;
	}
};
#endif //TEENSYLC_PID_PID_HPP
