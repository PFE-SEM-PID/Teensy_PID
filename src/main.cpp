#include <Arduino.h>

#include "encoder.hpp"
#include "motor.hpp"
#include "com.hpp"

void setup() {
	Serial.begin(9600);
	// pinMode H bridge
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	motor_init();
	encoder_init();

	Serial.println("SETUP OK");
}

void loop() {
	static uint32_t last_print=millis();
	if(recording && millis()-last_print>10) {
		motor_print_status();
		last_print=millis();
	}
	if(read_string()) {
		Serial.print("ORDER ");Serial.println(order);
		if (compare_strings(order, "r")) {//reset
			motor_reset_pos();
		} else if (compare_strings(order, "p")) {
			PID_read_setpoint();
		}
		else if(compare_strings(order, "pos_kp")){
			set_tuning(&pos_kp);
		}
		else if(compare_strings(order, "ki")){
			set_tuning(&pos_ki);
		}
		else if(compare_strings(order, "kd")){
			set_tuning(&pos_kd);
		}
		else if(compare_strings(order, "print")){
			Serial.print("Kp=");Serial.println(pos_kp);
			Serial.print("Ki=");Serial.println(pos_ki);
			Serial.print("Kd=");Serial.println(pos_kd);
		}
		else if(compare_strings(order, "start")){
			recording=true;
		}
		else if(compare_strings(order, "stop")){
			recording=false;
		}
	}

	static uint32_t moving_time=millis();
	static bool state=false;
	if(millis()-moving_time>100000000){
		if(state) {
			PID_set_position_setpoint(25);
			state=false;
		}
		else{
			PID_set_position_setpoint(-25);
			state=true;
		}
		moving_time=millis();
	}
}