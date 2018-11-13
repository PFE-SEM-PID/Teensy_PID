#include <Arduino.h>

#include "encoder.hpp"
#include "motor.hpp"
#include "com.hpp"

void setup() {
	Serial.begin(9600);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	motor_init();
}

void loop() {
	static uint32_t last_print=millis();
	if(recording && millis()-last_print>=5) {
		motor_print_status();
		last_print=millis();
	}
	if(read_string()) {
		Serial.print("ORDER ");Serial.println(order);
		if (compare_strings(order, "r")) {//reset
			motor_reset_pos();
		} else if (compare_strings(order, "i")) {
			PID_read_setpoint();
		} else if (compare_strings(order, "p")){
			PID_increment_setpoint();
		} else if(compare_strings(order, "kp")){
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
}