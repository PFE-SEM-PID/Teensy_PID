#include <Arduino.h>
#include <IntervalTimer.h>

#define SERIAL_RX_BUFFER_SIZE	64
#define INA 		4
#define INB 		5
#define PWM 	6
#define SEL0 	11
#define CS 		12
#define LED 		13

#define CHANA 2
#define CHANB 3

#define TIQUETAQUEPARTOURE 100
#define MAX_PWM	127
//ENCODER SETTINGS

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


//PID SETTINGS AND VALUES
volatile double output=0;
volatile double setPoint=0;
volatile double last_error=0;
volatile double derivative_error=0;
volatile double integral_error=0;
int32_t espilon_output=0;

float kp=5; //Oscilaltions à 3.5+ -> *1/2
float ki=0.1;
float kd=1;

int32_t compute_PID(int32_t input){
	int32_t error=(int32_t)(setPoint-input);
	derivative_error=error-last_error;
	integral_error+=ki*error;
	//Anti windup
	if(integral_error>MAX_PWM) integral_error=MAX_PWM;
	else if(integral_error<-MAX_PWM) integral_error=-MAX_PWM;
	if(derivative_error==0 && error<2){
		integral_error=0;
	}
	last_error=error;
	int32_t result=(int32_t)(kp*error+integral_error+kd*derivative_error);
	//Limitation à des valeurs 8bits
	if(result>MAX_PWM) result=MAX_PWM;
	else if(result<-MAX_PWM) result=-MAX_PWM;
	output=result;
	return (int32_t)output;
}

/*
 * MOTOR CONTROL
 */

enum DIRECTION{
	CLOCKWISE,
	COUNTER_CLOCKWISE
};

void set_direction(DIRECTION dir){
	if(dir==CLOCKWISE){
		digitalWrite(LED, HIGH);
		digitalWrite(INA, HIGH);
		digitalWrite(INB, LOW);
	}
	else{
		digitalWrite(LED, LOW);
		digitalWrite(INA, LOW);
		digitalWrite(INB, HIGH);
	}
}

void run_motor(int32_t pwm){
	if(pwm<0){
		set_direction(CLOCKWISE);
	}
	else{
		set_direction(COUNTER_CLOCKWISE);
	}
	pwm=abs(pwm);
	pwm=map(pwm,0,127,0,MAX_PWM);
//	if(pwm>MAX_PWM) pwm=MAX_PWM;
	analogWrite(PWM, (uint16_t)pwm);
}

/*
 * COMMUNICATION
 */

char order[SERIAL_RX_BUFFER_SIZE];

//Retourne vrai si les deux chaines sont égales
bool inline compare_strings(const char *first, const char *second){
	return !strcmp(first, second);
}

//Lit un char et l'enregistre dans buffer, vérifie si il vaut \r ou \n
bool inline read_char(char & buffer)
{
	buffer = Serial.read();
//	Serial.println((int)buffer);
	return (buffer != 10 && buffer!=0);
}

//Lit une chaine dans order
bool read_string(){
	if(Serial.available()>0){
		memset(order, 0, sizeof(order));
//		Serial.print("READING: ");
		char c=0;
		uint8_t i=0;
		while(read_char(c) && i<SERIAL_RX_BUFFER_SIZE){
			if(c!=-1) {
				order[i] = c;
				i++;
			}
		}
		if(Serial.peek()==10) {
			Serial.read();
		}
		return !compare_strings(order, "");
	}
	return false;
}
IntervalTimer timer;
void asservissement(){
	run_motor(compute_PID(encoder_pos));
}

void setup() {
	Serial.begin(9600);
	// pinMode H bridge
	pinMode(LED, OUTPUT);
	pinMode(INA,OUTPUT);
	pinMode(INB,OUTPUT);
	pinMode(PWM,OUTPUT);
	pinMode(SEL0,OUTPUT);
	pinMode(CS,INPUT);
	digitalWrite(LED, LOW);
	digitalWrite(INA,LOW);
	digitalWrite(INB,LOW);
	digitalWrite(SEL0,LOW);
	digitalWrite(PWM,LOW);
	// encoder setup
	pinMode(CHANA,INPUT_PULLUP);
	pinMode(CHANB,INPUT_PULLUP);

	digitalWrite(INA,HIGH);
	digitalWrite(INB,LOW);
	digitalWrite(SEL0,LOW);
	attachInterrupt(digitalPinToInterrupt(CHANA), readCodeuse, RISING);
	Serial.println("SETUP OK");
	timer.begin(asservissement, 10000);
	analogWriteRes(8);
	analogWriteFrequency(PWM, 20000);
//	Serial.setTimeout(5000);
}



void loop() {
	static uint32_t last_update=millis();
	if(millis()-last_update>1000) {
//		run_motor(compute_PID(encoder_pos));
		Serial.print("SETPOINT: ");Serial.print(setPoint);
		Serial.print(" PWM: ");Serial.println(output);
		Serial.print("DERIV: ");Serial.print(derivative_error);
		Serial.print(" INTEG: ");Serial.println(integral_error);
		last_update=millis();
	}
	static uint32_t last_print=millis();
	if(millis()-last_print>1000) {
		Serial.print("POS ");
		Serial.println(encoder_pos);
		last_print=millis();
	}
	if(read_string()) {
		Serial.print("ORDER ");Serial.println(order);
		if (compare_strings(order, "r")) {//reset
			Serial.println("Reseting position and PID");
			setPoint = encoder_pos;
			last_error = 0;
			derivative_error = 0;
			integral_error = 0;
		} else if (compare_strings(order, "p")) {
			Serial.println("Enter SetPoint");
			Serial.setTimeout(5000);
			setPoint = Serial.parseInt();
			Serial.setTimeout(50);
			Serial.println(setPoint);
		}
		else if(compare_strings(order, "kp")){
			Serial.println("Enter Kp");
			Serial.setTimeout(10000);
			kp= Serial.parseFloat();
			Serial.setTimeout(50);
			Serial.println(setPoint);
		}
		else if(compare_strings(order, "ki")){
			Serial.println("Enter Ki");
			Serial.setTimeout(10000);
			ki= Serial.parseFloat();
			Serial.setTimeout(50);
			Serial.println(setPoint);
		}
		else if(compare_strings(order, "kd")){
			Serial.println("Enter Kd");
			Serial.setTimeout(10000);
			kd= Serial.parseFloat();
			Serial.setTimeout(50);
			Serial.println(setPoint);
		}
	}
}