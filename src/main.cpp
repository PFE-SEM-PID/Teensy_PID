#include <Arduino.h>
#include <IntervalTimer.h>

//Serial
#define SERIAL_RX_BUFFER_SIZE	64
bool recording = false;
#define ticks_per_turn 100
#define MAX_PWM	127

//ENCODER SETTINGS
#define CHANA 2
#define CHANB 3
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
#define INA 		4
#define INB 		5
#define PWM 	6
#define SEL0 	11
#define CS 		12
volatile double output=0;
volatile double setPoint=0;
volatile double error=0;
volatile double last_error=0;
volatile double derivative_error=0;
volatile double integral_error=0;
volatile int32_t last_input=0;
int32_t espilon_output=0;

float kp=12;
float ki=1;
float kd=10;

int32_t compute_PID(int32_t input){
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

/*
 * MOTOR CONTROL
 */

enum DIRECTION{
	CLOCKWISE,
	COUNTER_CLOCKWISE
};

void set_direction(DIRECTION dir){
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
bool inline read_char(int8_t & buffer)
{
	buffer = (int8_t)Serial.read();
	return (buffer != 10);	//Stop si retour à la ligne ou 0
}

//Lit une chaine dans order
bool read_string(){
	if(Serial.available()>0){
		memset(order, 0, sizeof(order));
		int8_t c=0;
		uint8_t i=0;
		while(read_char(c) && i<SERIAL_RX_BUFFER_SIZE){
			if(c!=-1 && c!=0) {
				order[i] = c;
				i++;
			}
		}
		if(Serial.peek()==10) {//On vire le retour à la ligne
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
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(INA,OUTPUT);
	pinMode(INB,OUTPUT);
	pinMode(PWM,OUTPUT);
	pinMode(SEL0,OUTPUT);
	pinMode(CS,INPUT);
	digitalWrite(LED_BUILTIN, LOW);
	digitalWrite(INA,LOW);
	digitalWrite(INB,LOW);
	digitalWrite(SEL0,LOW);
	digitalWrite(PWM,LOW);
	// encoder setup
	pinMode(CHANA,INPUT);
	pinMode(CHANB,INPUT);

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

void set_tuning(float* constant){
	Serial.println("Enter constant value");
	Serial.setTimeout(5000);
	*constant=Serial.parseFloat();
	Serial.setTimeout(50);
	Serial.print("New value:");Serial.println(*constant);
}

void loop() {
	static uint32_t last_print=millis();
	if(recording && millis()-last_print>10) {
		Serial.print(millis());
		Serial.print(setPoint);
		Serial.println(encoder_pos);
		Serial.print(output);
		Serial.print(derivative_error);
		Serial.print(integral_error);
		last_print=millis();
	}
	if(read_string()) {
		Serial.print("ORDER ");Serial.println(order);
		if (compare_strings(order, "r")) {//reset
			Serial.println("Reseting position and PID");
			noInterrupts();
			encoder_pos=0;
			setPoint = 0;
			last_error = 0;
			derivative_error = 0;
			integral_error = 0;
			interrupts();
		} else if (compare_strings(order, "p")) {
			Serial.println("Enter SetPoint");
			Serial.setTimeout(5000);
			setPoint = Serial.parseInt();
			Serial.setTimeout(50);
			Serial.println(setPoint);
		}
		else if(compare_strings(order, "kp")){
			set_tuning(&kp);
		}
		else if(compare_strings(order, "ki")){
			set_tuning(&ki);
		}
		else if(compare_strings(order, "kd")){
			set_tuning(&kd);
		}
		else if(compare_strings(order, "print")){
			Serial.print("Kp=");Serial.println(kp);
			Serial.print("Ki=");Serial.println(ki);
			Serial.print("Kd=");Serial.println(kd);
		}
		else if(compare_strings(order, "start")){
			recording=true;
		}
		else if(compare_strings(order, "stop")){
			recording=false;
		}
	}
}