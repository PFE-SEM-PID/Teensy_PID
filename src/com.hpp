#ifndef TEENSYLC_PID_COM_HPP
#define TEENSYLC_PID_COM_HPP
#include <Arduino.h>
//Serial
#define SERIAL_RX_BUFFER_SIZE	64
bool recording = false;
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

#endif //TEENSYLC_PID_COM_HPP
