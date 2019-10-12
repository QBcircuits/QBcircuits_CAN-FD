/***************************************************
  This is our library for the QBcircuits Arduino Pro mini/micro CAN FD shield
  ----> https://www.qbcircuits.com

  Check out the link above for our tutorials, wiring diagrams
  QBcircuits invests time and effort providing this open source code,
  please support QBcircuits and open-source hardware by purchasing
  products from QBcircuits - GoLong!

  Written by Brother Q for QBcircuits
  All text above must be included in any redistribution
 ****************************************************/

#include "qb_obd2.h"
/**************************************************************************************************
Purpose: 	Performs OBD2 service 0x01 PID data algorithms & writes to the Arduino serial port
Inputs:		*ptrBuf_u8	- intended as pointer to the chnCan struct rxData member that has 
						  service 0x01 PID response data packet 
						  ptrBuf_u8[0] - #of additional bytes
						  ptrBuf_u8[1] - service 0x01 + 0x40
						  ptrBuf_u8[2] - service 0x01 PID
						  ptrBuf_u8[3] - service 0x01 data byte A
						  ptrBuf_u8[4] - service 0x01 data byte B
						  ptrBuf_u8[5] - service 0x01 data byte C
						  ptrBuf_u8[6] - service 0x01 data byte D
						  ptrBuf_u8[7] - service 0x01 data byte E

Outputs:	serial writes
**************************************************************************************************/	
void obd2s1PidDecrypt(uint8_t *ptrBuf_u8){	
	uint8_t j, pid_u8, A, B, C, D, E;
	char pCent2[3] = " %";
	char pCent4[5] = " %, ";
	
	pid_u8 = ptrBuf_u8[2];
	A 	   = ptrBuf_u8[3];
	B 	   = ptrBuf_u8[4];
	C 	   = ptrBuf_u8[5];
	D 	   = ptrBuf_u8[6];
	E 	   = ptrBuf_u8[7];
	
	Serial.print("Data [");
	for(j=0;j<8;j++){
		if(ptrBuf_u8[j] < 0x10){Serial.print("0");}
		Serial.print(ptrBuf_u8[j], HEX);
		if(j!=7){Serial.print(" ");}
	}
	Serial.print("] ");
	
	if(pid_u8 == 0x04){
		Serial.print(A/255 * 100); 
	}
	else if(pid_u8 == 0x05){
		Serial.print(A - 40); 
	}	
	else if(pid_u8 >= 0x06 && pid_u8 <= 0x09){
		Serial.print(A/1.28 - 100); 
	}
	else if(pid_u8 == 0x0A){
		Serial.print(A * 3); 
	}
	else if(pid_u8 == 0x0B){
		Serial.print(A); 
	}
	else if(pid_u8 == 0x0C){
		Serial.print(A * 64 + 0.25 * B); Serial.print(" rpm");
	}
	else if(pid_u8 == 0x0D){
		Serial.print(A); Serial.print(" km/h");
	}
	else if(pid_u8 == 0x0E){
		Serial.print(A>>2 - 64); Serial.print(" degrees before TDC");
	}
	else if(pid_u8 == 0x0F){
		Serial.print(A - 40); 
	}
	else if(pid_u8 == 0x10){
		Serial.print((256*A + B)/100); Serial.print(" grams/s");
	}
	else if(pid_u8 == 0x11){
		Serial.print(A/255 * 100); 
	}
	else if(pid_u8 >= 0x14 && pid_u8 <= 0x1B){
		if(B != 0xFF){
		  Serial.print(A/200); Serial.print(" V, ");
		  Serial.print(B*100/128 - 100); Serial.print(pCent2);
		}
	}
	else if(pid_u8 == 0x1F){
		Serial.print(A*256 + B); Serial.print(" seconds");
	}
	else if(pid_u8 == 0x21){
		Serial.print(A*256 + B); Serial.print(" km");
	}
	else if(pid_u8 == 0x22){
		Serial.print(0.079*(A*256 + B)); 
	}
	else if(pid_u8 == 0x23){
		Serial.print(10*(A*256 + B)); 
	}
	else if(pid_u8 >= 0x24 && pid_u8 <= 0x2B){
		Serial.print(2/65536*(A*256 + B)); Serial.print(" ratio, ");
		Serial.print(2/65536*(C*256 + D)); Serial.print(" V");
	}
	else if(pid_u8 == 0x2C){
		Serial.print(A/255 * 100); 
	}
	else if(pid_u8 == 0x2D){
		Serial.print(A/128 * 100 - 100); 
	}
	else if(pid_u8 == 0x2E){
		Serial.print(A/255 * 100); 
	}
	else if(pid_u8 == 0x2F){
		Serial.print(A/255 * 100); 
	}
	else if(pid_u8 == 0x30){
		Serial.print(A); Serial.print(" count");
	}
	else if(pid_u8 == 0x31){
		Serial.print(A*256 + B); Serial.print(" km");
	}
	else if(pid_u8 == 0x32){
		Serial.print(0.25*(A*256 + B)); Serial.print(" Pa");
	}
	else if(pid_u8 == 0x33){
		Serial.print(A); 
	}
	else if(pid_u8 >= 0x34 && pid_u8 <= 0x3B){
		Serial.print(2/65536*(A*256 + B)); Serial.print(" ratio, ");
		Serial.print(1/65536*(C*256 + D)-128); Serial.print(" mA");
	}
	else if(pid_u8 >= 0x3C && pid_u8 <= 0x3F){
		Serial.print(1/10*(A*256 + B)-40); 
	}
	else if(pid_u8 == 0x42){
		Serial.print(1/1000*(A*256 + B)); Serial.print(" V");
	}
	else if(pid_u8 == 0x43){
		Serial.print(100/256*(A*256 + B)); 
	}
	else if(pid_u8 >= 0x44 && pid_u8 <= 0x3B){
		Serial.print(2/65536*(A*256 + B)); Serial.print(" ratio, ");
	}
	else if(pid_u8 == 0x45){
		Serial.print(A/255 * 100); 
	}
	else if(pid_u8 == 0x46){
		Serial.print(A - 40); 
	}
	else if(pid_u8 >= 0x47 && pid_u8 <= 0x4C){
		Serial.print(A/255 * 100); 
	}
	else if(pid_u8 >= 0x4D && pid_u8 <= 0x4E){
		Serial.print(A*256 + B); Serial.print(" minutes");
	}
	else if(pid_u8 == 0x4F){
		Serial.print(A); Serial.print(" ratio, ");
		Serial.print(B); Serial.print(" V, ");
		Serial.print(C); Serial.print(" mA, ");
		Serial.print(D*10); Serial.print(" kPa");
	}
	else if(pid_u8 == 0x50){
		Serial.print(A*10); Serial.print(" g/s");
	}
	else if(pid_u8 == 0x52){
		Serial.print(A/255 * 100); 
	}
	else if(pid_u8 == 0x53){
		Serial.print(1/200*(A*256 + B)); 
	}
	else if(pid_u8 == 0x54){
		Serial.print((A*256 + B)-32767); 
	}
	else if(pid_u8 >= 0x55 && pid_u8 <= 0x58){
		Serial.print(A*100/128 - 100); Serial.print(pCent4);
		Serial.print(B*100/128 - 100); Serial.print(pCent2);
	}
	else if(pid_u8 == 0x59){
		Serial.print(10*(A*256 + B)); 
	}
	else if(pid_u8 >= 0x5A && pid_u8 <= 0x5B){
		Serial.print(A/255 * 100); 
	}
	else if(pid_u8 == 0x5C){
		Serial.print(A - 40); 
	}
	else if(pid_u8 == 0x5D){
		Serial.print(1/128*(A*256 + B)-210); Serial.print(" degree");
	}
	else if(pid_u8 == 0x5E){
		Serial.print(1/20*(A*256 + B)); Serial.print(" L/h");
	}
	else if(pid_u8 >= 0x61 && pid_u8 <= 0x62){
		Serial.print(A-125); 
	}
	else if(pid_u8 == 0x63){
		Serial.print((A*256 + B)); Serial.print(" Nm");
	}
	else if(pid_u8 == 0x64){
		Serial.print(A-125); Serial.print(pCent4);
		Serial.print(B-125); Serial.print(pCent4);
		Serial.print(C-125); Serial.print(pCent4);
		Serial.print(D-125); Serial.print(pCent4);
		Serial.print(E-125); Serial.print(pCent2);
	}
	else if(pid_u8 == 0x8E){
		Serial.print(A-125); 
	}

	if  (pid_u8 == 0x04 || (pid_u8 >= 0x06 && pid_u8 <= 0x09) || pid_u8 == 0x11 || 
		(pid_u8 >= 0x2C && pid_u8 <= 0x2F) || pid_u8 == 0x43 || pid_u8 == 0x45 || 
		 pid_u8 == 0x47 || pid_u8 == 0x52 || (pid_u8 >= 0x5A && pid_u8 <= 0x5B) ||
		(pid_u8 >= 0x61 && pid_u8 <= 0x62) || pid_u8 == 0x8E)
		Serial.print(pCent2);

	else if  (pid_u8 == 0x05 || pid_u8 == 0x0F || (pid_u8 >= 0x3C && pid_u8 <= 0x3F) || 
			  pid_u8 == 0x46 || pid_u8 == 0x5C)
		Serial.print(" C");
		
	else if  ((pid_u8 >= 0x0A && pid_u8 <= 0x0B) || (pid_u8 >= 0x22 && pid_u8 <= 0x23) || 
			  pid_u8 == 0x33 || (pid_u8 >= 0x53 && pid_u8 <= 0x54) || pid_u8 == 0x59)
		Serial.print(" kPa");	

	
	Serial.println();
}











