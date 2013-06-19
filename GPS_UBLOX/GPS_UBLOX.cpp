/*
	GPS_UBLOX.cpp - Ublox GPS library for Arduino
	
	This code works with boards based on AtMega168/328 and AtMega1280/2560 (Serial port 1)
	Written by Simon Tsaoussis
	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	GPS configuration : Ublox protocol
	Baud rate : 38400
	Active messages : 
		NAV-POSLLH Geodetic Position Solution, PAGE 66 of datasheet
		NAV-VELNED Velocity Solution in NED, PAGE 71 of datasheet
		NAV-STATUS Receiver Navigation Status
		  or 
		NAV-SOL Navigation Solution Information

	Methods:
		Init() : GPS Initialization
		Read() : Call this funcion as often as you want to ensure you read the incomming gps data
		
	Properties:
		Lattitude : Lattitude * 10000000 (long value)
		Longitude : Longitude * 10000000 (long value)
		Altitude :  Altitude * 100 (meters) (long value)
		Ground_speed : Speed (m/s) * 100 (long value)
		Ground_course : Course (degrees) * 100 (long value)
		NewData : 1 when a new data is received.
		          You need to write a 0 to NewData when you read the data
		Fix : 1: GPS FIX, 0: No Fix (normal logic)
		ch: 0 or 1
		flags
		count
		wnR
		wnF
		towMsR
		towMsF
		towSubMsF
		accEst
			
*/

#include "GPS_UBLOX.h"

#include <avr/interrupt.h>
#include "Arduino.h"


// Constructors ////////////////////////////////////////////////////////////////
GPS_UBLOX_Class::GPS_UBLOX_Class()
{
}


// Public Methods //////////////////////////////////////////////////////////////
void GPS_UBLOX_Class::Init(void)
{

	int hour = 0;
	int minute = 0;
	int sow = 0;
	int second = 0;
	int millisecond = 0;
	int microsecond = 0;
	int nanosecond = 0;
	int week_year = 0;
	int days = 0;
	int month = 0;
	int weeks = 0;
	int year = 0;
	PosData = 0;
	ck_a=0;
	ck_b=0;
	UBX_step=0;
	NewData=0;
	Fix=0;
	PrintErrors=0;
	GPS_timer=millis();   //Restarting timer...
	// Initialize serial port
	#if defined(__AVR_ATmega1280__)
		Serial1.begin(38400);         // Serial port 1 on ATMega1280
	#else
		Serial.begin(38400);
	#endif
}

void GPS_UBLOX_Class::Read(void)
{
  static unsigned long GPS_timer=0;
  byte data;
  int numc;
  
  #if defined(__AVR_ATmega1280__) || (__AVR_ATmega2560__)    // If AtMega1280 then Serial port 1...
	numc = Serial1.available();
  #else
	numc = Serial.available();
  #endif
  if (numc > 0)
    for (int i=0;i<numc;i++)  // Process bytes received
      {
	  #if defined(__AVR_ATmega1280__) || (__AVR_ATmega2560__)
        data = Serial1.read();
      #else
		data = Serial.read();
	  #endif
      switch(UBX_step)    
      {
      case 0:  
        if(data==0xB5)  
          UBX_step++;   
        break; 
      case 1:  
        if(data==0x62)  
          UBX_step++;   
        else 
          UBX_step=0;     
        break;
      case 2:
        UBX_class=data;
        ubx_checksum(UBX_class);
        UBX_step++;
        break;
      case 3:
        UBX_id=data;
        ubx_checksum(UBX_id);
        UBX_step++;
        break;
      case 4:
        UBX_length_hi=data;
        ubx_checksum(UBX_length_hi);
        UBX_step++;

		if (UBX_length_hi>=UBX_MAX_SIZE)
        {
		  if (PrintErrors)
			Serial.println("ERR:GPS_BAD_PAYLOAD_LENGTH!!");          
          UBX_step=0;   
          ck_a=0;
          ck_b=0;
        }
        break;
      case 5:
        UBX_length_lo=data;
        ubx_checksum(UBX_length_lo);
        UBX_step++;
		UBX_counter=0;
        break;
      case 6:         
	if (UBX_counter < UBX_length_hi)  
        {
          UBX_buffer[UBX_counter] = data;
          ubx_checksum(data);
          UBX_counter++;
          if (UBX_counter==UBX_length_hi)
            UBX_step++;
        }
        break;
      case 7:
        UBX_ck_a=data;  
	//	Serial.println(UBX_ck_a);
        UBX_step++;
        break;
      case 8:
        UBX_ck_b=data;   
    //   Serial.println(UBX_ck_b);
	
        if((ck_a==UBX_ck_a)&&(ck_b==UBX_ck_b))   
	  		parse_ubx_gps();       
        else
		  {
		  if (PrintErrors)
			Serial.println("ERR:GPS_CHK!!");
		  }
        UBX_step=0;
        ck_a=0;
        ck_b=0;
        GPS_timer=millis();
        break;
	  }
    }    
  if ((millis() - GPS_timer)>2000)
    {
	Fix = 0;
	if (PrintErrors)
	  Serial.println("ERR:GPS_TIMEOUT!!");
    }
}

void GPS_UBLOX_Class::Poll(byte hexa, byte hexb, byte hexc, byte hexd) 
{
byte data;

//Serial.print(hexa, HEX);
//Serial.print(hexb, HEX);
ubx_checksum(hexc);
//Serial.print(hexc, HEX);
ubx_checksum(hexd);
//Serial.print(hexd, HEX);
//Serial.print(ck_a, HEX);
//Serial.println(ck_b, HEX);
ck_a = 0;
ck_b = 0;
}
/****************************************************************
 * 
 ****************************************************************/
// Private Methods //////////////////////////////////////////////////////////////
void GPS_UBLOX_Class::parse_ubx_gps(void)
{
  int j;
  switch(UBX_class){
  case 0x01:
  
    switch(UBX_id)//Checking the UBX ID
    {
    case 0x02: //ID NAV-POSLLH 
      j=0;
      Time = join_4_bytes(&UBX_buffer[j]); // ms Time of week
      j+=4;
      Longitude = join_4_bytes(&UBX_buffer[j]); // lon*10000000
      j+=4;
      Lattitude = join_4_bytes(&UBX_buffer[j]); // lat*10000000
      j+=4;
      Altitude = join_4_bytes(&UBX_buffer[j]);  // elipsoid heigth mm
      j+=4;
      //Altitude = (float)join_4_bytes(&UBX_buffer[j]);  // MSL heigth mm
      NewData=1;
      break;
    case 0x03://ID NAV-STATUS 
      //if(UBX_buffer[4] >= 0x03)
	  if((UBX_buffer[4] >= 0x03)&&(UBX_buffer[5]&0x01))        
        Fix=1; //valid position        
      else
        Fix=0; //invalid position
      break;

    case 0x06://ID NAV-SOL
      if((UBX_buffer[10] >= 0x03)&&(UBX_buffer[11]&0x01))
        Fix=1; //valid position
      else
        Fix=0; //invalid position        
      UBX_ecefVZ=join_4_bytes(&UBX_buffer[36]);  //Vertical Speed in cm/s
      NumSats=UBX_buffer[47];                    //Number of sats...     
      break;

    case 0x12:// ID NAV-VELNED 
      j=16;
      Speed_3d = join_4_bytes(&UBX_buffer[j]); // cm/s
      j+=4;
      Ground_Speed = join_4_bytes(&UBX_buffer[j]); // Ground speed 2D cm/s
      j+=4;
      Ground_Course = join_4_bytes(&UBX_buffer[j]); // Heading 2D deg*100000
      Ground_Course /= 1000;	// Rescale heading to deg * 100
      j+=4;
      break; 
      }
	//  UBX_class = 0x0D;
	PosData = 1;
    break;   
	//Serial.println(UBX_class);
	case 0x0D: 
	//UBX_id = 0x03;
	    switch(UBX_id)//Checking the UBX ID
    {
	    case 0x03: //ID TIM-TM2 
	ch = one_byte(&UBX_buffer[0]);
	flags = one_byte(&UBX_buffer[1]);	
	count = join_2_bytes(&UBX_buffer[2]);
	wnR = join_2_bytes(&UBX_buffer[4]);
	wnF = join_2_bytes(&UBX_buffer[6]);
	towMsR = join_4_bytes(&UBX_buffer[8]);
	towSubMsR = join_4_bytes(&UBX_buffer[12]);
	towMsF = join_4_bytes(&UBX_buffer[16]);
	towSubMsF = join_4_bytes(&UBX_buffer[20]);
	accEst = join_4_bytes(&UBX_buffer[24]);
	checksum = join_2_bytes(&UBX_buffer[28]);
      break;
	}
	NewData=1;
break;
}
}

/****************************************************************
 * 
 ****************************************************************/
 // Join 4 bytes into a long
long GPS_UBLOX_Class::join_4_bytes(unsigned char Buffer[])
{
  union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

  longUnion.byte[0] = *Buffer;
  longUnion.byte[1] = *(Buffer+1);
  longUnion.byte[2] = *(Buffer+2);
  longUnion.byte[3] = *(Buffer+3);
  return(longUnion.dword);
}

long GPS_UBLOX_Class::join_2_bytes(unsigned char Buffer[])
{
  union long_union {
	int32_t dword;
	uint8_t  byte[2];
} longUnion;

  longUnion.byte[0] = *Buffer;
  longUnion.byte[1] = *(Buffer+1);
  return(longUnion.dword);
}

void GPS_UBLOX_Class::translate_tow(long ms, long week_number, long nanis) {
}

long GPS_UBLOX_Class::one_byte(unsigned char Buffer[])
{
  union long_union {
	int32_t dword;
	uint8_t  byte[1];
} longUnion;

  longUnion.byte[0] = *Buffer;
  return(longUnion.dword);
}

char* GPS_UBLOX_Class::bytes_to_decimal(char Buffer[])
{

int number;
return(itoa(number, Buffer, 10));
}
/****************************************************************
 * 
 ****************************************************************/
// Ublox checksum algorithm
void GPS_UBLOX_Class::ubx_checksum(byte ubx_data)
{
  ck_a+=ubx_data;
  ck_b+=ck_a; 
}

GPS_UBLOX_Class GPS;