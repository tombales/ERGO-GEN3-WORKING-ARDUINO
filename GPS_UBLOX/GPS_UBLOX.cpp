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
	String unitid = "01111";
	long GpsCycle = 0;
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

void GPS_UBLOX_Class::send_Message(int ID) {
if (ID == 1337) {
int gps_set_sucess=0; 
int setNav4[] = {// 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE
  0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91, 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x21, 0xAF };
//     while(i < 3)
//{
 // i++;
    sendUBX(setNav4, sizeof(setNav4)/sizeof(int));
    gps_set_sucess=getUBX_ACK(setNav4);
  //}
  gps_set_sucess=0; 
   int setNav5[] = {// 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0xCE 
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0xCE };
//     while(i < 3)
//{
 // i++;
    sendUBX(setNav5, sizeof(setNav5)/sizeof(int));
    gps_set_sucess=getUBX_ACK(setNav5);
  //}
  gps_set_sucess=0; 
  gps_set_sucess=0; 
  //int i = 0;
   int setNav2[] = {//0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x0D, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x20, 0x25
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x0D, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x20, 0x25 };
//     while(i < 3)
//{
 // i++;
    sendUBX(setNav2, sizeof(setNav2)/sizeof(int));
    gps_set_sucess=getUBX_ACK(setNav2);
  //}
  //i = 0;
  gps_set_sucess=0; 
 // int i = 0;
   int setNav[] = {// 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE };
//     while(i < 3)
//{
 // i++;
    sendUBX(setNav, sizeof(setNav)/sizeof(int));
    gps_set_sucess=getUBX_ACK(setNav);
  //}
  //i = 0;
  gps_set_sucess=0; 
  }
}

 String GPS_UBLOX_Class::URLEncode(const char* msg)
{
    const char *hex = "0123456789abcdef";
    String encodedMsg = "";

    while (*msg!='\0'){
        if( ('a' <= *msg && *msg <= 'z')
                || ('A' <= *msg && *msg <= 'Z')
                || ('0' <= *msg && *msg <= '9') ) {
            encodedMsg += *msg;
        } else {
            encodedMsg += '%';
            encodedMsg += hex[*msg >> 4];
            encodedMsg += hex[*msg & 15];
        }
        msg++;
    }
    return encodedMsg;
}

void GPS_UBLOX_Class::packatize(){
	String thisDatax;
	String time;
    String date;
    long altitude = Altitude/10;
    long latitude = (Lattitude*3.6)/10;
    long longitude = (Longitude*3.6)/10;
    long nanoseconds = towSubMsR + (1000000*(GPS.towMsR%1000));
          date = "";
                Mjd = GpsToMjd(GpsCycle, wnR, towMsR/1000);
            MjdToDate(Mjd, &Year, &Month, &Day);
            if (Month < 10){
              date += "0";
            }
           date += Month;
           date += "/" ;
            if (Day < 10){
              date += "0";
            } 
            date += Day;
            date += "/" ;
            if (Year - 2000 < 10) {
              date += "0";
            }
            date += (Year-2000);
            date += " ";
           time = GPSTOWtoUTC(towMsR/1000);
      String lat;
      lat += " ";
      if (latitude > 0) {
        lat += "+";
      }
      if (latitude < 0) {
        lat += "-";
      }
       for (int i = 10; i <= 100000000; i *= 10) {
            if (abs(latitude) < i) {
              lat += "0";
            }
          }     
 /*     if (abs(latitude) < 100000000) {
        lat += "0";
        if (abs(latitude) < 10000000) {
          lat += "0";
          if (abs(latitude) < 1000000) {
            lat += "0";
            if (abs(latitude) < 100000) {
              lat += "0";
              if (abs(latitude) < 10000) {
                lat += "0";
                if (abs(latitude) < 1000) {
                  lat += "0";
                  if (abs(latitude) < 100) {
                    lat += "0";
                    if (abs(latitude) < 10) {
                      lat += "0";
                  }
                }   
              }
            }
          }
        }
      }
    }*/
    lat += abs(latitude);
    lat += " ";
   // thisData6 += " ";
    String lon;
          if (longitude > 0) {
        lon += "+";
      }
      if (longitude < 0) {
        lon += "-";
      }
       for (int i = 10; i <= 100000000; i *= 10) {
            if (abs(longitude) < i) {
              lon += "0";
            }
          }      
   /*   if (abs(longitude) < 100000000) {
        lon += "0";
        if (abs(longitude) < 10000000) {
          lon += "0";
          if (abs(longitude) < 1000000) {
            lon += "0";
            if (abs(longitude) < 100000) {
              lon += "0";
              if (abs(longitude) < 10000) {
                lon += "0";
                if (abs(longitude) < 1000) {
                  lon += "0";
                  if (abs(longitude) < 100) {
                    lon += "0";
                    if (abs(longitude) < 10) {
                      lon += "0";
                  }
                }   
              }
            }
          }
        }
      }
    }*/
    lon += abs(longitude);
    lon += " ";
      String alt; //-0007736
          if (altitude > 0) {
        alt += "+";
      }
      if (altitude < 0) {
        alt += "-";
      }
       for (int i = 10; i <= 1000000; i *= 10) {
            if (abs(altitude) < i) {
              alt += "0";
            }
          }
      /*    if (abs(altitude) < 1000000) {
            alt += "0";
            if (abs(altitude) < 100000) {
              alt += "0";
              if (abs(altitude) < 10000) {
                alt += "0";
                if (abs(altitude) < 1000) {
                  alt += "0";
                  if (abs(altitude) < 100) {
                    alt += "0";
                    if (abs(altitude) < 10) {
                      alt += "0";
                  }
                }   
              }
            }
          }
        }*/
    alt += abs(altitude);
    alt += " ";
          String ns; //-0007736
          for (int i = 10; i <= 100000000; i *= 10) {
            if (abs(nanoseconds) < i) {
              ns += "0";
            }
          }
    /*  if (abs(nanoseconds) < 100000000) {
          ns += "0";
        if (abs(nanoseconds) < 10000000) {
          ns += "0";
          if (abs(nanoseconds) < 1000000) {
            ns += "0";
            if (abs(nanoseconds) < 100000) {
              ns += "0";
              if (abs(nanoseconds) < 10000) {
                ns += "0";
                if (abs(nanoseconds) < 1000) {
                  ns += "0";
                  if (abs(nanoseconds) < 100) {
                    ns += "0";
                    if (abs(nanoseconds) < 10) {
                      ns += "0";
                  }
                }   
              }
            }
          }
        }
      }
    }*/
    ns += abs(nanoseconds);
 thisDatax = date + time + unitid + lat + lon + alt + ns;
	thisDatax.toCharArray(thisData, 65);
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

String GPS_UBLOX_Class::GPSTOWtoUTC(long secs)
{//converts the GPS Time of the Week presented by the UBLOX GPS
 // to the UTC "packed integer" format

 //Useful constants
 #define SEC_PER_DAY 86400
 #define SEC_PER_HOUR 3600
 #define SEC_PER_MINUTE 60
 #define HOURS_PER_DAY 24
 #define MINUTES_PER_DAY 1440
 #define MINUTES_PER_HOUR 60
 String UTC_pack;
 long sleft;
 int offsetday = secs/SEC_PER_DAY;//today is tuesday
// 1514615313
//Serial.println(offsetday);
 sleft = secs - (offsetday * SEC_PER_DAY); //57300 secs left today
// Serial.println(sleft);
int hs = sleft/SEC_PER_HOUR;//15 hours today
//Serial.println(hs);
int sremainder;
sremainder = sleft - (hs * SEC_PER_HOUR); // 3300 unused seconds... YET!
//Serial.println(sremainder);
int mins = sremainder/SEC_PER_MINUTE; //55 and in this case, there are no seconds remaining
//Serial.println(mins);
long stime = sremainder - (mins*SEC_PER_MINUTE); //This would be used if there were seconds left over
//Serial.println(stime);
if (hs < 10) {UTC_pack += "0";}
UTC_pack += hs; 
UTC_pack += ":" ;
if (mins < 10) {UTC_pack += "0";}
UTC_pack += mins;
UTC_pack += ":";
if (stime < 10) {UTC_pack += "0";}
UTC_pack += stime;
UTC_pack += " ";


return UTC_pack;
}

void GPS_UBLOX_Class::MjdToDate (long Mjd, long *Year, long *Month, long *Day)
{
    long J = 0;
    long CD = 0;
    long Y = 0;
    long M = 0;

    J = Mjd + 2400001 + 68569;
    CD = 4 * J / 146097;
    J = J - (146097 * CD + 3) / 4;
    Y = 4000 * (J + 1) / 1461001;
    J = J - 1461 * Y / 4 + 31;
    M = 80 * J / 2447;
    *Day = J - 2447 * M / 80;
    J = M / 11;
    *Month = M + 2 - (12 * J);
    *Year = 100 * (CD - 49) + Y + J;
}

long GPS_UBLOX_Class::GpsToMjd(long GpsCycle, long GpsWeek, long GpsSeconds)
{
  long GpsDays;
	    GpsDays = ((GpsCycle * 1024) + GpsWeek) * 7 + (GpsSeconds / 86400);
    return DateToMjd(1980, 1, 6) + GpsDays;
}

long GPS_UBLOX_Class::DateToMjd (long Year, long Month, long Day)
{
      return
        367 * Year
        - 7 * (Year + (Month + 9) / 12) / 4
        - 3 * ((Year + (Month - 9) / 7) / 100 + 1) / 4
        + 275 * Month / 9
        + Day
        + 1721028
        - 2400000;
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

 // Send a byte array of UBX protocol to the GPS
void GPS_UBLOX_Class::sendUBX(int *MSG, int len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
   // Serial.print(MSG[i], HEX);
  }
  Serial.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean GPS_UBLOX_Class::getUBX_ACK(int *MSG) {
  int b;
  int ackByteID = 0;
  int ackPacket[10];
  unsigned long startTime = millis();
  //Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (int i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
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
