
#include <GPS_UBLOX.h> // UBLOX GPS Library

void setup()
{
  Serial.begin(38400);
  Serial.println("GPS UBLOX library test");
  GPS.Init();   // GPS Initialization
  delay(1000);
}
void loop()
{
  GPS.Read();
  if (GPS.NewData)  // New GPS data?
    {
      if (GPS.PosData == 1) {
        GPS.PosData = 0; //Lat/Long data
      }
      else {
    Serial.print("GPS:");
    ///*
    Serial.print(" Time:");
    Serial.print(GPS.Time);
    Serial.print(" Fix:");
    Serial.print((int)GPS.Fix);
    Serial.print(" Lat:");
    Serial.print(GPS.Lattitude);
    Serial.print(" Lon:");
    Serial.print(GPS.Longitude);
    Serial.print(" Alt:");
    Serial.print(GPS.Altitude/1000.0);
    Serial.print(" Speed:");
    Serial.print(GPS.Ground_Speed/100.0);
    Serial.print(" Course:");
    Serial.print(GPS.Ground_Course/100000.0);
    Serial.println();
   // GPS.Read();
    Serial.print(" ch:");
    Serial.print(GPS.ch);
    Serial.print(" flags:");
    Serial.print(GPS.flags);
    Serial.print(" count:");
    Serial.print(GPS.count);
    Serial.print(" wnR:");
    Serial.print(GPS.wnR);
    Serial.print(" wnF:");
    Serial.print(GPS.wnF);
    Serial.print(" towMsR:");
    Serial.print(GPS.towMsR);
    Serial.print(" towMsF:");
    Serial.print(GPS.towMsF); 
    Serial.print(" towSubMsF:");
    Serial.print(GPS.towSubMsF);    
    Serial.print(" accEst:");
    Serial.print(GPS.accEst);       
    Serial.print(" checksum:");
    Serial.print(GPS.checksum);   
    Serial.println();
      }
    GPS.NewData = 0; // We have read data, reseting
    }
  delay(20);
}
