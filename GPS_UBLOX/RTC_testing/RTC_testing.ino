
#include <GPS_UBLOX.h> // UBLOX GPS Library
        long Year, Month, Day;
    long Mjd;
    long GpsCycle = 0;
    long GpsWeek, GpsSeconds;
void setup()
{
  Serial.begin(38400);
  Serial.println("GPS UBLOX library test");
  delay(1000);
  GPS.Init();   // GPS Initialization
  delay(1000);
  Serial.println("Starting-UP");
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
    Serial.print(" Lat:");
    Serial.print(GPS.Lattitude);
    Serial.print(" Lon:");
    Serial.print(GPS.Longitude);
    Serial.print(" Alt:");
    Serial.print(GPS.Altitude/1000.0);
    Serial.print(" wnR:");
    Serial.print(GPS.wnR);
    Serial.print(" towMsR:");
    Serial.print(GPS.towMsR);
    Serial.print(" towSubMsR:");
    Serial.print(GPS.towSubMsR);    
    Serial.println();
    
                Mjd = GpsToMjd(GpsCycle, GPS.wnR, GPS.towMsR/1000);
            MjdToDate(Mjd, &Year, &Month, &Day);
            Serial.println((String) Mjd +"  "+ Month +"/" + Day+"/" + Year);
            Serial.println(GPSTOWtoUTC(GPS.towMsR/1000));
      }
    GPS.NewData = 0; // We have read data, reseting
    }
 // delay(20);
}

String GPSTOWtoUTC(long secs)
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
Serial.println(offsetday);
 sleft = secs - (offsetday * SEC_PER_DAY); //57300 secs left today
 Serial.println(sleft);
int hs = sleft/SEC_PER_HOUR;//15 hours today
Serial.println(hs);
int sremainder;
sremainder = sleft - (hs * SEC_PER_HOUR); // 3300 unused seconds... YET!
Serial.println(sremainder);
int mins = sremainder/SEC_PER_MINUTE; //55 and in this case, there are no seconds remaining
Serial.println(mins);
long stime = sremainder - (mins*SEC_PER_MINUTE); //This would be used if there were seconds left over
Serial.println(stime);
UTC_pack += hs; 
UTC_pack += ":" ;
UTC_pack += mins;
UTC_pack += ":";
UTC_pack += stime;


return UTC_pack; //return a 4byte integer in MediaTek "packed integer" format.
}
void MjdToDate (long Mjd, long *Year, long *Month, long *Day)
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

/*
 * Convert GPS Week and Seconds to Modified Julian Day.
 * - Ignores UTC leap seconds.
 */

long GpsToMjd (long GpsCycle, long GpsWeek, long GpsSeconds)
{
    long GpsDays;

    GpsDays = ((GpsCycle * 1024) + GpsWeek) * 7 + (GpsSeconds / 86400);
    return DateToMjd(1980, 1, 6) + GpsDays;
}
long DateToMjd (long Year, long Month, long Day)
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

