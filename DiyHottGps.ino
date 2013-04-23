/**
 * DIY-HOTT-GPS is a standalone Arduino based Application that acts 
 * as a HoTTv4 capable device to transmit GPS/Vario information. 
 * Code contains party by Oliver Bayer, Carsten Giesen, Jochen Boenig and Stefan Muerzl 04/2013
 */

#include "SoftwareSerial.h"
#include <avr/io.h> 
#include <TinyGPS.h> 
TinyGPS gps; 

float HOME_LAT = 0, HOME_LON = 0;
float start_height = 0;

bool feedgps();  
bool is_set_home = 0;
uint32_t last = 0;
int p_alt[4]={0,0,0,0};

struct {
  
  uint8_t  GPS_fix;
  uint8_t  GPS_numSat;
  uint32_t GPS_latitude;
  uint32_t GPS_longitude;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;
  uint16_t GPS_Vario1;
  uint8_t  GPS_Vario3;
  uint8_t  GPS_flightDirection;
  uint16_t GPS_distanceToHome;
  uint8_t  GPS_directionToHome;
  
} MultiHoTTModule;

#define LED 13

void setup() {
  
  #ifdef DEBUG
  mySerial.begin(19200);
  #endif
  pinMode(LED, OUTPUT);
  delay(200);

  Serial.begin(57600);
  delay(200);
  //Serial.println("$PMTK300,250,0,0,0,0*2A");  //   println haengt ein cr, nl an  
  delay(100);
  
  is_set_home = 0;

  hottV4Setup();
  
}

bool feedgps()
{
  while (Serial.available())
  {
    if (gps.encode(Serial.read()))
      return true;
  }
  return false;
}

void toggle_LED(){
 digitalWrite(LED, !digitalRead(LED)); 
}

void loop() {

  //Variablen fuers GPS-Funktionen
  unsigned long speed_k; // Hundertstel Koten 
  long lat, lon;
  float flat, flon, alt;
  unsigned long age, dat, tim, ui_course;
  //uint16_t alt;
  unsigned int numsat; 
  bool newdata = false;
  uint32_t now = millis();  
  
 if (feedgps()) newdata = true;
 
 if ( newdata == true)
 {

   gps.get_position(&lat, &lon, &age);
   gps.f_get_position(&flat, &flon);
   gps.get_datetime(&dat, &tim, &age);
   //alt = (int) gps.f_altitude();
   alt =  gps.altitude()/100;
   numsat = gps.satellites(); 
   ui_course = gps.course()/100;
   speed_k = gps.speed(); 
   
   if ((now - last) > 1000) //jede Sekunde 1 Messung fuer's Vario
   {  
     last = now;
     p_alt[3] = p_alt[2];
     p_alt[2] = p_alt[1];
     p_alt[1] = p_alt[0];
     p_alt[0] = alt+500;     //hier wird die aktuelle Messung gespeichert
   }
   
   //Homeposition setzen
   if (is_set_home == 0 && numsat >= 5)  // Wenn noch nicht gesetzt und mehr als 5 Sat. in Sicht
   {
       
       HOME_LAT = flat;
	   HOME_LON = flon;
	   //in Zukunft Starthoehe mittels BMP085 setzen
       start_height = alt;
	   
	   if (start_height < 10000)		//Problem mit hoehe
	   {
		is_set_home = 1; //true
	   }	
        
   }  
   
   
   MultiHoTTModule.GPS_fix       = 1;       //Haben wir einen fix?
   MultiHoTTModule.GPS_numSat    = numsat;  //Satelliten in Sicht
   MultiHoTTModule.GPS_latitude  = lat;     //Geograph. Breitengrad
   MultiHoTTModule.GPS_longitude = lon;     //Geograph. Laengengrad
   MultiHoTTModule.GPS_speed     = (speed_k * 1852) / 100000; // vom GPS in Knoten/100 -> km/h
   MultiHoTTModule.GPS_altitude = alt+500-start_height;  // vom GPS in cm, +500m Offset für Hott   
   MultiHoTTModule.GPS_distanceToHome = gps.distance_between(flat, flon, HOME_LAT, HOME_LON);
   MultiHoTTModule.GPS_directionToHome = gps.course_to(HOME_LAT, HOME_LON, lat, lon) / 2;
   MultiHoTTModule.GPS_flightDirection = ui_course/2;   
 }

  // Daten senden
  hottV4SendTelemetry();
}
