// TARRo GPS + IMU for Mega 2560
// See TARRo_gps.ino for explanation on NMEA parsing and hardware (not softserial) setup

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Servo.h> 

Servo myservo;  // create servo object to control a servo
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);

int pos = 0;    // variable to store the servo position 
int jump = 60;  // variable of step angle 
int wait = 1500;  // variable of wait time between steps
int min = 0;  // minimum servo angle
int max = 180; // maximum servo angle

// 'false' to turn off echoing the GPS data to the Serial console
//'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true 
boolean usingInterrupt = false;
void useInterrupt(boolean);

void setup() {
  myservo.attach(9);  // attach the servo to pin 9 to the servo object 
  
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
  GPS.begin(9600); // 9600 NMEA is the default baud rate for MTK
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //MC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // "minimum recommended" data for high update rates
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA); // all the available data - for 9600 baud you'll want 1 Hz rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  //GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet
  useInterrupt(true); 
  delay(1000);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c; 
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();


void servoloop(){
  
  for(pos = min; pos <= max; pos += (jump)) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(wait);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = max; pos>=min; pos-=(jump))     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(wait);                       // waits 15ms for the servo to reach the position 
  }
} 


void loop() {// do nothing! all reading and printing is done in the interrupt
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  
  servoloop();
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (timer > millis())  timer = millis(); // if millis() or timer wraps around, we'll just reset it

  if (millis() - timer > 1000) { // approximately every 2 seconds or so, print out the current stats
    timer = millis(); // reset the timer
    Serial.print("\n\nTime: ");
    Serial.print(GPS.hour, DEC); // would be nice to convert to PST
    Serial.print(':');
    Serial.print(GPS.minute, DEC); 
    Serial.print(':');
    Serial.print(GPS.seconds, DEC); 
    Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); 
    Serial.print('/');
    Serial.print(GPS.month, DEC); 
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix:  "); 
    Serial.print((int)GPS.fix);
    Serial.print(" Quality: "); 
    Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location:      ");
      Serial.print(GPS.latitude, 4); 
      Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); 
      Serial.println(GPS.lon);

      Serial.print("Speed (MPH):   "); 
      Serial.println(GPS.speed);
      //Serial.print("Angle:         "); 
      //Serial.println(GPS.angle);
      //Serial.print("Altitude (Ft): "); 
      //Serial.println(GPS.altitude);
      //Serial.print("Satellites:    "); 
      //Serial.println((int)GPS.satellites);
    }  
  }
}
