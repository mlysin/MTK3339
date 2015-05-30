//doesn't work well
#include <Servo.h> 
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
 
Servo myservo;  // create servo object to control a servo 
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);
           
 
int pos = 0;    // variable to store the servo position 
int jump = 60;  // variable of step angle 
int wait = 1500;  // variable of wait time between steps
int min = 0;  // minimum servo angle
int max = 180; // maximum servo angle

#define GPSECHO  true 
boolean usingInterrupt = false;
void useInterrupt(boolean);

void setup() 
{ 
  myservo.attach(9);  // attach the servo to pin 9 to the servo object 
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  useInterrupt(true); 
  delay(2000);
} 

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c;
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
uint32_t timer = millis();
 
void gpsloop() {// do nothing! all reading and printing is done in the interrupt
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
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
    }  
  }
}

void loop(){   
  gpsloop();
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
