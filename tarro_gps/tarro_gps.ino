// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
// Heavy physical hookup comments by Phantom YoYo A.D. 2014


#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// If you're using a bare GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Then wire for software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// Or wire for hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shieldon an Uno etc:
// Make sure the switch is set to SoftSerial
// Change line from:
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
//
// If using a Mega with GPS shield and software serial:
// Make sure the switch is set to SoftSerial
// Connect the GPS TX (transmit) solder pad to Digital 11
// Connect the GPS RX (receive) solder pad to Digital 10
// Change line from:
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(11, 10);
//
// On a Mega, with any Ada GPS use hardware serial by adding 2 jumper wires:
// Connect jumper from TX pad to Arduino RX1, RX2 or RX3
// Connect jumper from RX pad to Arduino TX1, TX2 or TX3

//Enable software serial or hardware serial below:

// To use software serial, keep the two lines below enabled
// (you can change the pin numbers to match your wiring):
//SoftwareSerial mySerial(11, 10);
//Adafruit_GPS GPS(&mySerial);
//
// To use hardware serial (e.g. Arduino Mega), comment
// out the above two lines and enable these two lines instead
// Make sure you change serial number for your wiring (Serial1,Serial2,Serial3 etc.)
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup() 
{   
  // connect at 115200 so we can read the GPS fast enuf and
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
 
  // You can adjust which sentences to have the module emit, below
 
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA); 
 
  // Set the update rate
  // 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
 
  delay(2000);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
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
void loop()                     // run over and over again
{
  // do nothing! all reading and printing is done in the interrupt
   
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
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

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 1000) { 
    timer = millis(); // reset the timer


    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); 
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
      Serial.print("Angle:         "); 
      Serial.println(GPS.angle);
      Serial.print("Altitude (Ft): "); 
      Serial.println(GPS.altitude);
      Serial.print("Satellites:    "); 
      Serial.println((int)GPS.satellites);
    }  
  }
}
