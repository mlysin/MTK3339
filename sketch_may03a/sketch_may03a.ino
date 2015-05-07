#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial s(1, 0);// Tx = 1; Rx = 0;

void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 2);

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200); breaks 
  s.begin(9600);//Just run it at 9600 baud
}

void loop() {
  // put your main code here, to run repeatedly:
  bool newdata = false;
  unsigned long start = millis();
  
  while (millis() - start < 250) {
    if (feedgps())
      newdata = true;
  }
  
  if (newdata) {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }
}

void printFloat(double number, int digits)
{
  //Handle negative numbers
  if (number < 0.0)
  {
    Serial.print('-');
    number = -number;
  }
  
  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
}

void gpsdump(TinyGPS &gps) {
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  unsigned short sentences, failed;
  
  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  feedgps(); 
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
  Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
  
}

bool feedgps() {
  while (s.available()) {
    if (gps.encode(s.read()))
      return true;
  }
  return false;
}
