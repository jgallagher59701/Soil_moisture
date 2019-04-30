/*  Soil Mositure Basic Example
    This sketch was written by SparkFun Electronics
    Joel Bartlett 
    August 31, 2015

    Basic skecth to print out soil moisture values to the Serial Monitor 

    Released under the MIT License(http://opensource.org/licenses/MIT)
*/

#include <SPI.h>
#include <SD.h>

const int soilPin = A0;   // Sensor made from nails 
const int soilPin2 = A3;  // The Sparkfun sensor

// Rather than powering the sensor through the 3.3V or 5V pins, 
// we'll use a digital pin to power the sensor. This will 
// prevent corrosion of the sensor as it sits in the soil. 
const int soilPower = 7;  // Variable for Soil moisture Power
const int soilPower2 = 8;

#if 0
const int chipSelect = 4; // CS pin for the SD card
const String dataFileName = "/data/data_log.txt";

File dataFile; // Data file
#endif

void setup() 
{
  Serial.begin(9600);   // open serial over USB
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // 'Power' the resistive probe using a digital I/O pin so that the probe is only 
  // powered during measurements to limit corrosion
  pinMode(soilPower, OUTPUT);   // Set D7 as an OUTPUT
  digitalWrite(soilPower, LOW); // Set to LOW so no power is flowing through the sensor

  // Repeat for the SparkFun sensor
  pinMode(soilPower2, OUTPUT);
  digitalWrite(soilPower2, LOW);

#if 0
  // Configure the SD card I/O
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more
    while (1);
  }
  
  Serial.println("card initialized.");
  
  // Open the data file in append mode
  dataFile = SD.open(dataFileName, FILE_WRITE);
  if (dataFile) {
    Serial.println("opened " + dataFileName);
  }
  else {
    Serial.println("error opening " + dataFileName);
    while (1);
  }
#endif
}

void loop() 
{
  Serial.print("Soil Moisture (Two Nails) = ");    
  // get soil moisture value from the function below and print it
  int val = readSoil(soilPower, soilPin);
  Serial.println(val);

  // delay between readings
  delay(1000);
  
  Serial.print("Soil Moisture (SparkFun) = ");    
  // get soil moisture value from the function below and print it
  val = readSoil(soilPower2, soilPin2);
  Serial.println(val);

#if 0
  log_measurement(val);
#endif

  // This 1 second timefrme is used so you can test the sensor and see it change in real-time.
  // For in-plant applications, you will want to take readings much less frequently.
  delay(1000);
}

// This is a function used to get the soil moisture content
int readSoil(int sensorProbe, int sensorPin)
{
  int val;
  
    digitalWrite(sensorProbe, HIGH);//turn D7 "On"
    delay(10);//wait 10 milliseconds 
    val = analogRead(sensorPin);//Read the SIG value form sensor 
    digitalWrite(sensorProbe, LOW);//turn D7 "Off"
    return val;//send current moisture value
}

#if 0
void log_measurement(int val)
{  
  String dataString = "date, " + String(val);
  
  // if the file is available, write to it:
  if (dataFile) {
    // FIXME Add a RTC to this. 
    dataFile.println(dataString);
    dataFile.flush();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening " + dataFileName);
  }
}
#endif
