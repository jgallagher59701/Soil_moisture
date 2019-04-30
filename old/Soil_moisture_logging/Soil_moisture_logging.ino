/*  Soil Mositure Basic Example
    This sketch was written by SparkFun Electronics
    Joel Bartlett 
    August 31, 2015

    Basic skecth to print out soil moisture values to the Serial Monitor 

    Released under the MIT License(http://opensource.org/licenses/MIT)
*/

#include <SPI.h>
#include <SD.h>

const int soilPin = A0;   // Declare a variable for the soil moisture sensor 
const int soilPower = 7;  // Variable for Soil moisture Power

const int chipSelect = 4; // CS pin for the SD card

const String dataFileName = "/data/data_log.txt";

//Rather than powering the sensor through the 3.3V or 5V pins, 
//we'll use a digital pin to power the sensor. This will 
//prevent corrosion of the sensor as it sits in the soil. 

File dataFile; // Data file

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

}

void loop() 
{
  Serial.print("Soil Moisture = ");    
  // get soil moisture value from the function below and print it
  int val = readSoil();
  Serial.println(val);

  log_measurement(val);

  // This 1 second timefrme is used so you can test the sensor and see it change in real-time.
  // For in-plant applications, you will want to take readings much less frequently.
  delay(1000);//take a reading every second
}

//This is a function used to get the soil moisture content
int readSoil()
{
  int val;
  
    digitalWrite(soilPower, HIGH);//turn D7 "On"
    delay(10);//wait 10 milliseconds 
    val = analogRead(soilPin);//Read the SIG value form sensor 
    digitalWrite(soilPower, LOW);//turn D7 "Off"
    return val;//send current moisture value
}

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
