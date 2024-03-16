#include <Wire.h>
//#include "sensors.h"
#include "BMP.h"
#include "BNO.h"


BMP581 pressureSensor;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("BMP581 and BNO055 data extraction"); Serial.println("");
  Wire_r.begin();
/* Initialise the sensor */
    // BMP581 sensor initialization
  while (pressureSensor.beginI2C(BMP5_I2C_ADDR_SEC) != BMP5_OK)
  {
    Serial.println("Error: BMP581 not connected, check wiring and I2C address!");
    delay(1000);
  }
  // BNO055 sensor initialization
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}

void loop(void) 
{

 unsigned long startTime = millis(); // Record the start time
  // BNO055 sensor data
  print_bno_data();
  // BMP581 sensor data
  read_bmp(pressureSensor);

   //Calculate and print the time elapsed since the last reading
  unsigned long elapsedTime = millis() - startTime;
  Serial.print("Time Elapsed (ms): ");
  Serial.println(elapsedTime);
  Serial.println();
  delay(750);  //Delay between readings remove // to debug
}

/*
North is along the positive y-axis,
East is along the positive x-axis, and
Down is along the positive z-axis
*/
