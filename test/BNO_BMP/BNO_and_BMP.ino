#include <Wire.h>
//#include "sensors.h"
#include "BMP.hpp"
#include "BNO.hpp"

// this file setup the BNO and BMP then print the data that it gets from them

WIRE_TEENSY test(IMXRT_LPI2C1_ADDRESS, WIRE_TEENSY::i2c1_hardware);
//configure the BMP581
/*
  Main functions:

  To get the data, you need to first configure a BMP581 variable, then create a structure, and finally call the function get_data.

  BMP581 pressureSensor;
  Wire_r.begin();  // Initialize the Wire instance
  int8_t bmpErr = pressureSensor.beginI2C(BMP5_I2C_ADDR_SEC, &Wire_r); // This should return 0. If a different result is obtained, it indicates failure to configure or detect the sensor.
  bmp5_sensor_data bmpData = {0, 0}; // This is a structure with 2 variables: pressure and temperature
  bmpErr = pressureSensor.getSensorData(&bmpData); // Now the data is present in your bmp5_sensor_data structure, call it with bmpData.pressure or bmpData.temperature

  read_bmp(pressureSensor); // Print data onto the serial monitor
*/
BMP581 pressureSensor;
//configure the BNO055
/*
  // Define the address and Wire interface for the BNO055 sensor
  Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS, &theWire); // You can change the theWire but be sure to configure it the same way as the theWire

  // Begin communication with the Wire interface
  theWire.begin(); 

  // Initialize the BNO055 sensor, it should return 0 if initialized successfully
  if (!bno.begin()) {
      // Handle initialization failure here
  }

  // Structure to store BNO055 data
  BNO_data data;

  // Function to read and print all BNO055 data to the serial monitor
  void print_bno_data(I2C_INTERFACE *theWire);

  // Function to update all BNO055 data
  void read_data_all(BNO_data* data,I2C_INTERFACE *theWire);

  // Function to update BNO055 calibration data
  void read_calibration(BNO_data* data,I2C_INTERFACE *theWire);

  // Function to update BNO055 linear acceleration data
  void read_linear(BNO_data* data,I2C_INTERFACE *theWire)

  // Function to update BNO055 gyro data
  void read_gyro(BNO_data* data,I2C_INTERFACE *theWire);

  // Function to update BNO055 magnetometer data
  void read_mag(BNO_data* data,I2C_INTERFACE *theWire);

  // Function to update BNO055 acceleration data
  void read_accel(BNO_data* data,I2C_INTERFACE *theWire);


*/
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS,&test);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("BMP581 and BNO055 data extraction"); Serial.println("");
  test.begin();
/* Initialise the sensor */
  // BMP581 sensor initialization 
  while (pressureSensor.beginI2C(BMP5_I2C_ADDR_SEC,&test) != BMP5_OK)
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
  //print  BNO055 sensor data
  print_bno_data(&test);
  //print BMP581 sensor data
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



