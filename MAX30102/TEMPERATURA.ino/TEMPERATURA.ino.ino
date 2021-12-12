/*
  MAX3010 Breakout: Read the onboard temperature sensor
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 20th, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This demo outputs the onboard temperature sensor. The temp sensor is accurate to +/-1 C but
  has an astonishing precision of 0.0625 C.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V es permitido)
  -GND = GND
  -SDA = GPIO15(or SDA)
  -SCL = GPIO14 (or SCL)
  -INT = No se
 
  EL MAX30102 SOPORTA CONEXIONES I2C LOGICAS DE 3.3V Y 5V , DE CUALQUIER MANERA SE RECOMIENDA 
  ALIMENTAR CON 5V.
*/

#include <Wire.h>

#include "MAX30105.h"  //OBTENER AQUI: http://librarymanager/All#SparkFun_MAX30105
MAX30105 particleSensor;

void setup()
{
  Serial.begin(9600);
  Serial.println("LECTOR DE TEMPERATURA:\n");
  Serial.println("Initializing...\n");

  // Initialize sensor
  Wire.begin(15,14);
  if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX3010X NO ENCONTRADO. POR FAVOR REVISE EL CABLEADO O LA ALIMENTACIÓN. \n");
    while (1);
  }

  //The LEDs are very low power and won't affect the temp reading much but
  //you may want to turn off the LEDs to avoid any local heating
  particleSensor.setup(0); //Configure sensor. Turn off LEDs
  //particleSensor.setup(); //Configure sensor. Use 25mA for LED drive

  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
}

void loop()
{
  float temperature = particleSensor.readTemperature();
  Serial.print(temperature, 4);
  Serial.println();
  delay(1000);
}
