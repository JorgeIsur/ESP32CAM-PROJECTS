/*
    AUTOR: Jorge Isur Balderas Ramírez
    FECHA: 25/08/2021
    DISPOSITIVO: ESP32CAM + DHT11
    DESCRIPCIÓN: Programa que muestra la temperatura medida por el DHT11
    GPIO DESCRIPTION:



*/
#include "DHT.h"
// DEFINIMOS LOS PINES QUE SE USARÁN
#define DHTPIN 2
#define DHTTYPE DHT11   // DHT 11
#define LED_OK 13
#define LED_WARNING 14
#define LED_FATAL 15
//inicializar el DHT
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  Serial.println(F("Funcionando!"));
  pinMode (LED_OK, OUTPUT);//Specify that LED pin is output
  pinMode (LED_WARNING, OUTPUT);//Specify that LED pin is output
  pinMode (LED_FATAL, OUTPUT);//Specify that LED pin is output
  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(5000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("ERROR AL REGISTRAR LOS DATOS DEL SENSOR!"));
    return;
  }
  if (t<30)
  {
      digitalWrite(LED_OK,HIGH);
      digitalWrite(LED_WARNING,LOW);
      digitalWrite(LED_FATAL,LOW);
  }
  if (t>=30 && t<40)
  {
      digitalWrite(LED_WARNING,HIGH);
      digitalWrite(LED_FATAL,LOW);
      digitalWrite(LED_OK,LOW);
  }
  else{
      digitalWrite(LED_OK,LOW);
      digitalWrite(LED_WARNING,LOW);
      digitalWrite(LED_FATAL,HIGH);
  }
  Serial.print(F("Humedad: "));
  Serial.print(h);
  Serial.print(F("%  Temperatura: "));
  Serial.print(t);
  Serial.print(F("°C \n"));
}
