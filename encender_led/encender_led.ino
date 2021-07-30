#include <dummy.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  #define LED_BUILTIN 4
  pinMode (LED_BUILTIN, OUTPUT);//Specify that LED pin is output
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0;i<10;i++){
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.write("Led encendido");
    delay(20);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.write("Led Apagado");
  }
}
