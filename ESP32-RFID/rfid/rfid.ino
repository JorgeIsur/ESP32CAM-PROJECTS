#include <MFRC522.h>
#include <SPI.h>
//DEFINICION DE PINES
#define SDA_PIN 4
#define RST_PIN 2
//INICIAR INSTANCIA DE LIBRERIA
MFRC522 mfrc522(SDA_PIN,RST_PIN);
void setup(){
    Serial.begin(115200);
    Serial.print("LECTOR RFID");
    SPI.begin();
    mfrc522.PCD_Init();
    Serial.print("Acerque la tarjeta al lector...\n");
}
void loop(){
    // Revisamos si hay nuevas tarjetas  presentes
	if ( mfrc522.PICC_IsNewCardPresent()) {  
  		//Seleccionamos una tarjeta
        if ( mfrc522.PICC_ReadCardSerial()){
            // Enviamos serialemente su UID
            Serial.print("Card UID:");
            for (byte i = 0; i < mfrc522.uid.size; i++) {
                Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
                Serial.print(mfrc522.uid.uidByte[i], HEX);   
            } 
            Serial.println();
            // Terminamos la lectura de la tarjeta  actual
            mfrc522.PICC_HaltA();         
        }      
	}	
}
