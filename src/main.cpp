#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>

#define PIN_SW_RX  PB7
#define PIN_SW_TX  PB6

HardwareSerial   Serial1(USART1);
HerkulexServoBus herkulex_bus(Serial1);


void setup() {
  Serial.begin(115200);
  Serial1.setRx(PIN_SW_RX); // Associe la broche RX à l'UART1
  Serial1.setTx(PIN_SW_TX); // Associe la broche TX à l'UART1
  Serial1.begin(115200);    // Initialise la communication série à 115200 bauds
  delay(500);
  Serial.println("Press 's' to scan the bus for servos.");
}

void loop() {
    
  herkulex_bus.update();

  if (Serial.available() > 0) {
    char c = Serial.read();
    Serial.print(c);
    
    if (c == 's') {
      Serial.println("Scanning...");
      Serial.println("Addresses are displayed in hexadecimal");
      uint8_t servos_found = 0;

      for (uint8_t id = 0; id <= 0xFD; id++) {
        HerkulexPacket resp;
        bool success = herkulex_bus.sendPacketAndReadResponse(resp, id, HerkulexCommand::Stat);

        if (success) {
          servos_found++;

          if (id <= 0x0F) {
            Serial.print("0");
          }

          Serial.print(id, HEX);
        } else {
          Serial.print("--");
        }

        if ( ( (id+1) % 0x0F) == 0) {
          Serial.println();
        } else {
          Serial.print(" ");
        }
      }

      Serial.println();
      Serial.println("Done!");
      Serial.print("Found ");
      Serial.print(servos_found);
      Serial.println(" servos.");
    }
  }
}