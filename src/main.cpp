#include <Arduino.h>


void setup() {
  delay(1000); 
  
  Serial.begin(115200);

  delay(1000); 
}

void loop() {
  Serial.println("esp32-S3 active");
  delay(3000);
}
