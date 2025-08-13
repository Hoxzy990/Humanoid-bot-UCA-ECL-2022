#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servo;

// Define servo pin numbers
const uint8_t servoPins[] = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15};


uint16_t servolivePositions[] = {385, 400, 500, 200, 140, 600, 500, 275, 375, 375};
uint16_t servolive[] = {385, 400, 500, 200, 140, 600, 500, 275, 375, 375};
void setup() {
  servo.begin();
  servo.setPWMFreq(60);
  Serial.begin(9600);
  Wire.begin();

  for ( int i = 0 ; i < 10 ; i ++ ) {
    servo.setPWM ( servoPins[i], 0, servolivePositions[i]);
  }

 
}

void loop() {

  if (Serial.available()) {
    char data = Serial.read(); // Read incoming data
    switch (data) {
      case 'S':
        servolivePositions[0] += 5;
        servo.setPWM(servoPins[0], 0, servolivePositions[0]);
        break;
      case 's':
        servolivePositions[0] -= 5;
        servo.setPWM(servoPins[0], 0, servolivePositions[0]);
        break;

      case 'D':
        servolivePositions[1] += 5;
        servo.setPWM(servoPins[1], 0, servolivePositions[1]);
        break;
      case 'd':
        servolivePositions[1] -= 5;
        servo.setPWM(servoPins[1], 0, servolivePositions[1]);
        break;

      case 'F':
        servolivePositions[2] += 5;
        servo.setPWM(servoPins[2], 0, servolivePositions[2]);
        break;
      case 'f':
        servolivePositions[2] -= 5;
        servo.setPWM(servoPins[2], 0, servolivePositions[2]);
        break;

      case 'G':
        servolivePositions[3] += 5;
        servo.setPWM(servoPins[3], 0, servolivePositions[3]);
        break;
      case 'g':
        servolivePositions[3] -= 5;
        servo.setPWM(servoPins[3], 0, servolivePositions[3]);
        break;

      case 'H':
        servolivePositions[4] += 5;
        servo.setPWM(servoPins[4], 0, servolivePositions[4]);
        break;
      case 'h':
        servolivePositions[4] -= 5;
        servo.setPWM(servoPins[4], 0, servolivePositions[4]);
        break;

      case 'J':
        servolivePositions[5] += 5;
        servo.setPWM(servoPins[5], 0, servolivePositions[5]);
        break;
      case 'j':
        servolivePositions[5] -= 5;
        servo.setPWM(servoPins[5], 0, servolivePositions[5]);
        break;

      case 'K':
        servolivePositions[6] += 5;
        servo.setPWM(servoPins[6], 0, servolivePositions[6]);
        break;
      case 'k':
        servolivePositions[6] -= 5;
        servo.setPWM(servoPins[6], 0, servolivePositions[6]);
        break;

      case 'L':
        servolivePositions[7] += 5;
        servo.setPWM(servoPins[7], 0, servolivePositions[7]);
        break;
      case 'l':
        servolivePositions[7] -= 5;
        servo.setPWM(servoPins[7], 0, servolivePositions[7]);
        break;

      case 'N':
        servolivePositions[8] += 5;
        servo.setPWM(servoPins[8], 0, servolivePositions[8]);
        break;
      case 'n':
        servolivePositions[8] -= 5;
        servo.setPWM(servoPins[8], 0, servolivePositions[8]);
        break;

      case 'M':
        servolivePositions[9] += 5;
        servo.setPWM(servoPins[9], 0, servolivePositions[9]);
        break;
      case 'm':
        servolivePositions[9] -= 5;
        servo.setPWM(servoPins[9], 0, servolivePositions[9]);
        break;

        
        //.......
      case 'z':
        Serial.print("int desiredPositions[] = {");
        Serial.print(servolivePositions[0]);
        Serial.print(", ");
        Serial.print(servolivePositions[1]);
        Serial.print(", ");
        Serial.print(servolivePositions[2]);
        Serial.print(", ");
        Serial.print(servolivePositions[3]);
        Serial.print(", ");
        Serial.print(servolivePositions[4]);
        Serial.print(", ");
        Serial.print(servolivePositions[5]);
        Serial.print(", ");
        Serial.print(servolivePositions[6]);
        Serial.print(", ");
        Serial.print(servolivePositions[7]);
        Serial.print(", ");
        Serial.print(servolivePositions[8]);
        Serial.print(", ");
        Serial.print(servolivePositions[9]);
        Serial.println("};");
        Serial.println("setServoPositions(desiredPositions);");
        break;
    }
  }
}


