#include <Servo.h>

Servo servoA; 
Servo servoB;
Servo servoC;


int knobA = A0;
int knobB = A1;
int knobC = A2;

int signalA;    // variable to read the value from the analog pin
int signalB;
int signalC;

const int CMD_LEN = 11;

char command[CMD_LEN];
char* token;


void setup() {
  // attach servos to their respective pwm pins
  servoA.attach(9);
  servoB.attach(10);
  servoC.attach(11);
  Serial.begin(9600);
}

void loop() {
    if(Serial.available() > 0){
      Serial.readBytesUntil('\n', command, CMD_LEN);
      token = strtok(command, ",");
      signalA = String(token).toInt();

      token = strtok(NULL, ",");
      signalB = String(token).toInt();

      token = strtok(NULL, ",");
      signalC = String(token).toInt();

      servoA.write(signalA);
      servoB.write(signalB);
      servoC.write(signalC);
    };   
  delay(15);                           // waits for the servo to get there
}
