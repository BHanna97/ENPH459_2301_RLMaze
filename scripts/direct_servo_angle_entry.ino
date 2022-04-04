#include <Servo.h>

Servo servoA; 
Servo servoB;
Servo servoC;


int signalA;    // variable to read the value from the analog pin
int signalB;
int signalC;

int enbl = 7;
int dir = 6;
int pul = 5;

int homesw = 4;
const int CMD_LEN = 11;

char command[CMD_LEN];
char* token;


void setup() {
  // attach servos to their respective pwm pins
  servoA.attach(9);
  servoB.attach(10);
  servoC.attach(11);
  Serial.begin(9600);

// Stepper motor pins
  pinMode(enbl, OUTPUT); 
  pinMode(dir, OUTPUT); 
  pinMode(pul, OUTPUT); 
  digitalWrite(enbl, LOW);
  digitalWrite(dir, HIGH);
  //Home
  pinMode(homesw, INPUT_PULLUP); 

 homely();
}

void loop() {
   if(Serial.available() > 0){
      Serial.readBytesUntil('\n', command, CMD_LEN);
      
if(String(command) == "rec"){recover();}
      
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
  delay(1);      // waits for the servo to get there
}

void recover(){
  digitalWrite(enbl, LOW); //ENABLE STEPPER
  delay(10000); //wait for ball to get to elevator
  //step
int  steps = 0 ;
  while(steps <= 630){
      digitalWrite(pul, HIGH);
      delay(2);
      digitalWrite(pul, LOW);
      delay(2);
      steps++;
    }
    steps = 0;
    digitalWrite(dir, HIGH);
   while(steps <= 40){
      digitalWrite(pul, HIGH);
      delay(10);
      digitalWrite(pul, LOW);
      delay(10);
      steps++;
    }
  delay(100);
  homely();
}

void homely(){

   digitalWrite(enbl, LOW); //enable motor
   digitalWrite(dir, HIGH);
  while(digitalRead(homesw) == HIGH){ //GO DOWN
    //step
    digitalWrite(pul, HIGH);
    delayMicroseconds(1000);
    digitalWrite(pul, LOW);
    delayMicroseconds(1000);
    }
digitalWrite(dir, LOW);
  while(digitalRead(homesw) == LOW){ //GO UP SLOWLY
    //step
    digitalWrite(pul, HIGH);
    delay(10);
    digitalWrite(pul, LOW);
    delay(10);
    }
    int count = 0;
while (count < 5){
  //step
    digitalWrite(pul, HIGH);
    delay(10);
    digitalWrite(pul, LOW);
    delay(10);
    count++;
  }
   digitalWrite(enbl, HIGH); //disable motor
   digitalWrite(dir, LOW);
  }
