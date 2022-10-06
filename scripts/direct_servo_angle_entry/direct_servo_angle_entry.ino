#include <Servo.h>

Servo servoA;
Servo servoB;
Servo servoC;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

boolean newData = false;

char * reco;  //T to initiate recovery. char to avoid confusion with signals
char F = 'F'; //pointer to reset reco to when finished
int signalA;    // variable to read the value from the analog pin
int signalB;
int signalC;

int minangle = 0;
int maxangle = 65;

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
  servoA.write(40); //HOMESERVOS
  servoB.write(40);
  servoC.write(40);

  homely();
}

void loop() {

  //parsing code modified from https://forum.arduino.cc/t/serial-input-basics-updated/382007/3
  Serial.print("looping");
  Serial.print(* reco);
  Serial.print(* reco == 'T');
  Serial.println();
  recieve();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    sendCommands();
    newData = false;
    delay(1); //Wait for the servos to arrive
  }
}

void sendCommands() {

  Serial.print("in sendCommands:");
  Serial.print(* reco == 'T');
  Serial.println();
  if (*reco == 'T') {
    Serial.print("in if, calling recovery");
    recover();
  }
  else {
    if (signalA > minangle && signalA < maxangle) {
      servoA.write(signalA);
    }
    if (signalB > minangle && signalB < maxangle) {
      servoB.write(signalB);
    }
    if (signalC > minangle && signalC < maxangle) {
      servoC.write(signalC);
    }
  }
}

void recieve() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");
  reco = strtokIndx;

  strtokIndx = strtok(NULL, ",");
  signalA = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  signalB = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  signalC = atoi(strtokIndx);

}


void recover() {
  digitalWrite(enbl, LOW); //ENABLE STEPPER

  //bring platform into reset position
  servoA.write(40);
  servoB.write(55);
  servoC.write(40);

  delay(2000); //wait for ball to get to elevator
  //step
  int  steps = 0 ;
  while (steps <= 630) {
    digitalWrite(pul, HIGH);
    delay(2);
    digitalWrite(pul, LOW);
    delay(2);
    steps++;
  }
  steps = 0;
  digitalWrite(dir, HIGH);
  while (steps <= 40) {
    digitalWrite(pul, HIGH);
    delay(10);
    digitalWrite(pul, LOW);
    delay(10);
    steps++;
  }
  homely();
}

void homely() {

  digitalWrite(enbl, LOW); //enable motor
  digitalWrite(dir, HIGH);
  while (digitalRead(homesw) == HIGH) { //GO DOWN
    //step
    digitalWrite(pul, HIGH);
    delayMicroseconds(1000);
    digitalWrite(pul, LOW);
    delayMicroseconds(1000);
  }
  digitalWrite(dir, LOW);
  while (digitalRead(homesw) == LOW) { //GO UP SLOWLY
    //step
    digitalWrite(pul, HIGH);
    delay(10);
    digitalWrite(pul, LOW);
    delay(10);
  }
  int count = 0;
  while (count < 5) {
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
