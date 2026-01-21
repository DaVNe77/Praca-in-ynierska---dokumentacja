#include <AccelStepper.h>
#include <Servo.h>

// Ustawienie pinów STEP/ DIR
#define DIR1_PIN 9
#define STEP1_PIN 8

#define DIR2_PIN 11
#define STEP2_PIN 10

#define DIR3_PIN 13
#define STEP3_PIN 12

// Ustawienie mikrokroku dla osi 1
#define M1_MS1_PIN 2
#define M1_MS2_PIN 3
#define M1_MS3_PIN 4
// Ustawienie pinu sygnałowgo dla serwa
#define SERVO_PIN 5 

// SERWO
Servo myServo;
int servoPos = 15; // Pozycja początkowa serwomechanizmu

AccelStepper step1(AccelStepper::DRIVER, STEP1_PIN, DIR1_PIN);
AccelStepper step2(AccelStepper::DRIVER, STEP2_PIN, DIR2_PIN);
AccelStepper step3(AccelStepper::DRIVER, STEP3_PIN, DIR3_PIN);

// Prędkość: w steps/second
const float MAXSPEED1 = 1000000.0f / 12000.0f; // ~83.3
const float MAXSPEED2 = 1000000.0f / 7000.0f;  // 125
const float MAXSPEED3 = 1000000.0f / 7000.0f;  // 125

// Przyspieszenie
const float ACC1 = 250.0f;
const float ACC2 = 350.0f;
const float ACC3 = 350.0f;

// Komunikacja przez port szeregowy

String inputString = "";
bool stringComplete = false;

void setup() {
// Mikrokrok 
  pinMode(M1_MS1_PIN, OUTPUT);
  pinMode(M1_MS2_PIN, OUTPUT);
  pinMode(M1_MS3_PIN, OUTPUT);

  digitalWrite(M1_MS1_PIN, HIGH);
  digitalWrite(M1_MS2_PIN, LOW);
  digitalWrite(M1_MS3_PIN, LOW);

// Sterowanie serwo
  myServo.attach(SERVO_PIN);
  myServo.write(servoPos);
// Parametry ruchu
  step1.setMaxSpeed(MAXSPEED1);
  step2.setMaxSpeed(MAXSPEED2);
  step3.setMaxSpeed(MAXSPEED3);

  step1.setAcceleration(ACC1);
  step2.setAcceleration(ACC2);
  step3.setAcceleration(ACC3);

  // Pozycja startowa = 0
  step1.setCurrentPosition(0);
  step2.setCurrentPosition(0);
  step3.setCurrentPosition(0);

  Serial.begin(9600);
  inputString.reserve(120);
}

void loop() {
  // Odbiór i przetwarzanie komend
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Ruch osi
  step1.run();
  step2.run();
  step3.run();
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}


//'1','2','3'- pozycje w krokach
//'V' - serwo kąt
//'H'- zerowanie 
void parseCommand(String cmd) {
  int sep = cmd.indexOf(':');
  if (sep == -1) return;

  char id = cmd.charAt(0);
  long val = cmd.substring(sep + 1).toInt();

  if (id == '1') {
    step1.moveTo(val);
  } else if (id == '2') {
    step2.moveTo(val);
  } else if (id == '3') {
    step3.moveTo(val);
  } else if (id == 'V') {
    servoPos = constrain(val, 0, 180);
    myServo.write(servoPos);
  } else if (id == 'H') {

    step1.setCurrentPosition(0); step1.moveTo(0);
    step2.setCurrentPosition(0); step2.moveTo(0);
    step3.setCurrentPosition(0); step3.moveTo(0);
  }
}
