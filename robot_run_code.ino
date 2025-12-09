#include <EEPROM.h>

int leftMotor1 = 10;
int leftMotor2 = 9;
int rightMotor1 = 6;
int rightMotor2 = 5;

int ledPin = 13;

int TURN_TIME = 500;
int FORWARD_TIME = 5000;

#define START_ADDR 0
#define MAX_LEN 80


void moveForward() {
  analogWrite(leftMotor1, 98);
  analogWrite(leftMotor2, 0);
  analogWrite(rightMotor1, 108);
  analogWrite(rightMotor2, 0);
}

void stopRobot() {
  analogWrite(leftMotor1, 0);
  analogWrite(leftMotor2, 0);
  analogWrite(rightMotor1, 0);
  analogWrite(rightMotor2, 0);
}

void turnLeft90() {
  analogWrite(leftMotor1, 0);
  analogWrite(leftMotor2, 150);
  analogWrite(rightMotor1, 150);
  analogWrite(rightMotor2, 0);
  delay(TURN_TIME + 70);
  stopRobot();
}

void turnRight90() {
  analogWrite(leftMotor1, 140);
  analogWrite(leftMotor2, 0);
  analogWrite(rightMotor1, 0);
  analogWrite(rightMotor2, 140);
  delay(TURN_TIME - 13);
  stopRobot();
}


void saveMovesToEEPROM(String moves) {
  int len = moves.length();
  EEPROM.write(START_ADDR, len);
  for (int i = 0; i < len; i++) {
    EEPROM.write(START_ADDR + 1 + i, moves[i]);
  }
  Serial.println("Moves saved to EEPROM!");
}

String readMovesFromEEPROM() {
  int len = EEPROM.read(START_ADDR);
  String result = "";
  for (int i = 0; i < len; i++) {
    result += (char)EEPROM.read(START_ADDR + 1 + i);
  }
  return result;
}


void moveOnward() {
  moveForward();
  delay(FORWARD_TIME);
  stopRobot();
}


void executeMoves(String moves) {
  moves.trim();
  Serial.println("Executing Moves: " + moves);

  int n = moves.length();

  for (int i = 0; i < n; i++) {
    char c = moves[i];
    if (c == ',') continue;

    Serial.print("Step ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(c);

    if (c == 'S') {
      stopRobot();
    }
    else if (c == 'L') {
      moveOnward();
      delay(200);
      turnLeft90();
    }
    else if (c == 'R') {
      moveOnward();
      delay(200);
      turnRight90();
    }
    else if (c == 'O') {
      Serial.println("Moving Onward (O)");
      moveOnward();
    }
    else if (c == 'F') {
      moveOnward();
      Serial.println("Finished Path!");
      digitalWrite(ledPin, HIGH);
      while (1);
    }

    delay(300);
  }

  Serial.println("All moves completed!");
  digitalWrite(ledPin, HIGH);
  while (1);
}


void setup() {
  Serial.begin(9600);

  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  delay(2000);

  if (Serial.available()) {
    String received = Serial.readStringUntil('\n');
    received.trim();
    saveMovesToEEPROM(received);
  }

  String moves = readMovesFromEEPROM();
  Serial.println("Loaded Moves = " + moves);

  executeMoves(moves);
}

void loop() {}


