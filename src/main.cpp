#include "Arduino.h"
#include "SoftwareSerial.h"
#include "EEPROM.h"
#include "Motors.h"
#include "LightSensor.h"
#include "UltrasonicSensor.h"

#define LINE_INTERRUPTION 1
#define OIL_SPOIL 2
#define OBSTACLE_FAR 3
#define OBSTACLE 4

// Motor and light sensor pins.
int motorsPins[4] = {5, 10, 9, 6};
int lightSensorsPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int numberLightSensors = 8;
int BTRx = 12;
int BTTx = 11;
int echoPin = 7;
int trigPin = 8;

int buttonPin = 2;
int obstacleSwitchPin = 3;

SoftwareSerial BT(BTRx, BTTx);
Motors motors(motorsPins);
LightSensor lightSensors[8];
UltrasonicSensor USSensor(echoPin, trigPin);

int previousEvent = 0;

// Kp, Kd constants - you have to experiment with them to have good results.
float Kp = 1;
float Ki = 1;
float Kd = 1;
// Last error.
int lastError = 0;

// Calibration values for light sensors - you have to set them depending on the light conditions
int blackSensorValue = 500;
int whiteSensorValue = 100;

// Base speed for motors.
int baseSpeed = 220;

bool running = false;
long time = 0;
int debounce = 200;
int previous = HIGH;
// Function declaration.
void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB);
int readLightSensorDigital(int sensor);
int getError(int *sensorsReadValue);
void getBluetoothData(float *kp, float *ki, float *kd, int *speed);
int checkEvent(int *lightSensorsValue, float distance);
int getSensorsOnLine(int *sensorsReadValue);
void interrupt();
void save();

void setup() {
  // Set interrupt button pin as INPUT_PULLUP.
  pinMode(buttonPin, INPUT_PULLUP);

  // Start serial communication.
  Serial.begin(9600);

  // Start bluetooth communication.
  BT.begin(9600);

  // Setup light sensors.
  for (int i = 0; i < 8; i++) {
    lightSensors[i] = LightSensor(lightSensorsPins[i]);
  }

  Kp = EEPROM.read(0);
  Ki = EEPROM.read(1);
  Kd = EEPROM.read(2);
  baseSpeed = EEPROM.read(3);
  Serial.println(Kp);
  Serial.println(Ki);
  Serial.println(Kd);
  Serial.println(baseSpeed);
}

void loop() {
  // If running = true.
  if (running) {
    // Light sensors readings array.
    int lightSensorsReading[numberLightSensors];
    float distance = USSensor.getDistance();

    // Read light sensors.
    for (int i = 0; i < numberLightSensors; i++) {
      lightSensorsReading[i] = readLightSensorDigital(i);
    }

    int event = checkEvent(lightSensorsReading, distance);
    // Get error based on the light sensors reading.
    int error = getError(lightSensorsReading);
    // Serial.println(error);

    int speedA = 0;
    int speedB = 0;

    // Calculate PID value based on error and Kp and Kd constants.
    calculatePID(error, Kp, Kd, &speedA, &speedB);

    // Move motors with speeds returned by PID algorhitm.
    motors.moveTank(speedA, speedB);

    // If data from bluetooth are available.
    if (BT.available()) {
      // Get Kp and Kd from the app.
      getBluetoothData(&Kp, &Ki, &Kd, &baseSpeed);

      // Debug messages.
      Serial.print(Kp);
      Serial.print("\t");
      Serial.print(Ki);
      Serial.print("\t");
      Serial.print(Kd);
      Serial.print("\t");
      Serial.print(baseSpeed);
      Serial.print("\n");
    }
  }

  else {
    motors.forward(0);
  }

  if (digitalRead(buttonPin) == LOW && previous == HIGH && millis() - time > debounce) {
    Serial.println("Change");
    if (running == HIGH)
      running = LOW;
    else
      running = HIGH;

    time = millis();
  }

  previous = digitalRead(buttonPin);
}

int checkEvent(int *lightSensorsValue, float distance) {
  int sensorsOnLine = getSensorsOnLine(lightSensorsValue);
  int event = 0;

  if (lastError < 2 && sensorsOnLine) {
    event = LINE_INTERRUPTION;
  }

  if (sensorsOnLine == numberLightSensors) {
    event = OIL_SPOIL;
  }

  if (distance < 15) {
    event = OBSTACLE_FAR;
  }

  return event;
}

void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB) {
  // Proportional.
  int P = error;
  int I = I + error;
  // Derivative.
  int D = error - lastError;
  // PD value.
  int PID = P * Kp + I * Ki + D * Kd;

  // Calculate speedA and speedB values based on PD value.
  *speedA = baseSpeed + PID > 255 ? 255 : baseSpeed + PID;
  *speedB = baseSpeed - PID > 255 ? 255 : baseSpeed - PID;
  // Serial.println(PID);
}

int readLightSensorDigital(int sensor) {
  // Sensor readings array.
  int sensorValue;

    // If light sensor is on black, write 1.
  if (lightSensors[sensor].readLightSensor() >= blackSensorValue) {
    sensorValue = 1;
  }

  // If light sensor is on white, write 0.
  else if (lightSensors[sensor].readLightSensor() <= whiteSensorValue) {
    sensorValue = 0;
  }
  return sensorValue;
}

int getSensorsOnLine(int *sensorsReadValue) {
  int count = 0;
  for (int i = 0; i < numberLightSensors; i++) {
    if (sensorsReadValue[i] == 1) {
      count++;
    }
  }

  return count;
}

int getError(int *sensorsReadValue) {
  // Set error to -4.
  int error = 0;

  for (int i = 0; i < numberLightSensors; i++) {

    Serial.print(sensorsReadValue[i]);
    Serial.print("\t");
    // If sensor is black, increment error.
    if (sensorsReadValue[i] == 1) {
      error = i - 4;
      return error;
    }
  }

  error = -4;

  Serial.print("\n");

  // Return error.
  return error;
}

void getBluetoothData(float *kp, float *ki, float *kd, int *speed) {
  String data = "";
  while (BT.available()) {
    char ReadChar = (char)BT.read();

    if (ReadChar == ')') {
      break;
    } else {
      data += ReadChar;
    }

    delay(10);
  }

  if (data == "s") {
    save();
    *kp = Kp;
    *kd = Kd;
  }

  else {
    int index0 = data.indexOf(':');
    int index1 = data.indexOf(':', index0 + 1);
    int index2 = data.indexOf(':', index1 + 1);
    int index3 = data.indexOf(':', index2 + 1);
    int kP = data.substring(0, index0).toInt();
    int kI = data.substring(index0 + 1, index1).toInt();
    int kD = data.substring(index1 + 1, index2).toInt();
    int Speed = data.substring(index2 + 1, index3).toInt();
    *kp = kP;
    *ki = kI;
    *kd = kD;
    *speed = Speed;
  }
}

void save() {
  EEPROM.write(0, Kp);
  EEPROM.write(1, Ki);
  EEPROM.write(2, Kd);
  EEPROM.write(3, baseSpeed);
  Serial.println("data saved");
  Serial.println(EEPROM.read(0));
  Serial.println(EEPROM.read(1));
  Serial.println(EEPROM.read(2));
  Serial.println(EEPROM.read(3));
}