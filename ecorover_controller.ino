#include <Servo.h>

// Cytron MDD3A assumed configured for PWM/PWM control mode.
const byte LEFT_A = 5;
const byte LEFT_B = 6;
const byte RIGHT_A = 9;
const byte RIGHT_B = 10;

// HC-SR04
const byte TRIG_PIN = 7;
const byte ECHO_PIN = 8;

// Four arm servos: base, shoulder, elbow, gripper
const byte SERVO_PINS[4] = {3, 4, 11, 12};
Servo arm[4];

// Example poses only; calibrate to the physical arm before operation.
const int HOME_POSE[4]  = {90, 90, 90, 90};
const int REACH_POSE[4] = {90, 120, 65, 90};
const int GRIP_POSE[4]  = {90, 120, 65, 45};
const int LIFT_POSE[4]  = {90, 70, 110, 45};

String input = "";
unsigned long lastDistanceReport = 0;
const unsigned long DISTANCE_INTERVAL_MS = 150;

void motorWrite(byte pinA, byte pinB, int speedValue) {
  speedValue = constrain(speedValue, -100, 100);
  int pwm = map(abs(speedValue), 0, 100, 0, 255);
  if (speedValue > 0) {
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else if (speedValue < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, pwm);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

void stopMotors() {
  motorWrite(LEFT_A, LEFT_B, 0);
  motorWrite(RIGHT_A, RIGHT_B, 0);
}

void drive(int leftSpeed, int rightSpeed) {
  motorWrite(LEFT_A, LEFT_B, leftSpeed);
  motorWrite(RIGHT_A, RIGHT_B, rightSpeed);
}

float measureDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (duration == 0) return -1.0;
  return duration * 0.0343 / 2.0;
}

void setArmPose(const int pose[4]) {
  for (byte i = 0; i < 4; i++) {
    arm[i].write(constrain(pose[i], 0, 180));
  }
  delay(650);
}

void pickupWaste() {
  stopMotors();
  setArmPose(REACH_POSE);
  setArmPose(GRIP_POSE);
  setArmPose(LIFT_POSE);
  setArmPose(HOME_POSE);
  Serial.println("PICKUP_COMPLETE");
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd == "STOP") {
    stopMotors();
    Serial.println("MOTORS_STOPPED");
  } else if (cmd == "PICK") {
    pickupWaste();
  } else if (cmd.startsWith("MOVE ")) {
    int splitAt = cmd.indexOf(' ', 5);
    if (splitAt < 0) {
      stopMotors();
      Serial.println("INVALID_MOVE");
      return;
    }
    int left = cmd.substring(5, splitAt).toInt();
    int right = cmd.substring(splitAt + 1).toInt();
    drive(left, right);
    Serial.println("MOVING");
  } else {
    stopMotors();
    Serial.println("UNKNOWN_COMMAND");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  stopMotors();

  for (byte i = 0; i < 4; i++) {
    arm[i].attach(SERVO_PINS[i]);
    arm[i].write(HOME_POSE[i]);
  }
  Serial.println("ECOROVER_READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        processCommand(input);
        input = "";
      }
    } else if (input.length() < 80) {
      input += c;
    } else {
      input = "";
      stopMotors();
    }
  }

  if (millis() - lastDistanceReport >= DISTANCE_INTERVAL_MS) {
    float distance = measureDistanceCm();
    Serial.print("DISTANCE_CM=");
    if (distance < 0) Serial.println("INVALID");
    else Serial.println(distance, 1);
    lastDistanceReport = millis();
  }
}
