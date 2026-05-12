// Rover Arduino Sketch
// This program is responsible for:
// 1. Receiving single-character commands over Serial from the Pi
// 2. Driving two DC motors (forward, backward, left, right, stop)
// 3. Controlling a servo motor to tilt the camera up/down/centre
// 4. Automatically stopping the rover if no command arrives in time
//
// Commands reference:
//   F / f = Forward
//   B / b = Backward
//   L / l = Turn Left
//   R / r = Turn Right
//   S / s = Stop
//   I / i = Camera tilt up
//   K / k = Camera tilt down
//   M / m = Camera centre (return to 90°)

#include <Servo.h>   // Built-in Arduino library for controlling servo motors

// Servo Setup

Servo camServo;            // The servo object that controls the camera tilt
const int servoPin = 3;    // Digital pin the servo signal wire is connected to
int angle = 90;            // Starting angle 
const int stepSize = 5;    // How many degrees to move per tilt command

// L298N Motor Driver Pin Definitions

#define ENA 5    // Motor A speed (PWM)
#define ENB 6    // Motor B speed (PWM)
#define IN1 8    // Motor A direction pin 1
#define IN2 7    // Motor A direction pin 2
#define IN3 11   // Motor B direction pin 1
#define IN4 9    // Motor B direction pin 2

// Motor Speed Settings

int maxSpeed  = 200;   // Speed used when driving straight (0-255)
int turnSpeed = 230;   // Speed used when turning

// Safety Timeout
// If no command is received for 500ms, the rover stops automatically.

unsigned long lastCommandTime = 0;           // Timestamp of the last received command
const unsigned long COMMAND_TIMEOUT = 500;   // Stop after 500 milliseconds of silence

char currentCommand = 'S';   // The most recently received drive command

// Setup 
void setup() {
  // Start serial communication at 9600 baud same on the Raspberry Pi
  Serial.begin(9600);

  // Attach the servo to its pin and move it to the starting angle
  camServo.attach(servoPin);
  camServo.write(angle);   // Move to 90°

  // Set all motor driver pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Make sure the motors are stopped before anything begins
  stopMotors();
}
// Motor Functions

// Drive both motors forward
void motorForwardRaw() {
  analogWrite(ENA, turnSpeed);   // Set Motor A speed
  analogWrite(ENB, turnSpeed);   // Set Motor B speed

  digitalWrite(IN1, LOW);      // Motor A: forward
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);      // Motor B: forward
  digitalWrite(IN4, LOW);
}

// Drive both motors backward
void motorBackwardRaw() {
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);

  digitalWrite(IN1, HIGH);   // Motor A: backward
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);   // Motor B: backward
  digitalWrite(IN4, HIGH);
}

// Spin in place to the left:
// Motor A goes backward, Motor B goes forward
void motorLeftRaw() {
  analogWrite(ENA, maxSpeed);
  analogWrite(ENB, maxSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Spin in place to the right:
// Motor A goes forward, Motor B goes backward
void motorRightRaw() {
  analogWrite(ENA, maxSpeed);
  analogWrite(ENB, maxSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Stop both motors by setting speed to 0 and all direction pins LOW
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Servo Motor Control
void controlServo(char cmd) {
  // Adjust the target angle based on the command
  if (cmd == 'I' || cmd == 'i') {
    angle += stepSize;   // Tilt up
  }
  else if (cmd == 'K' || cmd == 'k') {
    angle -= stepSize;   // Tilt down
  }
  else if (cmd == 'M' || cmd == 'm') {
    angle = 90;          // Centre
  }
  // Set the angle between 0° and 180° to protect the servo from over-rotating
  angle = constrain(angle, 0, 180);

  // Send the new angle to the servo
  camServo.write(angle);
}

// Motor Control Commands
// ============================================================
void controlMotors(char cmd) {
  // Route the command character to the correct motor function.
  // Note: the physical wiring on this rover swaps F/B and L/R,
  // so the switch cases call what looks like the "wrong" function —
  // this is intentional and corrects for the wiring layout.
  switch (cmd) {

    case 'F': case 'f':
      motorForwardRaw();     // Wiring compensation: 'F' drives the left-raw function
      break;

    case 'B': case 'b':
      motorBackwardRaw();    // Wiring compensation: 'B' drives the right-raw function
      break;

    case 'L': case 'l':
      motorLeftRaw();  // Wiring compensation: 'L' drives forward-raw
      break;

    case 'R': case 'r':
      motorRightRaw(); // Wiring compensation: 'R' drives backward-raw
      break;

    case 'S': case 's':
      stopMotors();
      break;

    default:
      // Unknown command — stop the rover as a safety measure
      stopMotors();
      break;
  }
}

// Main loop
void loop() {

  // Step 1: Check for incoming serial commands
  if (Serial.available()) {
    char cmd = Serial.read(); // Read one character from the Pi

    // Check if it's a servo command (camera tilt)
    if (cmd == 'I' || cmd == 'i' ||
        cmd == 'K' || cmd == 'k' ||
        cmd == 'M' || cmd == 'm') {
      controlServo(cmd); // Handle it immediately
    }

    // Otherwise check if it's a drive command
    else if (cmd == 'F' || cmd == 'f' ||
             cmd == 'B' || cmd == 'b' ||
             cmd == 'L' || cmd == 'l' ||
             cmd == 'R' || cmd == 'r' ||
             cmd == 'S' || cmd == 's') {
      currentCommand   = cmd;       // Store it for use in Step 3
      lastCommandTime  = millis();  // Reset the timeout timer
    }
    // Any other character is ignored
  }

  // Step 2: Safety timeout check
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    currentCommand = 'S';
  }

  // Step 3: Execute the current drive command
  // This runs every loop iteration, keeping the motors in the correct state.
  controlMotors(currentCommand);
}
