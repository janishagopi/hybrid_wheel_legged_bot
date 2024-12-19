#include <NewPing.h>

// Define ultrasonic sensor pins
const int trigPin = 8; // Trigger pin
const int echoPin = 12; // Echo pin

// Define servo pins on the Nano servo shield
#define rightLegServo1Pin 3 // 180-degree servo for right leg
#define rightLegServo2Pin 5 // 180-degree servo for right leg
#define leftLegServo3Pin 6  // 180-degree servo for left leg
#define leftLegServo4Pin 9   // 180-degree servo for left leg

// Variables for ultrasonic sensor
long duration;
int distance;
bool inWheelMotion = false; // Track if the bot is in wheel motion

// Ultrasonic sensor setup
#define MAX_DISTANCE 200
NewPing sonar(trigPin, echoPin, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);

  // Set all servos to the neutral position at the start
  setServoNeutralPosition();
}

void loop() {
  // Perform 7 complete walking cycles
  for (int i = 0; i < 7; i++) {
    if (checkForObstacle()) {
      performBackwardMotion();  // Perform backward motion if an obstacle is detected
      break;
    }

    // Right leg forward (left leg servos function)
    smoothServoMove(leftLegServo3Pin, 90, 150, 400);  // Left leg servo 3 moves to 60 degrees
    smoothServoMove(leftLegServo4Pin, 90, 30, 400); // Left leg servo 4 moves to 120 degrees
    smoothServoMove(leftLegServo3Pin, 150, 90, 400);  // Return left leg servo 3 to 90 degrees
    smoothServoMove(leftLegServo4Pin, 30, 90, 400); // Return left leg servo 4 to 90 degrees

    // Left leg forward (right leg servos function)
    smoothServoMove(rightLegServo1Pin, 90, 30, 400); // Right leg servo 1 moves to 120 degrees
    smoothServoMove(rightLegServo2Pin, 90, 150, 400);  // Right leg servo 2 moves to 60 degrees
    smoothServoMove(rightLegServo1Pin, 30, 90, 400); // Return right leg servo 1 to 90 degrees
    smoothServoMove(rightLegServo2Pin, 150, 90, 400);  // Return right leg servo 2 to 90 degrees
  }

  // Perform wheel motion for 5 seconds
  performWheelMotion();
}

// Function to check for an obstacle using the ultrasonic sensor
bool checkForObstacle() {
  duration = sonar.ping_cm();
  if (duration > 0 && duration < 10) {  // Adjust this threshold as needed
    return true;
  }
  return false;
}

// Function for smooth servo motion
void smoothServoMove(int servoPin, int startAngle, int endAngle, int durationMs) {
  int steps = 50;  // Number of steps for smooth movement
  int delayPerStep = durationMs / steps;  // Delay between each step
  int angleStep = (endAngle - startAngle) / steps;

  for (int i = 0; i <= steps; i++) {
    int currentAngle = startAngle + (angleStep * i);
    analogWrite(servoPin, angleToPWM(currentAngle));
    delay(delayPerStep);
  }
}

// Function to perform backward motion
void performBackwardMotion() {
  if (inWheelMotion) {
    reverseWheelMotion();  // Perform slow reverse when an obstacle is detected during wheel motion
  } else {
    // Define starting and end positions for backward motion
    int startPosition = 90;  // Neutral position (90 degrees)
    int endPosition = 120;   // Final position for backward motion
    int steps = 50;          // Number of steps for gradual movement
    int delayPerStep = 2000 / steps;  // 2 seconds distributed over the steps

    // Ensure wheels are in contact with the ground
    analogWrite(leftLegServo3Pin, angleToPWM(180));  // Rotate left leg servo 3 anticlockwise to 180 degrees
    analogWrite(rightLegServo1Pin, angleToPWM(0));   // Rotate right leg servo 1 clockwise to 0 degrees
    delay(500);

    // Rotate servos for backward motion
    for (int pos = startPosition; pos <= endPosition; pos += 1) {
      analogWrite(leftLegServo4Pin, angleToPWM(180 - pos)); // Left leg servo 4 moves clockwise
      analogWrite(rightLegServo2Pin, angleToPWM(pos));      // Right leg servo 2 moves anticlockwise
      delay(delayPerStep);
    }

    delay(100);  // Optional delay before reversing

    // Gradually move the servos back to 90 degrees
    for (int pos = endPosition; pos >= startPosition; pos -= 1) {
      analogWrite(leftLegServo4Pin, angleToPWM(180 - pos)); // Left leg servo 4 returns clockwise to neutral
      analogWrite(rightLegServo2Pin, angleToPWM(pos));      // Right leg servo 2 returns anticlockwise to neutral
      delay(delayPerStep);
    }

    // Ensure wheels are in contact with the ground
    analogWrite(leftLegServo3Pin, angleToPWM(90));  // Rotate left leg servo 3 anticlockwise to 180 degrees
    analogWrite(rightLegServo1Pin, angleToPWM(90));   // Rotate right leg servo 1 clockwise to 0 degrees
    delay(500);
  }
}

// Function to perform wheel motion
void performWheelMotion() {
  inWheelMotion = true;

  // Step 1: Ensure wheels are in contact with the ground
  analogWrite(leftLegServo3Pin, angleToPWM(180));   // Rotate left leg servo 3 to 180 degrees
  analogWrite(rightLegServo1Pin, angleToPWM(0));    // Rotate right leg servo 1 to 0 degrees
  delay(500);

  // Step 2: Rotate servos for forward motion for 5 seconds
  int startPosition = 0;   // Neutral position
  int endPosition = 180;   // Final position
  int steps = 50;          // Number of steps
  int delayPerStep = 5000 / steps;  // 5 seconds distributed over the steps

  // Gradual forward motion
  for (int pos = startPosition; pos <= endPosition; pos += 1) {
    analogWrite(rightLegServo2Pin, angleToPWM(pos)); // Right leg servo 2 moves clockwise
    analogWrite(leftLegServo4Pin, angleToPWM(endPosition - pos)); // Left leg servo 4 moves anticlockwise
    delay(delayPerStep);

    // Check for obstacles during forward motion
    if (checkForObstacle()) {
      reverseWheelMotion();  // Reverse motion for 2 seconds
      return;                // Exit the forward loop if an obstacle is detected
    }
  }

  delay(100);  // Optional delay before reversing

  // Gradual backward motion
  for (int pos = endPosition; pos >= startPosition; pos -= 1) {
    analogWrite(rightLegServo2Pin, angleToPWM(pos)); // Right leg servo 2 moves anticlockwise
    analogWrite(leftLegServo4Pin, angleToPWM(endPosition - pos)); // Left leg servo 4 moves clockwise
    delay(delayPerStep);

    // Check for obstacles during backward motion
    if (checkForObstacle()) {
      reverseWheelMotion();  // Reverse motion for 2 seconds
      return;                // Exit the backward loop if an obstacle is detected
    }
  }

  delay(1000);  // Wait for 1 second before repeating

  // Ensure wheels are in contact with the ground
  analogWrite(leftLegServo3Pin, angleToPWM(90));  // Rotate left leg servo 3 anticlockwise to 180 degrees
  analogWrite(rightLegServo1Pin, angleToPWM(90));   // Rotate right leg servo 1 clockwise to 0 degrees
  delay(500);

  inWheelMotion = false;  // Reset state after forward wheel motion
}

// Function to reverse wheel motion for 2 seconds
void reverseWheelMotion() {
  // Gradually reverse servos for 2 seconds (backward motion)
  int startPosition = 180;  // Start from the position achieved
  int endPosition = 90;     // End position (neutral)

  for (int pos = startPosition; pos >= endPosition; pos--) {
    analogWrite(rightLegServo2Pin, angleToPWM(pos)); // Reverse motion for right leg servo 2
    analogWrite(leftLegServo4Pin, angleToPWM(180 - pos)); // Reverse motion for left leg servo 4
    delay(40);  // Adjust delay as needed for speed
  }

  delay(500); // Pause after reversing
}

// Function to set all servos to neutral position
void setServoNeutralPosition() {
  analogWrite(leftLegServo3Pin, angleToPWM(90));
  analogWrite(leftLegServo4Pin, angleToPWM(90));
  analogWrite(rightLegServo1Pin, angleToPWM(90));
  analogWrite(rightLegServo2Pin, angleToPWM(90));
}

// Function to convert angle to PWM value (0-255)
int angleToPWM(int angle) {
  return map(angle, 0, 180, 0, 255); // Map 0-180 degrees to PWM range
}