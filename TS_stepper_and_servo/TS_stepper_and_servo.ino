// 3 Servos + Stepper Motor Control Program
// Teensy 4.1 + PCA9685 (3 Servos) + TB6600 (Stepper)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Stepper Motor Pins (TB6600)
const int stepPin = 3;
const int dirPin = 2;
const int enablePin = 4;

// Servo Settings
#define SERVO_FREQ 50
#define SERVOMIN 150
#define SERVOMAX 600
#define NUM_SERVOS 3  // Now we have 3 servos

// Servo channels
const int SERVO_CHANNELS[NUM_SERVOS] = {0, 1, 15};
int servoPositions[NUM_SERVOS] = {90, 90, 90}; // Track positions

// Motor specifications
const int stepsPerRevolution = 200;
const int microstepping = 16;
const int totalSteps = stepsPerRevolution * microstepping;

void setup() {
  Serial.begin(9600);
  Serial.println("🤖 3 Servos + Stepper Control System");
  
  // Initialize Stepper Motor pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); // Enable stepper
  
  // Initialize PCA9685 for Servos
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Initialize all servos to center position
  for (int i = 0; i < NUM_SERVOS; i++) {
    moveServo(i, 90);
    delay(200); // Small delay between servo movements
  }
  
  Serial.println("✅ All motors initialized!");
  Serial.println("Commands:");
  Serial.println("- SERVO <channel> <angle>  (channel: 0-2, angle: 0-180)");
  Serial.println("- STEPPER <steps>          (positive=CW, negative=CCW)");
  Serial.println("- ALL <angle>              (move all servos to angle)");
  Serial.println("- WAVE                     (servo wave pattern)");
  Serial.println("- DEMO                     (full demo sequence)");
  Serial.println("- STATUS                   (show all positions)");
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  // Auto demo every 20 seconds
  static unsigned long lastDemo = 0;
  if (millis() - lastDemo > 20000) {
    runAutoDemo();
    lastDemo = millis();
  }
}

void processCommand(String cmd) {
  cmd.toUpperCase();
  
  if (cmd.startsWith("SERVO")) {
    // Format: "SERVO 0 90" (channel, angle)
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      int channel = cmd.substring(firstSpace + 1, secondSpace).toInt();
      int angle = cmd.substring(secondSpace + 1).toInt();
      moveServo(channel, angle);
    }
  }
  else if (cmd.startsWith("STEPPER")) {
    int space = cmd.indexOf(' ');
    if (space > 0) {
      int steps = cmd.substring(space + 1).toInt();
      moveStepper(steps);
    }
  }
  else if (cmd.startsWith("ALL")) {
    int space = cmd.indexOf(' ');
    if (space > 0) {
      int angle = cmd.substring(space + 1).toInt();
      moveAllServos(angle);
    }
  }
  else if (cmd == "WAVE") {
    servoWavePattern();
  }
  else if (cmd == "DEMO") {
    runFullDemo();
  }
  else if (cmd == "STATUS") {
    showStatus();
  }
  else {
    Serial.println("❌ Unknown command");
  }
}

void moveServo(int channel, int angle) {
  if (channel < 0 || channel >= NUM_SERVOS) {
    Serial.println("❌ Invalid servo channel. Use 0-2");
    return;
  }
  
  if (angle < 0 || angle > 180) {
    Serial.println("❌ Servo angle must be 0-180");
    return;
  }
  
  // Convert angle to pulse width
  int pulseWidth = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  
  // Send signal to servo
  pwm.setPWM(SERVO_CHANNELS[channel], 0, pulseWidth);
  servoPositions[channel] = angle;
  
  Serial.print("🎯 Servo ");
  Serial.print(channel);
  Serial.print(" moved to ");
  Serial.print(angle);
  Serial.println("°");
}

void moveAllServos(int angle) {
  Serial.print("🎯 Moving all servos to ");
  Serial.print(angle);
  Serial.println("°");
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    moveServo(i, angle);
    delay(100); // Stagger movements to reduce power surge
  }
  
  Serial.println("✅ All servos moved");
}

void moveStepper(int steps) {
  if (steps == 0) {
    Serial.println("❌ Steps cannot be zero");
    return;
  }
  
  // Set direction
  bool clockwise = steps > 0;
  digitalWrite(dirPin, clockwise ? HIGH : LOW);
  
  // Get absolute steps
  int absSteps = abs(steps);
  
  Serial.print("🔄 Stepper moving ");
  Serial.print(absSteps);
  Serial.print(" steps ");
  Serial.println(clockwise ? "clockwise" : "counter-clockwise");
  
  // Execute steps
  for (int i = 0; i < absSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }
  
  Serial.println("✅ Stepper movement completed");
}

void servoWavePattern() {
  Serial.println("🌊 Creating servo wave pattern...");
  
  // Wave motion across all 3 servos
  for (int wave = 0; wave < 3; wave++) {
    // Forward wave
    for (int i = 0; i < NUM_SERVOS; i++) {
      moveServo(i, 45);
      delay(200);
    }
    
    // Backward wave
    for (int i = NUM_SERVOS - 1; i >= 0; i--) {
      moveServo(i, 135);
      delay(200);
    }
  }
  
  // Return all to center
  moveAllServos(90);
  Serial.println("✅ Wave pattern completed");
}

void runFullDemo() {
  Serial.println("🎭 Running full demo sequence...");
  
  // Demo 1: Sequential servo movement
  Serial.println("Demo 1: Sequential servo sweep");
  for (int angle = 0; angle <= 180; angle += 45) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      moveServo(i, angle);
      delay(300);
    }
  }
  
  // Demo 2: Coordinated with stepper
  Serial.println("Demo 2: Servos + Stepper coordination");
  moveAllServos(0);
  delay(500);
  moveStepper(800);  // Quarter turn
  
  moveAllServos(180);
  delay(500);
  moveStepper(-800); // Return
  
  // Demo 3: Wave pattern
  servoWavePattern();
  
  // Demo 4: Individual servo dance
  Serial.println("Demo 4: Servo dance");
  for (int i = 0; i < 5; i++) {
    moveServo(0, random(0, 181));
    moveServo(1, random(0, 181));
    moveServo(2, random(0, 181));
    delay(500);
  }
  
  // Return to center
  moveAllServos(90);
  
  Serial.println("✅ Full demo completed!");
}

void runAutoDemo() {
  Serial.println("🔄 Auto demo: Coordinated movement");
  
  // Simple coordinated pattern
  moveServo(0, 0);
  moveServo(1, 90);
  moveServo(2, 180);
  delay(1000);
  
  moveStepper(1600); // Half revolution
  
  moveServo(0, 180);
  moveServo(1, 90);
  moveServo(2, 0);
  delay(1000);
  
  moveStepper(-1600); // Return
  
  // Return servos to center
  moveAllServos(90);
  
  Serial.println("✅ Auto demo completed");
}

void showStatus() {
  Serial.println("📊 Current Motor Status:");
  Serial.println("========================");
  
  // Servo positions
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(servoPositions[i]);
    Serial.println("°");
  }
  
  Serial.println("Stepper: Ready");
  Serial.println("========================");
}

// Advanced function: Smooth servo movement
void smoothMoveServo(int channel, int targetAngle, int stepDelay = 20) {
  if (channel < 0 || channel >= NUM_SERVOS) return;
  if (targetAngle < 0 || targetAngle > 180) return;
  
  int currentAngle = servoPositions[channel];
  int step = (targetAngle > currentAngle) ? 1 : -1;
  
  while (currentAngle != targetAngle) {
    currentAngle += step;
    moveServo(channel, currentAngle);
    delay(stepDelay);
  }
}
