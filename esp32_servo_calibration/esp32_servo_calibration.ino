#include <Wire.h> // Required for I2C communication with PCA9685
#include <Adafruit_PWMServoDriver.h> // PCA9685 library

// --- PCA9685 Configuration ---
// PCA9685 I2C address (default is 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Standard servo refresh rate (50 Hz)
#define SERVO_FREQ 50

// --- Calibration Servo Channel ---
// Connect the SG90 servo you want to calibrate to this channel on your PCA9685.
// You'll test one servo at a time.
const int servoToCalibrateChannel = 0; // Example: Connect your servo to channel 0

// --- Initial Test Pulse Ranges ---
// These are generous (wide) ranges to ensure you can reach the servo's full physical limits.
// You will find your actual safe limits within these.
#define SERVOMIN_TEST  650  // A very low pulse value (roughly 0.4ms)
#define SERVOMAX_TEST  2000 // A very high pulse value (roughly 3.1ms)

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  Serial.println("\n--- SG90 Servo Calibration Sketch ---");
  Serial.println("Connect ONE SG90 servo to PCA9685 Channel " + String(servoToCalibrateChannel));
  Serial.println("Open Serial Monitor (115200 baud, No line ending)");
  Serial.println(" ");

  // --- Initialize I2C Bus ---
  Wire.begin();

  // --- Initialize PCA9685 ---
  Serial.print("Initializing PCA9685...");
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ); // Set PWM frequency for servos (50 Hz)
  Serial.println("SUCCESS!");

  // --- Set Initial Servo Pulse ---
  // Start the servo in a safe, known approximate center position
  int initialPulse = (SERVOMIN_TEST + SERVOMAX_TEST) / 2;
  pwm.setPWM(servoToCalibrateChannel, 0, initialPulse);
  Serial.print("Servo initialized to pulse: ");
  Serial.println(initialPulse);
  Serial.println(" ");

  Serial.println("--- Calibration Commands ---");
  Serial.println(" 'u' : Increase pulse (move towards max angle) by 1 tick");
  Serial.println(" 'd' : Decrease pulse (move towards min angle) by 1 tick");
  Serial.println(" 'r' : Reset servo to initial mid-point");
  Serial.println(" 'q' : Quit calibration (halting sketch execution)");
  Serial.println(" Any other key: Print current pulse value");
  Serial.println("\nObserve servo movement carefully. STOP IMMEDIATELY if you hear grinding or straining!");
}

void loop() {
  // Static variable to retain its value between loop calls
  static int currentPulse = (SERVOMIN_TEST + SERVOMAX_TEST) / 2;
  const int step = 10; // Increment/decrement step size (adjust if you want larger steps)

  if (Serial.available()) { // Check if a character has been sent via Serial Monitor
    char command = Serial.read(); // Read the character

    switch (command) { // Process the received command
      case 'u': // Increase pulse
        currentPulse += step;
        // Constrain to prevent going excessively high beyond test range
        if (currentPulse > SERVOMAX_TEST) currentPulse = SERVOMAX_TEST;
        Serial.print("Increased pulse: ");
        break;
      case 'd': // Decrease pulse
        currentPulse -= step;
        // Constrain to prevent going excessively low beyond test range
        if (currentPulse < SERVOMIN_TEST) currentPulse = SERVOMIN_TEST;
        Serial.print("Decreased pulse: ");
        break;
      case 'r': // Reset to mid-point
        currentPulse = (SERVOMIN_TEST + SERVOMAX_TEST) / 2;
        Serial.print("Reset pulse to mid-point: ");
        break;
      case 'q': // Quit/Halt
        Serial.println("\nCalibration stopped. Note your values!");
        Serial.print("Final Pulse: ");
        Serial.println(currentPulse);
        while(1); // Halt execution indefinitely
        break;
      default: // Any other character just prints current value
        Serial.print("Current pulse (no change): ");
        break;
    }

    // Apply the new pulse value to the servo
    pwm.setPWM(servoToCalibrateChannel, 0, currentPulse);

    // Print the current pulse value for observation
    Serial.println(currentPulse);
  }
}