#include <Wire.h> // For I2C communication (BNO085 and PCA9685)
#include <Adafruit_BNO08x.h> // BNO085 library
#include <Adafruit_PWMServoDriver.h> // PCA9685 library

// --- BNO085 Configuration ---
// BNO085 I2C address (default is 0x4A or 0x4B, check your module)
// If you have a different address, change it here.
Adafruit_BNO08x bno;

// --- PCA9685 Configuration ---
// PCA9685 I2C address (default is 0x40, can be changed with A0-A5 pins)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Standard servo refresh rate (50 Hz)
#define SERVO_FREQ 50

// --- SG90 Servo Calibration Values (CRITICAL!) ---
// *** IMPORTANT: REPLACE THESE WITH YOUR CALIBRATED VALUES ***
// These values define the min/max 'tick' values (out of 4096) for YOUR specific SG90 servos.
// Use the calibration sketch to find these values for each servo.
// Example values are given, but YOURS WILL LIKELY BE DIFFERENT.
#define SERVOMIN_CALIBRATED 90  // Actual 'tick' value for your servo's physical minimum position (e.g., 0 degrees)
#define SERVOMAX_CALIBRATED 400 // Actual 'tick' value for your servo's physical maximum position (e.g., ~160-170 degrees)

// --- Servo Channel Definitions ---
// Define which PCA9685 channels your servos are connected to
const int pitchServoChannel = 0; // Servo for Pitch control, connected to PCA9685 channel 0
const int rollServoChannel  = 1; // Servo for Roll control, connected to PCA9685 channel 1

// --- BNO085 Angle Range Mapping (Adjust as needed) ---
// Define the expected input range of Pitch and Roll from BNO085 in degrees.
// The BNO085 typically outputs values from -90 to +90 for Pitch and -180 to +180 for Roll/Yaw.
// Adjust these if your application has a smaller or larger range of motion.
const float BNO_PITCH_MIN_DEG = -90.0; // BNO085 Pitch min (degrees)
const float BNO_PITCH_MAX_DEG = 90.0;  // BNO085 Pitch max (degrees)

const float BNO_ROLL_MIN_DEG  = -90.0; // BNO085 Roll min (degrees) - often used for horizontal balance
const float BNO_ROLL_MAX_DEG  = 90.0;  // BNO085 Roll max (degrees)

// --- Servo Output Angle Range ---
// This defines the desired output range for the servos based on their ACTUAL usable range.
// Because SG90s often don't do a full 180, we use a slightly narrower mapping
// that will be calibrated by SERVOMIN_CALIBRATED and SERVOMAX_CALIBRATED.
const int SERVO_OUT_MIN_DEG = 0;
const int SERVO_OUT_MAX_DEG = 180; // This is the *conceptual* 0-180 mapping. The actual range is determined by SERVOMIN/MAX_CALIBRATED.


// --- Function to convert desired angle (0-180) to PCA9685 pulse length ---
// This function uses your calibrated SERVOMIN and SERVOMAX values.
int angleToPulse(int angle) {
  // map(value, fromLow, fromHigh, toLow, toHigh)
  // Maps the desired angle (0-180) to the servo's calibrated pulse range (SERVOMIN_CALIBRATED to SERVOMAX_CALIBRATED).
  int pulse = map(angle, SERVO_OUT_MIN_DEG, SERVO_OUT_MAX_DEG, SERVOMIN_CALIBRATED, SERVOMAX_CALIBRATED);
  return pulse;
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  Serial.println("BNO085 IMU to PCA9685 Servo Control");

  // --- Initialize I2C Bus ---
  // Uses default ESP32 I2C pins (GPIO8/9 for S3 or GPIO21/22 for earlier ESP32)
  Wire.begin();

  // --- Initialize BNO085 IMU ---
  Serial.print("Initializing BNO085...");
  if (!bno.begin_I2C()) {
    Serial.println("FAILED! BNO085 not found. Check wiring and address.");
    while (1) delay(10); // Halt if IMU not found
  }
  Serial.println("SUCCESS!");

  // Enable the desired reports from BNO085.
  // SH2_ROTATION_VECTOR is generally the most stable for orientation.
  // We'll then convert it to Euler angles for easier mapping to servos.
  bno.enableRotationVector(50); // Report every 50ms (20Hz)
  // If you prefer raw Euler angles directly (might be less stable for fast motion):
  // bno.enableEulerAngle(50);

  // --- Initialize PCA9685 Servo Driver ---
  Serial.print("Initializing PCA9685...");
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ); // Set PWM frequency for servos (50 Hz is standard)
  Serial.println("SUCCESS!");

  // --- Set Initial Servo Positions (e.g., center) ---
  Serial.println("Setting initial servo positions...");
  pwm.setPWM(pitchServoChannel, 0, angleToPulse(90)); // Center pitch servo
  pwm.setPWM(rollServoChannel, 0, angleToPulse(90));  // Center roll servo
  delay(1000); // Give servos time to move
  Serial.println("Setup complete. Reading IMU data and controlling servos.");
}

void loop() {
  // --- Check for BNO085 Reset ---
  if (bno.wasReset()) {
    Serial.println("BNO085 was reset. Re-enabling reports...");
    bno.enableRotationVector(50); // Re-enable if the sensor resets
  }

  // --- Read BNO085 Data ---
  if (bno.hasNewData()) {
    sh2_SensorValue sensorValue;
    bno.getEvent(&sensorValue);

    float yaw_rad, pitch_rad, roll_rad; // Angles in radians
    float yaw_deg, pitch_deg, roll_deg; // Angles in degrees

    // We enabled ROTATION_VECTOR, so get Euler angles from it
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      bno.getEulerAngles(&yaw_rad, &pitch_rad, &roll_rad); // Angles are in radians

      // Convert radians to degrees for easier understanding and mapping
      yaw_deg   = yaw_rad * 180.0 / PI;
      pitch_deg = pitch_rad * 180.0 / PI;
      roll_deg  = roll_rad * 180.0 / PI;

      // --- Print IMU Data (for debugging) ---
      Serial.print("Yaw: ");   Serial.print(yaw_deg, 1);
      Serial.print(" Pitch: "); Serial.print(pitch_deg, 1);
      Serial.print(" Roll: ");  Serial.println(roll_deg, 1);

      // --- Map BNO085 Angles to Servo Angles ---
      // 1. Pitch Control
      // Map BNO085 pitch range to your servo's conceptual 0-180 degree range.
      // Use 'constrain' to ensure the mapped value stays within the 0-180 bounds.
      int mappedPitchAngle = map(pitch_deg, BNO_PITCH_MIN_DEG, BNO_PITCH_MAX_DEG, SERVO_OUT_MIN_DEG, SERVO_OUT_MAX_DEG);
      mappedPitchAngle = constrain(mappedPitchAngle, SERVO_OUT_MIN_DEG, SERVO_OUT_MAX_DEG);

      // 2. Roll Control
      int mappedRollAngle = map(roll_deg, BNO_ROLL_MIN_DEG, BNO_ROLL_MAX_DEG, SERVO_OUT_MIN_DEG, SERVO_OUT_MAX_DEG);
      mappedRollAngle = constrain(mappedRollAngle, SERVO_OUT_MIN_DEG, SERVO_OUT_MAX_DEG);

      // --- Control Servos ---
      // Convert the conceptual 0-180 angle to PCA9685 pulse ticks using angleToPulse()
      pwm.setPWM(pitchServoChannel, 0, angleToPulse(mappedPitchAngle));
      pwm.setPWM(rollServoChannel, 0, angleToPulse(mappedRollAngle));

      // --- Print Servo Angles (for debugging) ---
      Serial.print("  Mapped Pitch Servo Angle: "); Serial.print(mappedPitchAngle);
      Serial.print("  Mapped Roll Servo Angle: ");  Serial.println(mappedRollAngle);
    }
    // If you explicitly enabled SH2_EULER_ANGLE instead of ROTATION_VECTOR
    else if (sensorValue.sensorId == SH2_EULER_ANGLE) {
      // Data is directly available as Euler angles in radians
      yaw_rad = sensorValue.un.eulerAngle.yaw;
      pitch_rad = sensorValue.un.eulerAngle.pitch;
      roll_rad = sensorValue.un.eulerAngle.roll;

      yaw_deg   = yaw_rad * 180.0 / PI;
      pitch_deg = pitch_rad * 180.0 / PI;
      roll_deg  = roll_rad * 180.0 / PI;

      Serial.print("Yaw: ");   Serial.print(yaw_deg, 1);
      Serial.print(" Pitch: "); Serial.print(pitch_deg, 1);
      Serial.print(" Roll: ");  Serial.println(roll_deg, 1);

      // Mapping and servo control would be identical to the ROTATION_VECTOR section above
      int mappedPitchAngle = map(pitch_deg, BNO_PITCH_MIN_DEG, BNO_PITCH_MAX_DEG, SERVO_OUT_MIN_DEG, SERVO_OUT_MAX_DEG);
      mappedPitchAngle = constrain(mappedPitchAngle, SERVO_OUT_MIN_DEG, SERVO_OUT_MAX_DEG);

      int mappedRollAngle = map(roll_deg, BNO_ROLL_MIN_DEG, BNO_ROLL_MAX_DEG, SERVO_OUT_MIN_DEG, SERVO_OUT_MAX_DEG);
      mappedRollAngle = constrain(mappedRollAngle, SERVO_OUT_MIN_DEG, SERVO_OUT_MAX_DEG);

      pwm.setPWM(pitchServoChannel, 0, angleToPulse(mappedPitchAngle));
      pwm.setPWM(rollServoChannel, 0, angleToPulse(mappedRollAngle));

      Serial.print("  Mapped Pitch Servo Angle: "); Serial.print(mappedPitchAngle);
      Serial.print("  Mapped Roll Servo Angle: ");  Serial.println(mappedRollAngle);
    }
  }

  // Small delay to prevent busy-waiting and allow other tasks to run.
  // Adjust as needed for responsiveness vs. processor load.
  delay(10);
}