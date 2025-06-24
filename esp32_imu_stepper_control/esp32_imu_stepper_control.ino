#include <AccelStepper.h>        // For stepper motor control
#include <Wire.h>              // For I2C communication (for BNO08x)
#include <Adafruit_BNO08x.h>   // For Adafruit BNO08x library (v1.3.0 or newer recommended)
#include <math.h>              // For atan2, asin, fabs functions (for quaternion to Euler conversion)

// --- TB6600 Stepper Motor Driver Pin Definitions ---
// Connect these to your ESP32 GPIOs and the TB6600's PUL-, DIR-, ENA- pins.
const int STEP_PIN    = 17; // Connects to TB6600 PUL- (Yellow Wire)
const int DIR_PIN     = 16; // Connects to TB6600 DIR- (Green Wire)
const int ENABLE_PIN  = 21; // Connects to TB6600 ENA- (Blue Wire)
const int ZERO_BUTTON_PIN = 33; // Connect a momentary push button between this GPIO and GND.
                                // Pressing it will re-zero the IMU's orientation and stepper's position.

// --- Stepper Motor (Bohong 17HD4063-03M NEMA 17) and TB6600 Settings ---
const int STEPS_PER_REVOLUTION = 200; // Your motor's native steps/360 degrees (1.8 deg/step = 200)
// --- IMPORTANT: MATCH THIS TO YOUR TB6600 DIP SWITCH SETTING (SW1, SW2, SW3)! ---
const int MICROSTEP_RESOLUTION = 16; // Set for 1/16 microstepping on TB6600: SW1=OFF, SW2=OFF, SW3=ON
// Calculated total steps for one full revolution at the chosen microstep resolution
const long STEPS_PER_FULL_REVOLUTION_MICROSTEPPED = (long)STEPS_PER_REVOLUTION * MICROSTEP_RESOLUTION;

// --- IMU-to-Stepper Mapping Parameters ---
// This defines the range of IMU yaw (e.g., 360 degrees) that maps to the stepper's range (360 degrees rotation).
const float IMU_MAPPING_RANGE_DEG = 10.0;
// This is the total number of steps the stepper will move to match the IMU_MAPPING_RANGE_DEG.
const long STEPPER_MAPPING_RANGE_STEPS = STEPS_PER_FULL_REVOLUTION_MICROSTEPPED;
// Deadband: +/- degrees around the zero reference where the motor will NOT move.
// This prevents constant micro-adjustments from IMU noise. Adjust as needed.
const float DEADBAND_DEG = 1.0; 

// --- BNO08x IMU Pin Definition ---
// This is the RESET pin for the BNO08x. Make sure this GPIO matches your wiring!
// Requires Adafruit_BNO08x library v1.3.0 or newer.
#define BNO08X_RESET 14 

// --- BNO08x IMU Object ---
// Initialize BNO08x object with the reset pin.
Adafruit_BNO08x imu(BNO08X_RESET);

// --- Sensor Event Structure ---
// This will hold the raw sensor data received from the BNO08x.
sh2_SensorValue_t sensorValue; 

// --- Global Variables for IMU-Stepper Control ---
// These variables store the IMU's yaw and stepper's position when the "zero" reference was set.
float zeroReferenceYaw = 0.0;          
long zeroReferenceStepperPosition = 0; 

// Variables to store the current Euler angles from the IMU.
float imuYaw = 0.0;   // Z-axis rotation
float imuPitch = 0.0; // Y-axis rotation
float imuRoll = 0.0;  // X-axis rotation

// Create an AccelStepper object for the motor driver.
// AccelStepper(interface, stepPin, dirPin)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Helper Function: getAngularDifference ---
// Calculates the shortest angular difference between two angles (handles 0-360 degree wrap-around).
float getAngularDifference(float angle1, float angle2) {
  float diff = angle2 - angle1;
  // Normalize difference to be within -180 to 180 degrees
  if (diff > 180.0) diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  return diff;
}

// --- Helper Function: quaternionToEuler ---
// Converts raw quaternion data from the IMU into Yaw, Pitch, and Roll angles (in degrees).
// This is necessary because older library versions or specific reports don't directly provide Euler angles.
void quaternionToEuler(float qw, float qx, float qy, float qz, float *yaw, float *pitch, float *roll) {
  // Roll (x-axis rotation)
  float sinr_cosp = 2 * (qw * qx + qy * qz);
  float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  *roll = atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  float sinp = 2 * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1)
    *pitch = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range to prevent NaN
  else
    *pitch = asin(sinp);

  // Yaw (z-axis rotation)
  float siny_cosp = 2 * (qw * qz + qx * qy);
  float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  *yaw = atan2(siny_cosp, cosy_cosp);

  // Convert radians to degrees
  *roll = *roll * 180.0 / M_PI;
  *pitch = *pitch * 180.0 / M_PI;
  *yaw = *yaw * 180.0 / M_PI;
}


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial Monitor to be open
  Serial.println("\n--- IMU-Controlled Stepper Motor System ---");
  Serial.println("Using Adafruit BNO08x Library (v1.3.0+), manual Euler conversion.");
  Serial.println("Motor: Bohong 17HD4063-03M NEMA 17 Hybrid Stepper Motor.");
  Serial.print("Stepper Native Steps/Revolution: "); Serial.println(STEPS_PER_REVOLUTION);
  Serial.print("TB6600 Microstep Resolution: 1/"); Serial.print(MICROSTEP_RESOLUTION);
  Serial.println(" (Set TB6600 DIP switches: SW1=OFF, SW2=OFF, SW3=ON).");
  Serial.print("Calculated Total Steps/Full Revolution: "); Serial.println(STEPS_PER_FULL_REVOLUTION_MICROSTEPPED);
  Serial.println("TB6600 Current Setting (SW4, SW5, SW6): Start at 1.0A (SW4=OFF, SW5=ON, SW6=ON). Adjust to 1.5A if needed (SW4=ON, SW5=OFF, SW6=ON) and monitor motor temperature.");
  Serial.println("Make sure to use a stable 12V-24V DC power supply (at least 3A) for the TB6600, with adequately thick wires (not thin Duponts for main power).");

  // --- Configure ESP32 GPIOs as Outputs ---
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP); // Configure button pin with internal pull-up resistor

  // --- Initialize AccelStepper ---
  // Set max speed and acceleration for smooth stepper operation.
  // You might need to fine-tune these values if the motor is jittery or loses steps.
  stepper.setMaxSpeed(1000.0);    // Maximum speed in steps per second.
  stepper.setAcceleration(500.0); // Acceleration in steps per second squared.
  
  // Initially disable the motor. TB6600 ENA- pin is active HIGH to DISABLE the motor.
  digitalWrite(ENABLE_PIN, HIGH); 
  
  // AccelStepper expects active-HIGH pulses. TB6600 PUL- and DIR- are active-LOW (via optocouplers).
  // Invert STEP and DIR signals. The third parameter (for enable) is 'false' because we control ENABLE_PIN manually.
  stepper.setPinsInverted(true, false, false); 

  // --- Initialize BNO08x IMU ---
  Wire.begin(); // Initialize I2C communication (SDA=GPIO21, SCL=GPIO22 on ESP32 by default)

  // Attempt to initialize the BNO08x IMU.
  if (!imu.begin_I2C()) { 
    Serial.println("ERROR: Failed to find BNO08x chip on I2C! Check wiring (SDA/SCL), power, and I2C address.");
    while (1) {
      delay(100); // Halt program if IMU fails to initialize
    }
  }
  Serial.println("BNO08x Found and initialized!");
  
  // Enable the specific report we need for orientation: Rotation Vector.
  // SH2_ROTATION_VECTOR provides orientation relative to its initial power-up orientation.
  // SH2_GEOMAGNETIC_ROTATION_VECTOR provides absolute heading using the magnetometer.
  Serial.println("Setting desired sensor reports...");
  if (!imu.enableReport(SH2_ROTATION_VECTOR)) { 
    Serial.println("ERROR: Could not enable Rotation Vector report!");
    while(1); // Halt if report cannot be enabled
  } else {
    Serial.println("Rotation Vector Report Enabled.");
  }

  // --- Enable the motor ---
  // Now that everything else is set up, enable the motor. Setting ENA- LOW enables the TB6600.
  digitalWrite(ENABLE_PIN, LOW); 
  Serial.println("Motor Enabled (holding torque ON).");
  delay(500); // Short delay to allow the driver to settle

  // --- Set Initial Zero Reference ---
  // Wait for the first valid IMU data to establish the initial orientation and stepper position reference.
  Serial.print("Waiting for initial IMU data to establish zero reference...");
  unsigned long startTime = millis();
  bool dataReceived = false;
  while (millis() - startTime < 5000) { // Timeout after 5 seconds if no data
    if (imu.getSensorEvent(&sensorValue)) { // Try to get a sensor event
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) { // Check if it's the Rotation Vector report
        // Convert the quaternion data to Euler angles (Yaw, Pitch, Roll)
        quaternionToEuler(sensorValue.un.rotationVector.real,
                          sensorValue.un.rotationVector.i,
                          sensorValue.un.rotationVector.j,
                          sensorValue.un.rotationVector.k,
                          &imuYaw, &imuPitch, &imuRoll);
        dataReceived = true;
        break; // Exit loop once valid data is received
      }
    }
    delay(10);
    Serial.print(".");
  }
  if (!dataReceived) {
    Serial.println("\nERROR: No Rotation Vector data received after 5 seconds during setup. Check IMU connection or operation.");
    while(1); // Halt if initial IMU data is not received, as system cannot function without it.
  }
  Serial.println(" Done!");

  zeroReferenceYaw = imuYaw; // Store the current IMU Yaw as the zero point
  zeroReferenceStepperPosition = stepper.currentPosition(); // Store the current stepper position as its zero point
  Serial.print("Initial Yaw Zero Reference Set: "); Serial.println(zeroReferenceYaw, 2); // Print with 2 decimal places
  Serial.print("Initial Stepper Position Reference Set: "); Serial.println(zeroReferenceStepperPosition);
  Serial.println("\n--- Ready! ---");
  Serial.println("Press the button (GPIO " + String(ZERO_BUTTON_PIN) + ") to re-zero the IMU and stepper position at any time.");
  Serial.println("Move the BNO08x sensor to control the stepper motor!");
}


void loop() {
  // Check if the IMU was reset (e.g., due to a power glitch or software issue).
  // If it reset, re-enable the sensor reports to resume operation.
  if (imu.wasReset()) {
    Serial.println("IMU was reset! Re-enabling reports...");
    if (!imu.enableReport(SH2_ROTATION_VECTOR)) {
      Serial.println("ERROR: Could not re-enable Rotation Vector report after reset!");
    }
  }

  // --- Check for Zeroing Button Press ---
  // If the button connected to ZERO_BUTTON_PIN is pressed (pulling the pin LOW)
  if (digitalRead(ZERO_BUTTON_PIN) == LOW) { 
    if (imu.getSensorEvent(&sensorValue)) { // Try to get the latest IMU event
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) { // If it's a Rotation Vector report
        // Convert to Euler angles and update the zero reference
        quaternionToEuler(sensorValue.un.rotationVector.real,
                          sensorValue.un.rotationVector.i,
                          sensorValue.un.rotationVector.j,
                          sensorValue.un.rotationVector.k,
                          &imuYaw, &imuPitch, &imuRoll);
        zeroReferenceYaw = imuYaw; // Update the IMU yaw zero reference
        zeroReferenceStepperPosition = stepper.currentPosition(); // Update the stepper position zero reference
        Serial.print("--- Re-zeroed! New Yaw Zero Reference: ");
        Serial.println(zeroReferenceYaw, 2);
        Serial.println("--- Stepper Position Reference Reset to: " + String(zeroReferenceStepperPosition));
        delay(500); // Debounce delay for the button to prevent multiple triggers
      } else {
        Serial.println("DBG: Cannot re-zero: No valid Rotation Vector data available for new reference.");
        delay(500); // Add a small delay even if no data, to prevent rapid re-checks
      }
    } else {
      Serial.println("DBG: Cannot re-zero: No new IMU event available to get reference.");
      delay(500); // Add a small delay if no new event
    }
  }

  // --- Read IMU Data & Control Stepper ---
  // Only process if a new IMU sensor event is available.
  if (imu.getSensorEvent(&sensorValue)) { 
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) { // Ensure it's the Rotation Vector report
      // Convert raw quaternion data to Euler angles (Yaw, Pitch, Roll)
      quaternionToEuler(sensorValue.un.rotationVector.real,
                        sensorValue.un.rotationVector.i,
                        sensorValue.un.rotationVector.j,
                        sensorValue.un.rotationVector.k,
                        &imuYaw, &imuPitch, &imuRoll);

      float currentYaw = imuYaw;
      // Calculate the relative yaw difference from the established zero reference.
      float relativeYaw = getAngularDifference(zeroReferenceYaw, currentYaw);

      // --- DEBUGGING PRINTS (Observe these in Serial Monitor for troubleshooting!) ---
      // These prints help you see the raw IMU data and the calculated values.
      Serial.print("IMU Yaw: "); Serial.print(currentYaw, 2);
      Serial.print(" | Ref Yaw: "); Serial.print(zeroReferenceYaw, 2);
      Serial.print(" | Rel Yaw (pre-deadband): "); Serial.print(relativeYaw, 2);

      // Apply deadband: If the relative yaw is within the DEADBAND_DEG range, treat it as zero.
      // This prevents the motor from constantly adjusting due to small IMU fluctuations.
      if (abs(relativeYaw) < DEADBAND_DEG) {
        relativeYaw = 0.0;
        Serial.print(" (Deadband Active)");
      } else if (relativeYaw > 0) { // If positive, subtract the deadband amount
        relativeYaw -= DEADBAND_DEG;
      } else { // If negative, add the deadband amount
        relativeYaw += DEADBAND_DEG;
      }
      Serial.print(" | Rel Yaw (post-deadband): "); Serial.print(relativeYaw, 2);

      // Map the adjusted relative yaw to the corresponding number of stepper steps.
      // We scale by 100 for 'map' function to handle floating-point precision, then scale back.
      long targetRelativeStepperSteps = map(
        (long)(relativeYaw * 100), // Current relative yaw scaled up
        (long)(-IMU_MAPPING_RANGE_DEG / 2.0 * 100), // IMU min range scaled up
        (long)(IMU_MAPPING_RANGE_DEG / 2.0 * 100), // IMU max range scaled up
        (long)(-STEPPER_MAPPING_RANGE_STEPS / 2.0), // Stepper min steps
        (long)(STEPPER_MAPPING_RANGE_STEPS / 2.0)  // Stepper max steps
      );
      targetRelativeStepperSteps /= 100; // Scale back down to actual steps

      // Calculate the final absolute target position for the stepper motor.
      // This is the stepper's position at zero reference plus the calculated relative steps.
      long finalTargetPosition = zeroReferenceStepperPosition + targetRelativeStepperSteps;

      // Command the stepper to move to the new target position.
      // AccelStepper intelligently handles acceleration/deceleration and only moves if needed.
      stepper.moveTo(finalTargetPosition);

      // --- MORE DEBUGGING PRINTS ---
      Serial.print(" | Target Rel Steps: "); Serial.print(targetRelativeStepperSteps);
      Serial.print(" | Final Target Abs Pos: "); Serial.print(finalTargetPosition);
      Serial.print(" | Stepper Current Pos: "); Serial.print(stepper.currentPosition());
      Serial.println();
    }
  }

  // --- Crucial for Stepper Movement! ---
  // This must be called repeatedly and frequently within the loop() to make the motor move.
  // It runs the stepper motor's internal state machine (accelerates, decelerates, steps).
  stepper.run(); 
}