#include <AccelStepper.h> // Include the AccelStepper library

// --- Define TB6600 Control Pins (Connect to ESP32 GPIOs) ---
// These are the '-' pins of the TB6600 inputs (PUL-, DIR-, ENA-)
// connected to your ESP32 GPIOs.
const int STEP_PIN    = 17; // Connected to TB6600 PUL- (Yellow Wire)
const int DIR_PIN     = 16; // Connected to TB6600 DIR- (Green Wire)
const int ENABLE_PIN  = 21; // Connected to TB6600 ENA- (Blue Wire)

// --- Stepper Motor and TB6600 Settings ---
// Define the number of steps per revolution for your specific stepper motor.
// This is typically 200 steps for a 1.8-degree stepper motor (360 / 1.8 = 200).
const int STEPS_PER_REVOLUTION = 200;

// Set the microstep resolution you've configured on your TB6600's DIP switches.
// Examples:
// 1   (Full Step)
// 2   (Half Step)
// 4   (1/4 Step)
// 8   (1/8 Step)
// 16  (1/16 Step)
// 32  (1/32 Step)
const int MICROSTEP_RESOLUTION = 16; // <--- **IMPORTANT: MATCH THIS TO YOUR TB6600 DIP SWITCH SETTING!**

// Calculate the total steps required for one full revolution at your chosen microstep resolution.
const long STEPS_PER_FULL_REVOLUTION_MICROSTEPPED = (long)STEPS_PER_REVOLUTION * MICROSTEP_RESOLUTION;

// Create an instance of the AccelStepper library
// AccelStepper (interface, stepPin, dirPin) for drivers that take STEP/DIR pulses
// We need to customize this for TB6600's active-low enable and active-low pulses.
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  Serial.println("\n--- TB6600 Stepper Motor Test ---");
  Serial.println("Ensure TB6600 DIP switches are set for microstepping and current.");
  Serial.print("Steps per physical revolution (motor): "); Serial.println(STEPS_PER_REVOLUTION);
  Serial.print("Microstep Resolution (on TB6600): 1/"); Serial.println(MICROSTEP_RESOLUTION);
  Serial.print("Total steps per full revolution (microstepped): "); Serial.println(STEPS_PER_FULL_REVOLUTION_MICROSTEPPED);
  Serial.println(" ");

  // --- Configure ESP32 GPIOs as Outputs ---
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // --- Initialize AccelStepper ---
  stepper.setMaxSpeed(1000);    // Set max speed in steps/second. Adjust as needed.
  stepper.setAcceleration(500); // Set acceleration in steps/second^2. Smooths motion.

  // TB6600 ENA- pin is active LOW to enable. So set HIGH to disable initially.
  digitalWrite(ENABLE_PIN, HIGH); // Disable motor initially (free spin, no holding torque)
  Serial.println("Motor is initially DISABLED (free-spinning).");

  // TB6600 PUL- (STEP_PIN) and DIR- (DIR_PIN) are active LOW for optocouplers.
  // AccelStepper sends HIGH/LOW pulses. To make it work with active LOW optocouplers
  // where a LOW on the '-' pin activates the optocoupler, we use:
  stepper.setPinsInverted(true, true, true); // Invert STEP, DIR, and ENABLE logic (if you wire ENABLE through AccelStepper's pins)
                                            // More commonly, ENABLE is controlled separately for TB6600.
                                            // If you control ENABLE_PIN manually, you only need to invert STEP and DIR.
                                            // For this code, we control ENABLE_PIN manually, so we only need:
  stepper.setPinsInverted(true, true, false); // Invert STEP and DIR, but not the 3rd (unused enable pin by AccelStepper)

  // --- Enable the motor ---
  Serial.println("Enabling motor (setting ENA- LOW)...");
  digitalWrite(ENABLE_PIN, LOW); // Set ENA- LOW to ENABLE the motor
  delay(100); // Give driver a moment to enable

  Serial.println("\n--- Test Commands (type into Serial Monitor, 115200 baud, Newline) ---");
  Serial.println(" 'f' : Move motor FORWARD by 1 revolution");
  Serial.println(" 'b' : Move motor BACKWARD by 1 revolution");
  Serial.println(" 's' : Stop motor immediately");
  Serial.println(" 'e' : Enable motor (holding torque)");
  Serial.println(" 'd' : Disable motor (free-spinning)");
  Serial.println(" '+' : Increase speed by 100 steps/sec");
  Serial.println(" '-' : Decrease speed by 100 steps/sec");
  Serial.println(" Current Max Speed: " + String(stepper.maxSpeed()));
}


void loop() {
  // Run the stepper motor's internal state machine
  stepper.run();

  // --- Serial Monitor Command Processing ---
  if (Serial.available()) {
    char command = Serial.read(); // Read the character command
    Serial.print("Received command: '"); Serial.print(command); Serial.println("'");

    switch (command) {
      case 'f': // Move forward 1 revolution
        Serial.println("Moving FORWARD 1 revolution...");
        stepper.move(STEPS_PER_FULL_REVOLUTION_MICROSTEPPED);
        break;
      case 'b': // Move backward 1 revolution
        Serial.println("Moving BACKWARD 1 revolution...");
        stepper.move(-STEPS_PER_FULL_REVOLUTION_MICROSTEPPED);
        break;
      case 's': // Stop immediately
        Serial.println("Stopping motor...");
        stepper.stop(); // Stops current motion smoothly
        stepper.setCurrentPosition(0); // Optional: reset position counter
        break;
      case 'e': // Enable motor
        Serial.println("Enabling motor (holding torque ON)...");
        digitalWrite(ENABLE_PIN, LOW); // ENA- LOW to ENABLE
        break;
      case 'd': // Disable motor
        Serial.println("Disabling motor (free-spinning)...");
        digitalWrite(ENABLE_PIN, HIGH); // ENA- HIGH to DISABLE
        break;
      case '+': // Increase speed
        stepper.setMaxSpeed(stepper.maxSpeed() + 100);
        Serial.print("Increased Max Speed to: "); Serial.println(stepper.maxSpeed());
        break;
      case '-': // Decrease speed
        if (stepper.maxSpeed() > 100) { // Don't go below 100 steps/sec
          stepper.setMaxSpeed(stepper.maxSpeed() - 100);
        } else {
          stepper.setMaxSpeed(100);
        }
        Serial.print("Decreased Max Speed to: "); Serial.println(stepper.maxSpeed());
        break;
      default:
        Serial.println("Unknown command. Use 'f', 'b', 's', 'e', 'd', '+', '-'.");
        break;
    }
  }
}