#include <Wire.h> // Required for I2C communication. Standard library.
#include <Adafruit_PWMServoDriver.h> // Required for PCA9685. Install via Library Manager.
#include <ArduinoJson.h> // Required for parsing JSON data. Install via Library Manager.

// --- PCA9685 Object Declaration ---
// The default address for the PCA9685 is 0x40. If you changed it (e.g., by soldering A0-A5), update this.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// --- PCA9685 Channel Definitions for Servos ---
// Assign each animatronic joint to a specific channel on the PCA9685 (0-15).
// Based on your confirmation:
const uint8_t OVERALL_ROTATION_CHANNEL = 0; // Connected to 'overallRotation'
const uint8_t WAIST_TILT_CHANNEL = 1;       // Connected to 'waistTilt'
const uint8_t HEAD_TURN_CHANNEL = 15;       // Connected to 'headTurn'

// --- Servo Calibration Constants for PCA9685 ---
// These values define the pulse width range for 0 and 180 degrees.
// You MUST calibrate these for your specific servos.
// Use Adafruit's example sketches for PCA9685 servo calibration or
// experiment with values to find the sweet spot for your servos.
// A typical servo frequency is 50-60Hz.
const int SERVO_FREQ = 60; // Servo refresh rate in Hz.
// Adjusted for typical SG90 microservo ranges (approx. 500us to 2500us pulse width at 60Hz)
const int SERVOMIN = 123;  // Minimum pulse width (0 degrees) out of 4096. (Equivalent to ~500us pulse)
const int SERVOMAX = 614;  // Maximum pulse width (180 degrees) out of 4096. (Equivalent to ~2500us pulse)
// The PCA9685 works with a 12-bit resolution (0-4095).
// Adjust SERVOMIN and SERVOMAX to match your actual servo's full travel from 0 to 180 degrees.

// --- Constants for Mapping Animation Values to Servo Angles (0-180 degrees) ---
// These values are CRITICAL and will likely need to be adjusted based on:
// 1. The actual range of values output by your Maya animation for each attribute.
// 2. The physical limits and desired range of motion of your animatronic's joints.
// 3. The specific travel range of your servo motors.

// overallRotation:
// Example input range from Maya: -180 to 180 degrees.
// Mapping to servo range: 0 to 180 degrees, where 0 input maps to 90 degrees (center).
const float OVERALL_ROTATION_INPUT_MIN = -180.0; // Minimum expected rotation value from Maya
const float OVERALL_ROTATION_INPUT_MAX = 180.0;  // Maximum expected rotation value from Maya

// waistTilt:
// Example input values: 0.0 to 0.852. These are very small.
// Assuming 0.0 is the center/neutral position (e.g., servo at 90 degrees).
// The input value represents a deviation from this center. We need a scale factor
// to convert this small input into a noticeable servo angle deviation.
const float WAIST_TILT_CENTER_ANGLE = 90.0;       // Servo angle when waistTilt input is 0.0
const float WAIST_TILT_SCALE_FACTOR = 10.0;       // How much servo angle changes per unit of waistTilt input.
                                                  // Adjust this: e.g., 0.852 * 10 = 8.52 degrees deviation.
const float WAIST_TILT_MAX_DEVIATION = 30.0;      // Maximum allowed deviation from center (in degrees).
                                                  // Prevents the servo from trying to go beyond 0 or 180.

// headTurn:
// Example input values like 97.016.
// Assuming these values are already in degrees and can be directly mapped to a servo range.
// If your Maya output for these can be negative (e.g., -90 to +90), you'll need to adjust
// DEGREE_INPUT_MIN/MAX and the mapping function to center them appropriately (e.g., -90 -> 0, 0 -> 90, 90 -> 180).
const float DEGREE_INPUT_MIN = 0.0;  // Minimum expected degree input (adjust if your Maya outputs negative angles)
const float DEGREE_INPUT_MAX = 180.0; // Maximum expected degree input (adjust if your Maya outputs larger angles)


// --- Serial Communication Buffer ---
// Define the size of the buffer to store incoming JSON data.
// Ensure this is large enough to hold your largest single JSON frame object.
const int JSON_DOC_SIZE = 512;
char jsonBuffer[JSON_DOC_SIZE];
int jsonBufferIndex = 0;
bool jsonComplete = false; // Flag to indicate a complete JSON object has been received

// --- Custom map function for floats ---
// Arduino's built-in map() function only works with integers (longs).
// This version allows mapping floating-point numbers.
long mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  // Ensure the input value is within the specified input range
  x = constrain(x, in_min, in_max);
  return (long)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// Function to convert an angle (0-180) to a PCA9685 12-bit pulse value (0-4096)
uint16_t setServoAngle(uint8_t channel, int angle) {
  // Map the angle (0-180) to the servo's pulse range (SERVOMIN to SERVOMAX)
  uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulselen);
  return pulselen;
}

void setup() {
    // Initialize serial communication. Baud rate must match the sender (Python script).
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect. (Needed for Teensy boards)

    // Initialize the PCA9685
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ); // Set the PWM frequency.

    // Set initial servo positions for the connected servos
    setServoAngle(OVERALL_ROTATION_CHANNEL, 90);
    setServoAngle(WAIST_TILT_CHANNEL, 90);
    setServoAngle(HEAD_TURN_CHANNEL, 90);

    Serial.println("Teensy with PCA9685 ready to receive animation data.");
    Serial.println("Send JSON objects over Serial Monitor or a Python script.");
}

void loop() {
    // Read incoming serial data character by character
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();

        // Accumulate characters into the buffer
        // We assume each JSON object starts with '{' and ends with '}'.
        // And that each object is sent on a single line or as a self-contained message.
        if (incomingChar == '{' && jsonBufferIndex == 0) {
            // Start of a new JSON object
            jsonBuffer[jsonBufferIndex++] = incomingChar;
        } else if (jsonBufferIndex > 0 && incomingChar == '}') {
            // End of the current JSON object
            jsonBuffer[jsonBufferIndex++] = incomingChar;
            jsonBuffer[jsonBufferIndex] = '\0'; // Null-terminate the string
            jsonComplete = true; // Mark JSON as complete
        } else { // Handle cases like newlines before '{' or buffer overflow
            if (incomingChar != '\n' && incomingChar != '\r') { // Ignore newlines/carriage returns outside JSON
                Serial.print("Warning: Ignoring unexpected character or buffer full: '");
                Serial.print(incomingChar);
                Serial.print("' (index: ");
                Serial.print(jsonBufferIndex);
                Serial.println(")");
            }
            if (jsonBufferIndex >= JSON_DOC_SIZE - 1) {
                Serial.println("Error: JSON buffer overflow. Resetting buffer.");
                jsonBufferIndex = 0; // Reset buffer to prevent further issues
                jsonComplete = false;
            }
        }
    }

    // If a complete JSON object has been received
    if (jsonComplete) {
        Serial.print("Received JSON: ");
        Serial.println(jsonBuffer);

        // Create a StaticJsonDocument to parse the JSON.
        // This is the correct way to declare a fixed-size JSON document for ArduinoJson v6,
        // which resolves the 'not a template' error.
        StaticJsonDocument<JSON_DOC_SIZE> doc;

        // Deserialize the JSON string from the buffer
        DeserializationError error = deserializeJson(doc, jsonBuffer);

        // Check for parsing errors
        if (error) {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.f_str());
        } else {
            // Extract values for the *active* servos from the parsed JSON object.
            // Use the '|' operator to provide a default value if the key is missing.
            float overallRotation = doc["overallRotation"] | 0.0;
            float waistTilt = doc["waistTilt"] | 0.0;
            float headTurn = doc["headTurn"] | 0.0;
            // 'hatTurn', 'eyeDirection', 'jawOpen', 'pupilColor' are present in the JSON but not used in this setup.

            // --- Map values to servo angles (0-180) and set PCA9685 PWM ---
            int targetAngle; // Variable to hold the calculated target angle for servo

            // Overall Rotation Servo Control (Channel 0)
            targetAngle = mapFloat(overallRotation, OVERALL_ROTATION_INPUT_MIN, OVERALL_ROTATION_INPUT_MAX, 0, 180);
            targetAngle = constrain(targetAngle, 0, 180); // Ensure angle is within valid servo range
            setServoAngle(OVERALL_ROTATION_CHANNEL, targetAngle);
            Serial.print("Overall Rotation Servo Angle (Ch 0): "); Serial.println(targetAngle);


            // Waist Tilt Servo Control (Channel 1)
            // Calculate deviation from center based on input and scale factor
            float tiltDeviation = waistTilt * WAIST_TILT_SCALE_FACTOR;
            // Constrain the deviation to prevent excessive movement
            tiltDeviation = constrain(tiltDeviation, -WAIST_TILT_MAX_DEVIATION, WAIST_TILT_MAX_DEVIATION);
            // Apply deviation to the center angle
            targetAngle = (int)(WAIST_TILT_CENTER_ANGLE + tiltDeviation);
            targetAngle = constrain(targetAngle, 0, 180);
            setServoAngle(WAIST_TILT_CHANNEL, targetAngle);
            Serial.print("Waist Tilt Servo Angle (Ch 1): "); Serial.println(targetAngle);


            // Head Turn Servo Control (Channel 15)
            targetAngle = mapFloat(headTurn, DEGREE_INPUT_MIN, DEGREE_INPUT_MAX, 0, 180);
            targetAngle = constrain(targetAngle, 0, 180);
            setServoAngle(HEAD_TURN_CHANNEL, targetAngle);
            Serial.print("Head Turn Servo Angle (Ch 15): "); Serial.println(targetAngle);

            Serial.println("--- Servos Updated ---");
        }

        // Reset buffer for the next incoming JSON object
        jsonBufferIndex = 0;
        jsonComplete = false;
    }
}
