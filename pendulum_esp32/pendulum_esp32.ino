#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50
#define SDA_PIN 21
#define SCL_PIN 9

// Servo position limits for pendulum swing
#define PENDULUM_CENTER 375   // Center position
#define PENDULUM_LEFT   250   // Left swing limit
#define PENDULUM_RIGHT  500   // Right swing limit

// Smooth motion variables
float currentPosition = PENDULUM_CENTER;
float targetPosition = PENDULUM_RIGHT;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 20;  // Update every 20ms for smooth motion

// Pendulum state
bool swingingRight = true;
float smoothingFactor = 0.05;  // Lower = smoother, slower motion

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 Elegant Pendulum Motion");
  
  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Start at center position
  pwm.setPWM(0, 0, (int)currentPosition);
  
  Serial.println("Starting elegant pendulum motion...");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update servo position at regular intervals
  if (currentTime - lastUpdate >= updateInterval) {
    lastUpdate = currentTime;
    
    // Smooth motion calculation - natural acceleration/deceleration
    currentPosition += (targetPosition - currentPosition) * smoothingFactor;
    
    // Set servo position
    pwm.setPWM(0, 0, (int)currentPosition);
    
    // Check if we're close to target position
    if (abs(currentPosition - targetPosition) < 2.0) {
      // Switch direction for pendulum effect
      if (swingingRight) {
        targetPosition = PENDULUM_LEFT;
        swingingRight = false;
        Serial.println("Swinging left...");
      } else {
        targetPosition = PENDULUM_RIGHT;
        swingingRight = true;
        Serial.println("Swinging right...");
      }
    }
    
    // Optional: Print current position for debugging
    // Serial.print("Position: ");
    // Serial.println((int)currentPosition);
  }
}
