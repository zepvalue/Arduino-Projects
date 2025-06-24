#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50
#define HEAD_SERVO_CHANNEL 0  // Use channel 0 for head rotation

// Servo PWM range for your setup
#define SERVO_MIN 150   // Your minimum PWM value
#define SERVO_MAX 600   // Your maximum PWM value
#define SERVO_CENTER 375 // Center position

// Animation timing
#define FRAME_RATE 60   // 60 FPS from your animation data
#define FRAME_DELAY_MS (1000 / FRAME_RATE)  // 16.67ms per frame

// Head rotation keyframe data (extracted from your JSON)
struct Keyframe {
  int frame;
  float time;
  float value;  // Rotation angle in degrees
};

// Key head rotation values from your animation
Keyframe headRotationKeyframes[] = {
  {1, 0.0167, 0},
  {62, 1.033, 0.911},
  {73, 1.217, 90},
  {85, 1.417, 180},
  {98, 1.633, 90},
  {111, 1.85, 0},
  {120, 2.0, -0.294},
  {139, 2.317, -36.661},
  {180, 3.0, -36.661},
  {197, 3.283, 0},
  {200, 3.333, 0}
};

int numKeyframes = sizeof(headRotationKeyframes) / sizeof(headRotationKeyframes[0]);
int currentKeyframe = 0;
unsigned long animationStartTime;
unsigned long lastFrameTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("TEDD Head Rotation Animation Control");
  
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(10);
  
  // Start animation
  animationStartTime = millis();
  Serial.println("Starting head rotation animation...");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Calculate animation time in seconds
  float animationTime = (currentTime - animationStartTime) / 1000.0;
  
  // Get current head rotation angle from animation data
  float headAngle = getHeadRotationAtTime(animationTime);
  
  // Convert angle to servo PWM value
  int servoPWM = angleToServoPWM(headAngle);
  
  // Send PWM signal to head servo
  pwm.setPWM(HEAD_SERVO_CHANNEL, 0, servoPWM);
  
  // Print debug info every 100ms
  if (currentTime - lastFrameTime >= 100) {
    lastFrameTime = currentTime;
    Serial.print("Time: ");
    Serial.print(animationTime, 2);
    Serial.print("s | Head Angle: ");
    Serial.print(headAngle, 1);
    Serial.print("° | PWM: ");
    Serial.println(servoPWM);
  }
  
  // Loop animation every 3.33 seconds
  if (animationTime >= 3.33) {
    animationStartTime = millis();
    Serial.println("Restarting animation...");
  }
  
  delay(FRAME_DELAY_MS);  // Maintain 60 FPS timing
}

// Function to get head rotation angle at specific time
float getHeadRotationAtTime(float time) {
  // Handle time beyond animation length
  if (time > 3.33) {
    time = fmod(time, 3.33);  // Loop the animation
  }
  
  // Find the appropriate keyframe pair for interpolation
  for (int i = 0; i < numKeyframes - 1; i++) {
    if (time >= headRotationKeyframes[i].time && time <= headRotationKeyframes[i + 1].time) {
      // Linear interpolation between keyframes
      float t1 = headRotationKeyframes[i].time;
      float t2 = headRotationKeyframes[i + 1].time;
      float v1 = headRotationKeyframes[i].value;
      float v2 = headRotationKeyframes[i + 1].value;
      
      // Interpolation factor
      float factor = (time - t1) / (t2 - t1);
      
      // Interpolated angle
      return v1 + (v2 - v1) * factor;
    }
  }
  
  // Default to last keyframe value
  return headRotationKeyframes[numKeyframes - 1].value;
}

// Convert rotation angle to servo PWM value
int angleToServoPWM(float angle) {
  // Map your animation angles to servo range
  // Your animation: -36.66° to +180°
  // Servo range: 150 to 600 PWM values
  
  // Clamp angle to reasonable servo range
  float clampedAngle = constrain(angle, -45, 180);
  
  // Map angle to PWM range
  // -45° = SERVO_MIN (150), +180° = SERVO_MAX (600)
  int pwmValue = map(clampedAngle * 10, -450, 1800, SERVO_MIN, SERVO_MAX);
  
  // Ensure PWM value is within bounds
  return constrain(pwmValue, SERVO_MIN, SERVO_MAX);
}
