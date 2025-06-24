#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50
#define CHANNEL 1     // Using channel 16 as requested
#define PWM_MIN 800       // Minimum PWM value
#define PWM_MAX 2200      // Maximum PWM value

void setup() {
  Serial.begin(115200);
  Serial.println("PWM Signal 800-2200 on Channel 16");
  
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(10);
  Serial.println("Starting PWM sweep from 800 to 2200...");
}

void loop() {
  // Sweep from 800 to 2200
  Serial.println("Sweeping 800 → 2200");
  for (int pwmValue = PWM_MIN; pwmValue <= PWM_MAX; pwmValue += 10) {
    pwm.setPWM(CHANNEL, 0, pwmValue);
    Serial.print("PWM: ");
    Serial.println(pwmValue);
    delay(50);  // Adjust speed as needed
  }
  
  // Sweep back from 2200 to 800
  Serial.println("Sweeping 2200 → 800");
  for (int pwmValue = PWM_MAX; pwmValue >= PWM_MIN; pwmValue -= 10) {
    pwm.setPWM(CHANNEL, 0, pwmValue);
    Serial.print("PWM: ");
    Serial.println(pwmValue);
    delay(50);  // Adjust speed as needed
  }
  
  delay(1000);  // Pause between cycles
}
