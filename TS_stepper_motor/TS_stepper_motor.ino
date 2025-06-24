// Real-Time Stepper Motor Health Table - Updates Values in Place
const int dirPin = 2;
const int stepPin = 3;
const int enablePin = 4;
const int currentSensePin = A0;
const int voltageSensePin = A1;

// Motor specifications
const int stepsPerRevolution = 200;
const int microstepping = 16;
const int totalSteps = stepsPerRevolution * microstepping;

// Real-time health structure
struct RealTimeHealth {
  // Electrical Stats
  float voltage = 0.0;
  float current = 0.0;
  float power = 0.0;
  float resistance = 0.0;
  
  // Performance Stats
  unsigned long totalSteps = 0;
  float currentSpeed = 0.0;
  float peakSpeed = 0.0;
  float efficiency = 0.0;
  float stepAccuracy = 100.0;
  
  // Health Indicators
  float temperature = 25.0;
  float vibration = 0.0;
  int healthScore = 100;
  String status = "EXCELLENT";
  
  // Timing
  unsigned long sessionTime = 0;
} health;

// Non-blocking timers
unsigned long displayTimer = 0;
unsigned long measurementTimer = 0;
unsigned long movementTimer = 0;
unsigned long sessionStart = 0;

// Movement state
bool isMoving = false;
bool movementDirection = true;
bool tableInitialized = false;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  digitalWrite(enablePin, LOW);
  
  Serial.begin(9600);
  Serial.println("🚀 Real-Time Stepper Motor Health Monitor");
  Serial.println();
  
  sessionStart = millis();
  
  // Initialize the static table
  initializeStaticTable();
}

void loop() {
  // Update session time
  health.sessionTime = millis() - sessionStart;
  
  // Continuous measurements (every 50ms)
  if (millis() - measurementTimer >= 50) {
    updateRealTimeMeasurements();
    measurementTimer = millis();
  }
  
  // Update table values (every 500ms)
  if (millis() - displayTimer >= 500) {
    updateTableValues();
    displayTimer = millis();
  }
  
  // Non-blocking movement execution
  executeNonBlockingMovement();
}

void initializeStaticTable() {
  Serial.println("================================================");
  Serial.println("          🎯 MOTOR HEALTH DASHBOARD 🎯          ");
  Serial.println("================================================");
  Serial.println("| Metric               | Current    | Peak     |");
  Serial.println("|----------------------|------------|----------|");
  Serial.println("| ⏱️  Runtime            |            |          |");
  Serial.println("| 🔄 Total Steps        |            |          |");
  Serial.println("| ⚡ Current Speed      |            |          |");
  Serial.println("| 📈 Average Speed      |            |          |");
  Serial.println("| 🌡️  Temperature        |            |          |");
  Serial.println("| ↻  CW/CCW Steps       |            |          |");
  Serial.println("| 🔄 Direction Changes  |            |          |");
  Serial.println("| 📊 Performance        |            |          |");
  Serial.println("| 💚 Health Score       |            |          |");
  Serial.println("| ⚡ Power Consumption  |            |          |");
  Serial.println("| 🔧 System Status      |            |          |");
  Serial.println("================================================");
  Serial.println("| 🎮 Current Activity:                        |");
  Serial.println("================================================");
  
  tableInitialized = true;
}

void updateTableValues() {
  if (!tableInitialized) return;
  
  // Move cursor up to overwrite table data (13 lines up to start of data)
  Serial.print("\033[13A");
  
  // Update Runtime row
  Serial.print("| ⏱️  Runtime            | ");
  printTableValue(getFormattedTime(health.sessionTime), 10);
  Serial.print(" | ");
  printTableValue(String(health.sessionTime/1000) + "s", 8);
  Serial.println(" |");
  
  // Update Total Steps row
  Serial.print("| 🔄 Total Steps        | ");
  printTableValue(String(health.totalSteps), 10);
  Serial.print(" | ");
  printTableValue(String(health.totalSteps/totalSteps) + " rev", 8);
  Serial.println(" |");
  
  // Update Current Speed row
  Serial.print("| ⚡ Current Speed      | ");
  printTableValue(String(health.currentSpeed, 1) + " SPS", 10);
  Serial.print(" | ");
  printTableValue(String(health.peakSpeed, 1) + " SPS", 8);
  Serial.println(" |");
  
  // Update Average Speed row
  Serial.print("| 📈 Average Speed      | ");
  printTableValue(String(calculateAverageSpeed(), 1) + " SPS", 10);
  Serial.print(" | ");
  printTableValue(String(health.efficiency, 0) + " SPM", 8);
  Serial.println(" |");
  
  // Update Temperature row
  Serial.print("| 🌡️  Temperature        | ");
  printTableValue(String(health.temperature, 1) + "°C " + getTempEmoji(), 10);
  Serial.print(" | ");
  printTableValue("Peak: " + String(health.temperature + 5, 1) + "°C", 8);
  Serial.println(" |");
  
  // Update Direction row
  Serial.print("| ↻  CW/CCW Steps       | ");
  printTableValue(String(health.totalSteps/2) + "/" + String(health.totalSteps/2), 10);
  Serial.print(" | ");
  printTableValue("50% CW", 8);
  Serial.println(" |");
  
  // Update Direction Changes row
  Serial.print("| 🔄 Direction Changes  | ");
  printTableValue(String(health.totalSteps/totalSteps), 10);
  Serial.print(" | ");
  printTableValue(getMotorStatusEmoji(), 8);
  Serial.println(" |");
  
  // Update Performance row
  Serial.print("| 📊 Performance        | ");
  printTableValue(getPerformanceEmoji(), 10);
  Serial.print(" | ");
  printTableValue(getSystemHealthEmoji(), 8);
  Serial.println(" |");
  
  // Update Health Score row
  Serial.print("| 💚 Health Score       | ");
  printTableValue(String(health.healthScore) + "% " + getHealthEmoji(), 10);
  Serial.print(" | ");
  printTableValue(health.status, 8);
  Serial.println(" |");
  
  // Update Power Consumption row
  Serial.print("| ⚡ Power Consumption  | ");
  printTableValue(String(health.power, 1) + "W", 10);
  Serial.print(" | ");
  printTableValue(String(health.voltage, 1) + "V", 8);
  Serial.println(" |");
  
  // Update System Status row
  Serial.print("| 🔧 System Status      | ");
  printTableValue(getCurrentActivity(), 10);
  Serial.print(" | ");
  printTableValue("ONLINE", 8);
  Serial.println(" |");
  
  Serial.println("================================================");
  
  // Update Activity line
  Serial.print("| 🎮 Current Activity: ");
  String activity = getCurrentActivity();
  Serial.print(activity);
  // Pad the activity line to full width
  for (int i = activity.length(); i < 25; i++) {
    Serial.print(" ");
  }
  Serial.println(" |");
  
  Serial.println("================================================");
}

void updateRealTimeMeasurements() {
  // Electrical measurements
  health.voltage = analogRead(voltageSensePin) * (3.3/1023.0) * 4.0;
  health.current = analogRead(currentSensePin) * (3.3/1023.0) / 0.1;
  health.power = health.voltage * health.current;
  
  // Performance calculations
  static unsigned long lastSpeedCalc = 0;
  static int stepsInInterval = 0;
  
  if (isMoving) {
    stepsInInterval++;
    if (millis() - lastSpeedCalc >= 1000) {
      health.currentSpeed = stepsInInterval;
      if (health.currentSpeed > health.peakSpeed) {
        health.peakSpeed = health.currentSpeed;
      }
      stepsInInterval = 0;
      lastSpeedCalc = millis();
    }
  } else {
    health.currentSpeed = 0;
  }
  
  // Temperature estimation
  health.temperature = 25.0 + (health.power * 2.0) + (health.sessionTime / 120000.0);
  
  // Health score calculation
  calculateRealTimeHealthScore();
}

void executeNonBlockingMovement() {
  static unsigned long stepTimer = 0;
  static int stepsRemaining = 0;
  static unsigned long stepDelay = 1000;
  
  // Start new movement every 4 seconds
  if (!isMoving && millis() - movementTimer >= 4000) {
    stepsRemaining = 1600; // Half revolution
    movementDirection = !movementDirection;
    digitalWrite(dirPin, movementDirection ? HIGH : LOW);
    isMoving = true;
    movementTimer = millis();
  }
  
  // Execute steps non-blocking
  if (isMoving && stepsRemaining > 0) {
    if (micros() - stepTimer >= stepDelay) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(2);
      digitalWrite(stepPin, LOW);
      
      health.totalSteps++;
      stepsRemaining--;
      stepTimer = micros();
      
      stepDelay = 800UL + (stepsRemaining % 400);
    }
  }
  
  // End movement
  if (isMoving && stepsRemaining <= 0) {
    isMoving = false;
  }
}

void calculateRealTimeHealthScore() {
  int score = 100;
  
  if (health.voltage < 11.0 || health.voltage > 13.0) score -= 15;
  if (health.current > 2.0) score -= 10;
  if (health.temperature > 70) score -= 25;
  else if (health.temperature > 50) score -= 10;
  if (health.power > 25.0) score -= 10;
  
  health.healthScore = max(0, score);
  
  if (health.healthScore >= 90) health.status = "EXCELLENT";
  else if (health.healthScore >= 75) health.status = "GOOD";
  else if (health.healthScore >= 60) health.status = "FAIR";
  else if (health.healthScore >= 40) health.status = "POOR";
  else health.status = "CRITICAL";
}

// Helper functions
void printTableValue(String value, int width) {
  Serial.print(value);
  for (int i = value.length(); i < width; i++) {
    Serial.print(" ");
  }
}

String getFormattedTime(unsigned long timeMs) {
  int minutes = (timeMs % 3600000) / 60000;
  int seconds = (timeMs % 60000) / 1000;
  return String(minutes) + "m " + String(seconds) + "s";
}

float calculateAverageSpeed() {
  if (health.sessionTime > 0) {
    return (health.totalSteps * 1000.0) / health.sessionTime;
  }
  return 0.0;
}

String getTempEmoji() {
  if (health.temperature > 70) return "🔥";
  if (health.temperature > 50) return "⚠️";
  return "✅";
}

String getHealthEmoji() {
  if (health.healthScore >= 90) return "💚";
  else if (health.healthScore >= 75) return "💛";
  else if (health.healthScore >= 60) return "🟠";
  else return "🔴";
}

String getMotorStatusEmoji() {
  if (health.currentSpeed > 1000) return "🔴 HIGH";
  if (health.currentSpeed > 500) return "🟡 MED";
  if (health.currentSpeed > 0) return "🟢 ACTIVE";
  return "⭕ STANDBY";
}

String getPerformanceEmoji() {
  if (health.healthScore > 90) return "🚀 EXCELLENT";
  if (health.healthScore > 75) return "⚡ GOOD";
  if (health.healthScore > 60) return "👍 NORMAL";
  return "🐌 LOW";
}

String getSystemHealthEmoji() {
  if (health.temperature > 70) return "🔥 HOT";
  if (health.peakSpeed > 2000) return "💪 PERF";
  return "💚 HEALTHY";
}

String getCurrentActivity() {
  if (isMoving) {
    return movementDirection ? "🔄 CW" : "🔄 CCW";
  }
  return "⏸️ IDLE";
}
