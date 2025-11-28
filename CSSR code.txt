#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BluetoothSerial.h>

// ──────────────────────────────────────────────────────────────
// PINS
// ──────────────────────────────────────────────────────────────

Adafruit_MLX90614 mlx;
BluetoothSerial SerialBT;

// Motors
const int L_IN1 = 26, L_IN2 = 27;
const int R_IN1 = 14, R_IN2 = 4;
const int ENA = 25, ENB = 33;

// Ultrasonic
const int TRIG_F = 13, ECHO_F = 34;

// Alerts
const int BUZZER = 23, LED = 19;

// ──────────────────────────────────────────────────────────────
// CONFIG
// ──────────────────────────────────────────────────────────────

bool AUTO_MODE = true;
const float TEMP_MIN = 35.0, TEMP_MAX = 38.0;
const int OBSTACLE_DIST = 30;
const int SPEED = 180;
const int PWM_FREQ = 20000;

// ──────────────────────────────────────────────────────────────
// GLOBAL STATE
// ──────────────────────────────────────────────────────────────

int motorL = 0, motorR = 0;
float lastTemp = NAN;
long lastDist = -1;
int bodiesFound = 0;
unsigned long lastAlert = 0;

// ──────────────────────────────────────────────────────────────
// BLUETOOTH
// ──────────────────────────────────────────────────────────────

void sendMsg(String msg) {
  Serial.println(msg);
  if (SerialBT.hasClient()) SerialBT.println(msg);
}

char readCmd() {
  if (Serial.available()) return (char)Serial.read();
  if (SerialBT.hasClient() && SerialBT.available()) return (char)SerialBT.read();
  return '\0';
}

// ──────────────────────────────────────────────────────────────
// MOTORS
// ──────────────────────────────────────────────────────────────

void setupMotors() {
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  
  ledcSetup(0, PWM_FREQ, 8);
  ledcAttachPin(ENA, 0);
  ledcSetup(1, PWM_FREQ, 8);
  ledcAttachPin(ENB, 1);
  
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void setMotor(int left, int right) {
  motorL = left;
  motorR = right;
  
  // Left
  if (left > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    ledcWrite(0, left);
  } else if (left < 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    ledcWrite(0, -left);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    ledcWrite(0, 0);
  }
  
  // Right
  if (right > 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    ledcWrite(1, right);
  } else if (right < 0) {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    ledcWrite(1, -right);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);
    ledcWrite(1, 0);
  }
}

void stop() { setMotor(0, 0); }
void forward(int s) { setMotor(s, s); }
void backward(int s) { setMotor(-s, -s); }
void turnLeft(int s) { setMotor(-s, s); }
void turnRight(int s) { setMotor(s, -s); }

// ──────────────────────────────────────────────────────────────
// SENSORS
// ──────────────────────────────────────────────────────────────

float readTemp() {
  float t = mlx.readObjectTempC();
  if (isnan(t) || t < -100 || t > 1000) return NAN;
  return t;
}

long readDist() {
  digitalWrite(TRIG_F, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_F, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_F, LOW);
  
  long dur = pulseIn(ECHO_F, HIGH, 20000);
  if (dur == 0) return -1;
  return (dur * 0.034) / 2;
}

// ──────────────────────────────────────────────────────────────
// ALERTS
// ──────────────────────────────────────────────────────────────

void alert() {
  if (millis() - lastAlert < 5000) return;
  lastAlert = millis();
  bodiesFound++;
  
  sendMsg("");
  sendMsg("╔════════════════════════╗");
  sendMsg("║  🔥 BODY DETECTED! 🔥   ║");
  sendMsg("╚════════════════════════╝");
  sendMsg("Detection #" + String(bodiesFound));
  sendMsg("Temperature: " + String(lastTemp, 1) + "°C");
  sendMsg("Distance: " + String(lastDist) + "cm");
  sendMsg("");
  
  // Buzzer: 5 beeps
  for (int i = 0; i < 5; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(100);
  }
  
  stop();
  delay(2000);
}

// ──────────────────────────────────────────────────────────────
// AUTONOMOUS
// ──────────────────────────────────────────────────────────────

static int phase = 0;
static unsigned long phaseTime = 0;

void autoSearch() {
  unsigned long now = millis();
  
  // Check for obstacles
  if (lastDist > 0 && lastDist < OBSTACLE_DIST) {
    stop();
    delay(200);
    backward(100);
    delay(400);
    stop();
    delay(200);
    turnRight(150);
    delay(500);
    stop();
    phase = 0;
    phaseTime = now;
    return;
  }
  
  // Grid search pattern
  if (phaseTime == 0) phaseTime = now;
  
  unsigned long elapsed = now - phaseTime;
  
  switch (phase) {
    case 0:  // Forward
      if (elapsed < 2500) forward(SPEED);
      else { stop(); phase = 1; phaseTime = now; }
      break;
    
    case 1:  // Turn left
      if (elapsed < 600) turnLeft(150);
      else { stop(); phase = 2; phaseTime = now; }
      break;
    
    case 2:  // Forward
      if (elapsed < 2500) forward(SPEED);
      else { stop(); phase = 3; phaseTime = now; }
      break;
    
    case 3:  // Turn right
      if (elapsed < 600) turnRight(150);
      else { stop(); phase = 0; phaseTime = now; }
      break;
  }
}

// ──────────────────────────────────────────────────────────────
// MANUAL
// ──────────────────────────────────────────────────────────────

void manual(char cmd) {
  if (lastDist > 0 && lastDist < OBSTACLE_DIST) {
    stop();
    return;
  }
  
  switch (cmd) {
    case 'w': case 'W': sendMsg("[CMD] Forward"); forward(SPEED); break;
    case 's': case 'S': sendMsg("[CMD] Back"); backward(SPEED); break;
    case 'a': case 'A': sendMsg("[CMD] Left"); turnLeft(150); break;
    case 'd': case 'D': sendMsg("[CMD] Right"); turnRight(150); break;
    case 'x': case 'X': sendMsg("[CMD] Stop"); stop(); break;
    case 'e': case 'E':
      AUTO_MODE = !AUTO_MODE;
      stop();
      sendMsg(AUTO_MODE ? "[MODE] AUTO" : "[MODE] MANUAL");
      break;
    default: break;
  }
}

// ──────────────────────────────────────────────────────────────
// LOGGING
// ──────────────────────────────────────────────────────────────

unsigned long lastLog = 0;

void logStatus() {
  if (millis() - lastLog < 2000) return;
  lastLog = millis();
  
  String s = "[";
  s += String(millis()/1000);
  s += "s] Temp: ";
  if (isnan(lastTemp)) s += "N/A";
  else s += String(lastTemp, 1);
  s += "C  Dist: " + String(lastDist) + "cm  ";
  s += "Mode: " + String(AUTO_MODE ? "AUTO" : "MANUAL");
  s += "  Found: " + String(bodiesFound);
  
  sendMsg(s);
}

// ──────────────────────────────────────────────────────────────
// SETUP
// ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(50);
  
  SerialBT.begin("CSSR_Robot");
  delay(200);
  
  Wire.begin(21, 22);
  if (!mlx.begin()) {
    sendMsg("[WARN] MLX90614 not found");
  }
  
  pinMode(TRIG_F, OUTPUT);
  digitalWrite(TRIG_F, LOW);
  pinMode(ECHO_F, INPUT);
  
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  setupMotors();
  
  sendMsg("");
  sendMsg("╔════════════════════════╗");
  sendMsg("║  CSSR ROBOT - SIMPLE   ║");
  sendMsg("║  Autonomous + Thermal  ║");
  sendMsg("╚════════════════════════╝");
  sendMsg("Mode: " + String(AUTO_MODE ? "AUTO" : "MANUAL"));
  sendMsg("Commands: W/A/S/D/X/E");
  sendMsg("");
  
  // Startup buzzer
  digitalWrite(BUZZER, HIGH);
  delay(200);
  digitalWrite(BUZZER, LOW);
}

void loop() {
  // Read sensors
  lastTemp = readTemp();
  lastDist = readDist();
  
  // Check for body heat
  if (!isnan(lastTemp) && lastTemp >= TEMP_MIN && lastTemp <= TEMP_MAX) {
    if (lastDist > 0 && lastDist < 300) {
      alert();
    }
  }
  
  // Check commands
  char cmd = readCmd();
  if (cmd != '\0') {
    if (cmd == 'e' || cmd == 'E') {
      AUTO_MODE = !AUTO_MODE;
      stop();
      sendMsg(AUTO_MODE ? "[MODE] AUTO" : "[MODE] MANUAL");
    } else if (!AUTO_MODE) {
      manual(cmd);
    }
  }
  
  // Execute mode
  if (AUTO_MODE) {
    autoSearch();
  }
  
  // Log status
  logStatus();
  
  delay(50);
}