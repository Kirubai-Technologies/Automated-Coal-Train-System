/*
  ESP32 Automated Coal Transport Train - WiFi REST API Controller
  Integrates with React Mobile Web Dashboard

  Motor Driver: L298N (IN1: GPIO 25, IN2: GPIO 26)
  Sensors: 2x HC-SR04 Ultrasonic Sensors
           - Rear (Mining):   TRIG -> GPIO 33, ECHO -> GPIO 35
           - Front (Unload):  TRIG -> GPIO 32, ECHO -> GPIO 34
  Detection Threshold: <= 5.0 cm
  Unloading Gate: SG90 Servo (GPIO 23)
  Network: Station Mode ("KIRUBAI ILLAM")
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// --- Wi-Fi Configuration ---
const char* ssid = "KIRUBAI ILLAM";
const char* password = "Sjebajas";

WebServer server(80);

// L298N Motor Pins
const int TRAIN_IN1 = 25;
const int TRAIN_IN2 = 26;

// Motor PWM Speed Control Configuration
// Default speed is 100% full speed (255 PWM). Decreasing speed lowers duty cycle.
const int DEFAULT_TRAIN_SPEED = 255;
const int MIN_TRAIN_SPEED = 70; // Minimum PWM to prevent DC motor stalling under load
int trainSpeed = DEFAULT_TRAIN_SPEED;

// HC-SR04 Ultrasonic Sensor Pins
// Rear Sensor (Mining Area)
const int REAR_TRIG_PIN = 33;  // OUTPUT trigger pulse
const int REAR_ECHO_PIN = 35;  // INPUT echo return (GPIO 35 is input-only)

// Front Sensor (Unloading Area)
const int FRONT_TRIG_PIN = 32; // OUTPUT trigger pulse
const int FRONT_ECHO_PIN = 34; // INPUT echo return (GPIO 34 is input-only)

// Ultrasonic Detection Distance Threshold in Centimeters (<= 5 cm considered arrived)
const float DISTANCE_THRESHOLD_CM = 5.0;

// SG90 Servo Pin
const int SERVO_PIN = 23;

// Timing Constants
const unsigned long LOADING_TIME_MS = 10000; // 10-second coal loading at mine
const unsigned long UNLOAD_GATE_DELAY_MS = 3000; // 3-second wait before opening gate
const unsigned long UNLOADING_TIME_MS = 6000;
const unsigned long DIRECTION_DELAY_MS = 2000;
const unsigned long SENSOR_CONFIRM_MS = 80;

// Servo Angles (SG90 Servo: Close at 90°, Open at 60°)
const int GATE_CLOSED_ANGLE = 90;  // Gate closed angle (90°)
const int GATE_OPEN_ANGLE = 60;   // Unload open angle (60°)
int currentServoAngle = GATE_CLOSED_ANGLE;

Servo unloadingServo;

enum TrainMode {
  MODE_AUTO,
  MODE_MANUAL
};

enum TrainState {
  REVERSING_TO_MINE,
  LOADING_COAL,
  WAITING_TO_MOVE_FORWARD,
  MOVING_TO_UNLOADING_AREA,
  WAITING_TO_OPEN_GATE,
  UNLOADING_COAL,
  CLOSING_GATE,
  PROJECT_COMPLETE,
  MANUAL_IDLE,
  MANUAL_FORWARD,
  MANUAL_REVERSE,
  PAUSED
};

TrainMode currentMode = MODE_AUTO;
TrainState currentState = PROJECT_COMPLETE;

// Pause & Resume Memory
TrainState pausedState = PROJECT_COMPLETE;
TrainMode pausedMode = MODE_AUTO;
unsigned long pausedElapsedTime = 0;

unsigned long stateStartTime = 0;
unsigned long sensorStartTime = 0;
bool sensorTiming = false;

// Ultrasonic Telemetry & Readings
float rearDistanceCm = 999.0;
float frontDistanceCm = 999.0;
bool rearDetected = false;
bool frontDetected = false;
unsigned long lastSensorReadTime = 0;

// Real-time Track Telemetry & Position Modeling
float trainPosition = 88.0; // 10% (Mine) to 88% (Unloading)
int coalLevel = 0;           // 0 to 100%
unsigned long lastTelemetryUpdate = 0;

// Function Prototypes
void moveTrainForward();
void moveTrainReverse();
void stopTrain();
void applyTrainSpeed();
void setTrainSpeed(int spd);
void pauseTrain();
void resumeTrain();
float readUltrasonicDistance(int trigPin, int echoPin);
void updateUltrasonicSensors();
bool confirmRearDetection();
bool confirmFrontDetection();
const char* getStateString(TrainState st);
void handleRoot();
void handleGetStatus();
void handleCommand();
void handleSerialInput();
void runAutoStateMachine();
void updateTrainTelemetry();
void setCORS();
void setGateAngle(int angle);
void testGateServo();

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n==================================");
  Serial.println("ESP32 Coal Train Server Initializing");

  // 1. Allocate all 4 PWM timers for ESP32PWM/ESP32Servo FIRST
  // Prevents LEDC timer & channel collision with motor analogWrite()
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // 2. Configure SG90 Servo BEFORE any motor PWM / analogWrite
  unloadingServo.setPeriodHertz(50);
  int servoChannel = unloadingServo.attach(SERVO_PIN, 500, 2400);
  if (servoChannel < 0) {
    Serial.printf("❌ [SERVO] ERROR: Failed to attach SG90 Servo on GPIO %d!\n", SERVO_PIN);
  } else {
    Serial.printf("✅ [SERVO] Attached on GPIO %d (LEDC Channel %d)\n", SERVO_PIN, servoChannel);
  }
  setGateAngle(GATE_CLOSED_ANGLE);

  // Startup calibration & self-test twitch (verifies 5V power, GND and signal)
  delay(300);
  setGateAngle(GATE_OPEN_ANGLE);
  delay(400);
  setGateAngle(GATE_CLOSED_ANGLE);
  Serial.println("[SERVO] Startup self-test sweep complete (Gate at 90°).");

  // 3. Configure HC-SR04 Ultrasonic Sensor Pins
  pinMode(REAR_TRIG_PIN, OUTPUT);
  pinMode(REAR_ECHO_PIN, INPUT);
  digitalWrite(REAR_TRIG_PIN, LOW);

  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  digitalWrite(FRONT_TRIG_PIN, LOW);

  // 4. Configure Motor Pins (use digital LOW initially; does not pre-empt LEDC channels)
  pinMode(TRAIN_IN1, OUTPUT);
  pinMode(TRAIN_IN2, OUTPUT);
  digitalWrite(TRAIN_IN1, LOW);
  digitalWrite(TRAIN_IN2, LOW);

  // Setup Wi-Fi: Station Mode
  Serial.println("\n==================================");
  Serial.println("ESP32 Coal Train Server Initializing");
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin(ssid, password);

  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(">>> Wi-Fi Connected Successfully! <<<");
    Serial.print("Connected to SSID: ");
    Serial.println(ssid);
    Serial.print("ESP32 Assigned IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    // Setup mDNS responder (http://coal-train.local)
    if (MDNS.begin("coal-train")) {
      Serial.println("mDNS responder active: http://coal-train.local");
    }

    Serial.println("----------------------------------");
    Serial.print("Access Dashboard at: http://");
    Serial.println(WiFi.localIP());
    Serial.println("==================================");
  } else {
    Serial.println(">>> Wi-Fi Connection Timeout (10s)! <<<");
    Serial.println("Train will operate via Serial commands. Check SSID/Password or router.");
    Serial.println("==================================");
  }

  // Built-in Web Dashboard on root path
  server.on("/", HTTP_GET, handleRoot);

  // Configure REST API Routes with CORS
  server.on("/api/status", HTTP_GET, handleGetStatus);
  server.on("/api/cmd", HTTP_POST, handleCommand);
  server.on("/api/cmd", HTTP_GET, handleCommand);

  // Pre-flight CORS handler
  server.on("/api/status", HTTP_OPTIONS, []() {
    setCORS();
    server.send(204);
  });
  server.on("/api/cmd", HTTP_OPTIONS, []() {
    setCORS();
    server.send(204);
  });

  // Catch-all 404 handler redirecting to dashboard root
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {
      setCORS();
      server.send(204);
      return;
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting to Dashboard...");
  });

  server.begin();
  Serial.println("HTTP Web Dashboard & REST server ready.");
}

void loop() {
  server.handleClient();
  handleSerialInput();

  updateUltrasonicSensors();

  if (currentMode == MODE_AUTO) {
    runAutoStateMachine();
  }

  updateTrainTelemetry();

  delay(15);
}

// ----------------------------------------------------
// Serial Monitor Keyboard Input Handler
// ----------------------------------------------------
void handleSerialInput() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') return;

    switch (toupper(c)) {
      case ' ': // Spacebar = Pause/Resume toggle
      case 'P': // 'P' = Pause/Resume toggle
        if (currentState == PAUSED) {
          resumeTrain();
        } else {
          pauseTrain();
        }
        break;
      case 'D':
        Serial.printf("[SENSORS] Rear US (T33/E35): %.1f cm (%s) | Front US (T32/E34): %.1f cm (%s) | Gate: %d° | State: %s\n",
          rearDistanceCm, rearDetected ? "DETECTED" : "CLEAR",
          frontDistanceCm, frontDetected ? "DETECTED" : "CLEAR",
          currentServoAngle, getStateString(currentState));
        break;
      case 'F':
        currentMode = MODE_MANUAL;
        currentState = MANUAL_FORWARD;
        moveTrainForward();
        Serial.println("[SERIAL] Move FORWARD");
        break;
      case 'R':
        currentMode = MODE_MANUAL;
        currentState = MANUAL_REVERSE;
        moveTrainReverse();
        Serial.println("[SERIAL] Move REVERSE");
        break;
      case 'S':
        stopTrain();
        if (currentMode == MODE_AUTO) currentState = PROJECT_COMPLETE;
        else currentState = MANUAL_IDLE;
        Serial.println("[SERIAL] Train STOPPED");
        break;
      case 'O':
        setGateAngle(GATE_OPEN_ANGLE);
        Serial.println("[SERIAL] Gate OPEN (60°)");
        break;
      case 'C':
        setGateAngle(GATE_CLOSED_ANGLE);
        Serial.println("[SERIAL] Gate CLOSE (90°)");
        break;
      case 'T':
        testGateServo();
        break;
      case 'A':
        currentMode = MODE_AUTO;
        currentState = REVERSING_TO_MINE;
        sensorTiming = false;
        Serial.println("[SERIAL] Started AUTO CYCLE");
        break;
      case 'L':
        currentMode = MODE_AUTO;
        stopTrain();
        stateStartTime = millis();
        currentState = LOADING_COAL;
        sensorTiming = false;
        Serial.println("[SERIAL] LOAD COAL & RUN DEFAULT triggered");
        break;
      case '-': // Decrease speed
        trainSpeed = max(MIN_TRAIN_SPEED, trainSpeed - 25);
        applyTrainSpeed();
        Serial.printf("[SPEED] Decreased to %d (%d%%)\n", trainSpeed, (int)round(trainSpeed * 100.0f / 255.0f));
        break;
      case '+': // Increase speed
      case '=':
        trainSpeed = min(255, trainSpeed + 25);
        applyTrainSpeed();
        Serial.printf("[SPEED] Increased to %d (%d%%)\n", trainSpeed, (int)round(trainSpeed * 100.0f / 255.0f));
        break;
      case '1':
        setTrainSpeed(100);
        Serial.println("[SPEED] Set to 40% (100)");
        break;
      case '2':
        setTrainSpeed(150);
        Serial.println("[SPEED] Set to 60% (150)");
        break;
      case '3':
        setTrainSpeed(200);
        Serial.println("[SPEED] Set to 80% (200)");
        break;
      case '4':
      case '0':
        setTrainSpeed(255);
        Serial.println("[SPEED] Reset to DEFAULT (100% / 255)");
        break;
      case 'H':
      case '?':
        Serial.println("\n--- Commands: F=Fwd, R=Rev, S=Stop, P/Space=Pause, -/+=Speed, 1-4=Speed Presets, O/C=Gate, T=Test Servo, A=Auto, L=Load, D=Diag ---");
        break;
      default:
        break;
    }
  }
}

// ----------------------------------------------------
// Autonomous State Machine
// ----------------------------------------------------
void runAutoStateMachine() {
  switch (currentState) {
    case PAUSED:
      stopTrain();
      break;

    case REVERSING_TO_MINE:
      moveTrainReverse();
      if (confirmRearDetection()) {
        stopTrain();
        Serial.printf("[AUTO] Mining Area reached (Rear US: %.1f cm <= %.1f cm) -> Loading Coal (10s)\n",
          rearDistanceCm, DISTANCE_THRESHOLD_CM);
        stateStartTime = millis();
        currentState = LOADING_COAL;
      }
      break;

    case LOADING_COAL:
      stopTrain();
      if (millis() - stateStartTime >= LOADING_TIME_MS) {
        Serial.println("[AUTO] Coal loaded -> Pausing before direction change");
        stateStartTime = millis();
        currentState = WAITING_TO_MOVE_FORWARD;
      }
      break;

    case WAITING_TO_MOVE_FORWARD:
      stopTrain();
      if (millis() - stateStartTime >= DIRECTION_DELAY_MS) {
        Serial.println("[AUTO] Moving forward to Unloading Area");
        sensorTiming = false;
        currentState = MOVING_TO_UNLOADING_AREA;
      }
      break;

    case MOVING_TO_UNLOADING_AREA:
      moveTrainForward();
      if (confirmFrontDetection()) {
        stopTrain();
        Serial.printf("[AUTO] Unloading Area reached (Front US: %.1f cm <= %.1f cm) -> Waiting 3 seconds before opening gate\n",
          frontDistanceCm, DISTANCE_THRESHOLD_CM);
        stateStartTime = millis();
        currentState = WAITING_TO_OPEN_GATE;
      }
      break;

    case WAITING_TO_OPEN_GATE:
      stopTrain();
      if (millis() - stateStartTime >= UNLOAD_GATE_DELAY_MS) {
        Serial.println("[AUTO] 3s wait complete -> Opening Gate (60°)");
        setGateAngle(GATE_OPEN_ANGLE);
        stateStartTime = millis();
        currentState = UNLOADING_COAL;
      }
      break;

    case UNLOADING_COAL:
      stopTrain();
      if (millis() - stateStartTime >= UNLOADING_TIME_MS) {
        Serial.println("[AUTO] Coal unloaded -> Closing Gate (90°)");
        setGateAngle(GATE_CLOSED_ANGLE);
        stateStartTime = millis();
        currentState = CLOSING_GATE;
      }
      break;

    case CLOSING_GATE:
      stopTrain();
      if (millis() - stateStartTime >= 1000) {
        Serial.println("[AUTO] Project Cycle Completed!");
        currentState = PROJECT_COMPLETE;
      }
      break;

    case PROJECT_COMPLETE:
      stopTrain();
      break;

    default:
      break;
  }
}

// ----------------------------------------------------
// Gate Servo Controls (SG90)
// ----------------------------------------------------
void setGateAngle(int angle) {
  currentServoAngle = constrain(angle, 0, 180);
  if (!unloadingServo.attached()) {
    unloadingServo.setPeriodHertz(50);
    unloadingServo.attach(SERVO_PIN, 500, 2400);
  }
  unloadingServo.write(currentServoAngle);
  Serial.printf("[SERVO] Gate Angle -> %d°\n", currentServoAngle);
}

void testGateServo() {
  Serial.println("[SERVO] Starting Diagnostic Sweep (0° -> 60° -> 90° -> 120° -> 90°)...");
  setGateAngle(0);
  delay(400);
  setGateAngle(60);
  delay(400);
  setGateAngle(90);
  delay(400);
  setGateAngle(120);
  delay(400);
  setGateAngle(GATE_CLOSED_ANGLE);
  Serial.println("[SERVO] Diagnostic Sweep Complete.");
}

// ----------------------------------------------------
// Motor Controls (L298N with PWM Speed Modulation)
// ----------------------------------------------------
void moveTrainForward() {
  analogWrite(TRAIN_IN1, trainSpeed);
  analogWrite(TRAIN_IN2, 0);
}

void moveTrainReverse() {
  analogWrite(TRAIN_IN1, 0);
  analogWrite(TRAIN_IN2, trainSpeed);
}

void stopTrain() {
  analogWrite(TRAIN_IN1, 0);
  analogWrite(TRAIN_IN2, 0);
}

void applyTrainSpeed() {
  if (currentState == MOVING_TO_UNLOADING_AREA || currentState == MANUAL_FORWARD) {
    moveTrainForward();
  } else if (currentState == REVERSING_TO_MINE || currentState == MANUAL_REVERSE) {
    moveTrainReverse();
  }
}

void setTrainSpeed(int spd) {
  trainSpeed = constrain(spd, MIN_TRAIN_SPEED, 255);
  applyTrainSpeed();
}

// ----------------------------------------------------
// Pause & Resume Control System
// ----------------------------------------------------
void pauseTrain() {
  if (currentState == PAUSED) {
    Serial.println("[TRAIN] Train is already PAUSED.");
    return;
  }
  if (currentState == PROJECT_COMPLETE || currentState == MANUAL_IDLE) {
    Serial.println("[TRAIN] Pause ignored: Train is idle / not running.");
    return;
  }

  pausedState = currentState;
  pausedMode = currentMode;
  pausedElapsedTime = millis() - stateStartTime;
  currentState = PAUSED;
  stopTrain();
  sensorTiming = false;

  Serial.print("[TRAIN] Train PAUSED at state: ");
  Serial.print(getStateString(pausedState));
  Serial.print(" | Elapsed in state: ");
  Serial.print(pausedElapsedTime);
  Serial.println(" ms");
}

void resumeTrain() {
  if (currentState != PAUSED) {
    Serial.println("[TRAIN] Resume ignored: Train is not in PAUSED state.");
    return;
  }

  currentState = pausedState;
  currentMode = pausedMode;
  stateStartTime = millis() - pausedElapsedTime;
  sensorTiming = false;

  if (currentState == REVERSING_TO_MINE || currentState == MANUAL_REVERSE) {
    moveTrainReverse();
  } else if (currentState == MOVING_TO_UNLOADING_AREA || currentState == MANUAL_FORWARD) {
    moveTrainForward();
  } else {
    stopTrain();
  }

  Serial.print("[TRAIN] Train RESUMED to state: ");
  Serial.println(getStateString(currentState));
}

// ----------------------------------------------------
// Ultrasonic Distance Measurement & Detection
// ----------------------------------------------------
float readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure pulse duration (timeout: 18000 us ≈ 300 cm max range)
  unsigned long duration = pulseIn(echoPin, HIGH, 18000);
  if (duration == 0) {
    return 999.0f; // Timeout or obstacle out of range
  }
  float dist = (duration * 0.0343f) / 2.0f;
  if (dist < 0.2f || dist > 400.0f) {
    return 999.0f;
  }
  return dist;
}

void updateUltrasonicSensors() {
  unsigned long now = millis();
  if (now - lastSensorReadTime < 40) return; // 40ms interval (~25Hz)
  lastSensorReadTime = now;

  // Prioritize active movement direction for instant stop response
  if (currentState == REVERSING_TO_MINE) {
    rearDistanceCm = readUltrasonicDistance(REAR_TRIG_PIN, REAR_ECHO_PIN);
    rearDetected = (rearDistanceCm > 0.5f && rearDistanceCm <= DISTANCE_THRESHOLD_CM);

    static unsigned long lastOtherPing = 0;
    if (now - lastOtherPing >= 200) {
      lastOtherPing = now;
      frontDistanceCm = readUltrasonicDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
      frontDetected = (frontDistanceCm > 0.5f && frontDistanceCm <= DISTANCE_THRESHOLD_CM);
    }
  } else if (currentState == MOVING_TO_UNLOADING_AREA) {
    frontDistanceCm = readUltrasonicDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    frontDetected = (frontDistanceCm > 0.5f && frontDistanceCm <= DISTANCE_THRESHOLD_CM);

    static unsigned long lastOtherPing = 0;
    if (now - lastOtherPing >= 200) {
      lastOtherPing = now;
      rearDistanceCm = readUltrasonicDistance(REAR_TRIG_PIN, REAR_ECHO_PIN);
      rearDetected = (rearDistanceCm > 0.5f && rearDistanceCm <= DISTANCE_THRESHOLD_CM);
    }
  } else {
    // When idle, loading, unloading, or manual: alternate readings
    static bool alternate = false;
    alternate = !alternate;
    if (alternate) {
      rearDistanceCm = readUltrasonicDistance(REAR_TRIG_PIN, REAR_ECHO_PIN);
      rearDetected = (rearDistanceCm > 0.5f && rearDistanceCm <= DISTANCE_THRESHOLD_CM);
    } else {
      frontDistanceCm = readUltrasonicDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
      frontDetected = (frontDistanceCm > 0.5f && frontDistanceCm <= DISTANCE_THRESHOLD_CM);
    }
  }
}

bool confirmRearDetection() {
  if (rearDetected) {
    if (!sensorTiming) {
      sensorTiming = true;
      sensorStartTime = millis();
    }
    if (millis() - sensorStartTime >= SENSOR_CONFIRM_MS) {
      sensorTiming = false;
      return true;
    }
  } else {
    sensorTiming = false;
  }
  return false;
}

bool confirmFrontDetection() {
  if (frontDetected) {
    if (!sensorTiming) {
      sensorTiming = true;
      sensorStartTime = millis();
    }
    if (millis() - sensorStartTime >= SENSOR_CONFIRM_MS) {
      sensorTiming = false;
      return true;
    }
  } else {
    sensorTiming = false;
  }
  return false;
}

// ----------------------------------------------------
// REST API Handlers for React Dashboard
// ----------------------------------------------------
void setCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

const char* getStateString(TrainState st) {
  switch (st) {
    case REVERSING_TO_MINE: return "REVERSING_TO_MINE";
    case LOADING_COAL: return "LOADING_COAL";
    case WAITING_TO_MOVE_FORWARD: return "WAITING_TO_MOVE_FORWARD";
    case MOVING_TO_UNLOADING_AREA: return "MOVING_TO_UNLOADING_AREA";
    case WAITING_TO_OPEN_GATE: return "WAITING_TO_OPEN_GATE";
    case UNLOADING_COAL: return "UNLOADING_COAL";
    case CLOSING_GATE: return "CLOSING_GATE";
    case PROJECT_COMPLETE: return "PROJECT_COMPLETE";
    case MANUAL_FORWARD: return "MANUAL_FORWARD";
    case MANUAL_REVERSE: return "MANUAL_REVERSE";
    case MANUAL_IDLE: return "MANUAL_IDLE";
    case PAUSED: return "PAUSED";
    default: return "UNKNOWN";
  }
}

void handleGetStatus() {
  setCORS();

  StaticJsonDocument<896> doc;
  doc["mode"] = (currentMode == MODE_AUTO) ? "AUTO" : "MANUAL";
  doc["state"] = getStateString(currentState);
  doc["isPaused"] = (currentState == PAUSED);
  doc["pausedState"] = getStateString(pausedState);
  doc["rearIr"] = rearDetected ? 1 : 0;
  doc["frontIr"] = frontDetected ? 1 : 0;
  doc["rearIrRaw"] = rearDetected ? 0 : 1;
  doc["frontIrRaw"] = frontDetected ? 0 : 1;
  doc["rearDist"] = (rearDistanceCm < 900.0f) ? round(rearDistanceCm * 10) / 10.0 : -1;
  doc["frontDist"] = (frontDistanceCm < 900.0f) ? round(frontDistanceCm * 10) / 10.0 : -1;
  doc["threshold"] = DISTANCE_THRESHOLD_CM;
  doc["servoAngle"] = currentServoAngle;
  doc["speed"] = trainSpeed;
  doc["speedPercent"] = (int)round((trainSpeed * 100.0f) / 255.0f);
  doc["defaultSpeed"] = DEFAULT_TRAIN_SPEED;
  doc["minSpeed"] = MIN_TRAIN_SPEED;
  doc["in1"] = (currentState == MOVING_TO_UNLOADING_AREA || currentState == MANUAL_FORWARD) ? 1 : 0;
  doc["in2"] = (currentState == REVERSING_TO_MINE || currentState == MANUAL_REVERSE) ? 1 : 0;

  doc["trainPosition"] = round(trainPosition * 10) / 10.0;
  doc["coalLevel"] = coalLevel;

  bool isConnected = (WiFi.status() == WL_CONNECTED);
  doc["wifiConnected"] = isConnected;
  doc["ip"] = isConnected ? WiFi.localIP().toString() : "0.0.0.0";
  doc["ssid"] = ssid;
  doc["rssi"] = isConnected ? WiFi.RSSI() : 0;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleCommand() {
  setCORS();

  if (!server.hasArg("action")) {
    server.send(400, "application/json", "{\"error\":\"Missing action\"}");
    return;
  }

  String action = server.arg("action");
  Serial.print("[API CMD] Received: ");
  Serial.println(action);

  if (action == "start_auto") {
    currentMode = MODE_AUTO;
    currentState = REVERSING_TO_MINE;
    sensorTiming = false;
  } else if (action == "pause") {
    pauseTrain();
  } else if (action == "resume") {
    resumeTrain();
  } else if (action == "pause_toggle") {
    if (currentState == PAUSED) {
      resumeTrain();
    } else {
      pauseTrain();
    }
  } else if (action == "load_and_run") {
    currentMode = MODE_AUTO;
    stopTrain();
    stateStartTime = millis();
    currentState = LOADING_COAL;
    sensorTiming = false;
    Serial.println("[API CMD] Coal Load & Run Default triggered.");
  } else if (action == "stop" || action == "estop") {
    stopTrain();
    if (currentMode == MODE_AUTO) {
      currentState = PROJECT_COMPLETE;
    } else {
      currentState = MANUAL_IDLE;
    }
  } else if (action == "forward") {
    currentMode = MODE_MANUAL;
    currentState = MANUAL_FORWARD;
    moveTrainForward();
  } else if (action == "reverse") {
    currentMode = MODE_MANUAL;
    currentState = MANUAL_REVERSE;
    moveTrainReverse();
  } else if (action == "gate_open") {
    setGateAngle(GATE_OPEN_ANGLE);
  } else if (action == "gate_close") {
    setGateAngle(GATE_CLOSED_ANGLE);
  } else if (action == "gate_test") {
    testGateServo();
  } else if (action == "gate_angle" && server.hasArg("angle")) {
    int angle = server.arg("angle").toInt();
    setGateAngle(angle);
  } else if (action == "set_speed") {
    if (server.hasArg("speed")) {
      int spd = server.arg("speed").toInt();
      setTrainSpeed(spd);
      Serial.printf("[API CMD] Set speed to %d (%d%%)\n", trainSpeed, (int)round(trainSpeed * 100.0f / 255.0f));
    } else if (server.hasArg("percent")) {
      int pct = constrain(server.arg("percent").toInt(), 0, 100);
      setTrainSpeed((int)round((pct * 255.0f) / 100.0f));
      Serial.printf("[API CMD] Set speed to %d%% (%d)\n", pct, trainSpeed);
    }
  } else if (action == "speed_decrease") {
    int step = server.hasArg("step") ? server.arg("step").toInt() : 25;
    trainSpeed = max(MIN_TRAIN_SPEED, trainSpeed - step);
    applyTrainSpeed();
    Serial.printf("[API CMD] Decreased speed to %d (%d%%)\n", trainSpeed, (int)round(trainSpeed * 100.0f / 255.0f));
  } else if (action == "speed_increase") {
    int step = server.hasArg("step") ? server.arg("step").toInt() : 25;
    trainSpeed = min(255, trainSpeed + step);
    applyTrainSpeed();
    Serial.printf("[API CMD] Increased speed to %d (%d%%)\n", trainSpeed, (int)round(trainSpeed * 100.0f / 255.0f));
  } else if (action == "speed_reset") {
    trainSpeed = DEFAULT_TRAIN_SPEED;
    applyTrainSpeed();
    Serial.println("[API CMD] Speed reset to Default (100% / 255)");
  }

  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ----------------------------------------------------
// Real-time Track Position & Coal Level Telemetry
// ----------------------------------------------------
void updateTrainTelemetry() {
  unsigned long now = millis();
  float dt = (now - lastTelemetryUpdate) / 1000.0f;
  if (dt <= 0 || dt > 1.0f) dt = 0.05f;
  lastTelemetryUpdate = now;

  // While paused, freeze train position and coal level exactly where they are
  if (currentState == PAUSED) {
    return;
  }

  // Scale track animation delta dynamically by motor speed
  float speedRatio = (float)trainSpeed / 255.0f;
  if (speedRatio < 0.25f) speedRatio = 0.25f;

  switch (currentState) {
    case REVERSING_TO_MINE:
      trainPosition = max(10.0f, trainPosition - (14.0f * speedRatio * dt));
      break;

    case LOADING_COAL:
      trainPosition = 10.0f;
      coalLevel = constrain((int)((now - stateStartTime) * 100 / LOADING_TIME_MS), 0, 100);
      break;

    case WAITING_TO_MOVE_FORWARD:
      trainPosition = 10.0f;
      coalLevel = 100;
      break;

    case MOVING_TO_UNLOADING_AREA:
      trainPosition = min(88.0f, trainPosition + (14.0f * speedRatio * dt));
      break;

    case WAITING_TO_OPEN_GATE:
      trainPosition = 88.0f;
      coalLevel = 100;
      break;

    case UNLOADING_COAL:
      trainPosition = 88.0f;
      coalLevel = constrain(100 - (int)((now - stateStartTime) * 100 / UNLOADING_TIME_MS), 0, 100);
      break;

    case CLOSING_GATE:
    case PROJECT_COMPLETE:
      trainPosition = 88.0f;
      coalLevel = 0;
      break;

    case MANUAL_FORWARD:
      trainPosition = min(88.0f, trainPosition + (16.0f * speedRatio * dt));
      break;

    case MANUAL_REVERSE:
      trainPosition = max(10.0f, trainPosition - (16.0f * speedRatio * dt));
      break;

    default:
      break;
  }

  // Physical hardware ultrasonic sensor confirmations (only snap when in relevant zones)
  if (rearDetected && (currentState == REVERSING_TO_MINE || currentState == LOADING_COAL)) {
    trainPosition = 10.0f;
  }
  if (frontDetected && (currentState == MOVING_TO_UNLOADING_AREA || currentState == WAITING_TO_OPEN_GATE || currentState == UNLOADING_COAL)) {
    trainPosition = 88.0f;
  }
}

// ----------------------------------------------------
// Embedded Animated Railway Dashboard (HTML / CSS / JS)
// ----------------------------------------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Coal Train Controller</title>
<style>
  :root {
    --bg-primary: #080b11;
    --bg-card: rgba(18, 24, 38, 0.88);
    --border-subtle: rgba(255, 255, 255, 0.08);
    --border-bright: rgba(255, 255, 255, 0.2);
    --text-primary: #f8fafc;
    --text-secondary: #94a3b8;
    --text-muted: #64748b;
    --amber-primary: #f59e0b;
    --amber-hover: #d97706;
    --cyan-primary: #06b6d4;
    --emerald-primary: #10b981;
    --red-primary: #ef4444;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; }
  body { background: var(--bg-primary); color: var(--text-primary); padding: 1rem; min-height: 100vh; }
  .dashboard-container { max-width: 760px; margin: 0 auto; display: flex; flex-direction: column; gap: 1.25rem; }

  /* Header */
  .dashboard-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    background: var(--bg-card);
    border: 1px solid var(--border-subtle);
    border-radius: 14px;
    padding: 1rem 1.25rem;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.4);
  }
  .brand-group { display: flex; align-items: center; gap: 0.75rem; }
  .brand-icon {
    width: 42px; height: 42px; border-radius: 10px;
    background: linear-gradient(135deg, rgba(245, 158, 11, 0.2), rgba(217, 119, 6, 0.1));
    border: 1px solid rgba(245, 158, 11, 0.35);
    display: flex; align-items: center; justify-content: center; font-size: 1.35rem;
  }
  .brand-title { font-size: 1.05rem; font-weight: 800; letter-spacing: 0.05em; color: #fff; }
  .brand-subtitle { font-size: 0.72rem; color: var(--text-secondary); }
  .header-badges { display: flex; gap: 0.5rem; align-items: center; }
  .status-pill {
    display: inline-flex; align-items: center; gap: 0.45rem;
    padding: 0.35rem 0.75rem; border-radius: 999px;
    font-size: 0.72rem; font-weight: 700; font-family: monospace;
    background: rgba(239, 68, 68, 0.15); color: #f87171; border: 1px solid rgba(239, 68, 68, 0.3);
  }
  .status-pill.online {
    background: rgba(16, 185, 129, 0.15); color: #34d399; border-color: rgba(16, 185, 129, 0.4);
  }
  .status-dot { width: 7px; height: 7px; border-radius: 50%; background: currentColor; }

  /* Track Visualization Canvas */
  .track-canvas-card {
    background: var(--bg-card);
    border: 1px solid var(--border-subtle);
    border-radius: 14px;
    padding: 1.25rem;
    box-shadow: 0 6px 24px rgba(0, 0, 0, 0.5);
    position: relative;
    overflow: hidden;
  }
  .card-header-bar {
    display: flex; justify-content: space-between; align-items: center; margin-bottom: 1rem;
  }
  .card-title {
    font-size: 0.85rem; font-weight: 700; text-transform: uppercase; letter-spacing: 0.06em;
    color: var(--text-secondary); display: flex; align-items: center; gap: 0.5rem;
  }
  .pos-badge {
    font-family: monospace; font-size: 0.75rem; font-weight: 700;
    padding: 0.2rem 0.65rem; border-radius: 999px;
    background: rgba(0, 0, 0, 0.5); color: #38bdf8; border: 1px solid rgba(56, 189, 248, 0.3);
  }

  /* Track Stage */
  .track-stage {
    position: relative;
    background: linear-gradient(180deg, #090d16 0%, #0f172a 100%);
    border: 1px solid rgba(255, 255, 255, 0.07);
    border-radius: 12px;
    padding: 2.2rem 1.2rem 1.6rem 1.2rem;
    overflow: hidden;
  }
  .track-waypoints {
    display: flex; justify-content: space-between; position: relative; z-index: 2; margin-bottom: 2.8rem;
  }
  .station-node {
    display: flex; flex-direction: column; align-items: center; gap: 0.35rem;
    padding: 0.55rem 0.9rem; background: rgba(18, 24, 38, 0.85);
    border: 1px solid var(--border-subtle); border-radius: 8px; min-width: 140px;
    box-shadow: 0 4px 12px rgba(0,0,0,0.4); transition: all 0.3s ease;
  }
  .station-node.mining { border-left: 3px solid var(--amber-primary); }
  .station-node.unloading { border-right: 3px solid var(--cyan-primary); }
  .station-node.active-zone {
    box-shadow: 0 0 18px rgba(245, 158, 11, 0.35); transform: translateY(-2px);
    border-color: var(--amber-primary);
  }
  .station-label { font-size: 0.72rem; font-weight: 700; text-transform: uppercase; color: #fff; }
  .station-sensor-badge {
    font-family: monospace; font-size: 0.68rem; font-weight: 600;
    padding: 0.15rem 0.5rem; border-radius: 999px;
    background: rgba(0,0,0,0.45); color: var(--text-muted);
    border: 1px solid transparent; transition: all 0.2s ease;
  }
  .station-sensor-badge.active {
    background: rgba(16, 185, 129, 0.2); color: #34d399;
    border-color: rgba(16, 185, 129, 0.4); box-shadow: 0 0 10px rgba(16, 185, 129, 0.3);
  }

  /* SG90 Servo Arm Graphic */
  .servo-gate-graphic {
    position: absolute; right: 28px; top: 20px; display: flex; flex-direction: column;
    align-items: center; gap: 4px; z-index: 3;
  }
  .servo-arm {
    width: 6px; height: 34px; background: var(--amber-primary); border-radius: 3px;
    transform-origin: top center; transition: transform 0.6s cubic-bezier(0.34, 1.56, 0.64, 1);
    box-shadow: 0 0 8px rgba(245, 158, 11, 0.5);
  }
  .servo-label {
    font-family: monospace; font-size: 0.65rem; color: var(--amber-primary); font-weight: 700;
    background: rgba(0,0,0,0.65); padding: 2px 6px; border-radius: 4px; border: 1px solid rgba(245, 158, 11, 0.3);
  }

  /* Railway Line */
  .railway-line {
    position: relative; height: 12px; background: #1e293b; border-radius: 6px;
    border-top: 2px solid #475569; border-bottom: 2px solid #0f172a; margin: 1.5rem 0.5rem 0.5rem 0.5rem;
  }
  .railway-sleepers {
    position: absolute; top: -4px; left: 0; right: 0; height: 20px;
    background-image: repeating-linear-gradient(90deg, #334155 0px, #334155 4px, transparent 4px, transparent 18px);
    z-index: 1;
  }

  /* Physical Train Sprite */
  .train-wrapper {
    position: absolute; top: -46px; left: 88%;
    transition: left 0.45s ease-out; z-index: 5; transform: translateX(-50%);
  }
  .train-sprite { display: flex; align-items: flex-end; gap: 4px; filter: drop-shadow(0 6px 12px rgba(0,0,0,0.6)); }
  .train-locomotive {
    background: linear-gradient(135deg, #3b82f6 0%, #1d4ed8 100%);
    border: 1px solid #60a5fa; border-radius: 6px 14px 4px 4px;
    width: 56px; height: 37px; position: relative;
    display: flex; flex-direction: column; justify-content: flex-end; padding: 4px;
  }
  .loco-window {
    width: 14px; height: 11px; background: #bae6fd; border-radius: 3px;
    position: absolute; top: 6px; right: 8px; box-shadow: 0 0 8px rgba(186, 230, 253, 0.8);
  }
  .loco-light {
    width: 6px; height: 6px; background: #fef08a; border-radius: 50%;
    position: absolute; top: 14px; right: -3px; box-shadow: 0 0 12px #fde047;
  }
  .train-coal-car {
    background: linear-gradient(180deg, #1e293b 0%, #0f172a 100%);
    border: 1px solid #475569; border-radius: 4px;
    width: 62px; height: 31px; position: relative; overflow: visible;
  }
  .coal-pile {
    position: absolute; top: -11px; left: 5px; right: 5px; height: 13px;
    background: #020617; border-radius: 10px 10px 0 0; border: 1px solid #334155;
    display: flex; align-items: center; justify-content: center; gap: 2px;
    box-shadow: inset 0 2px 4px rgba(255, 255, 255, 0.1);
    transform-origin: bottom center; transform: scaleY(0); opacity: 0;
    transition: transform 0.4s ease, opacity 0.4s ease;
  }
  .coal-lump { width: 7px; height: 7px; background: #0f172a; border-radius: 50%; border: 1px solid #1e293b; }
  .train-wheels { display: flex; justify-content: space-between; padding: 0 4px; margin-top: -3px; }
  .wheel { width: 10px; height: 10px; background: #cbd5e1; border: 2px solid #475569; border-radius: 50%; }

  @keyframes trainWheelSpin {
    from { transform: rotate(0deg); }
    to { transform: rotate(360deg); }
  }
  .train-moving .wheel { animation: trainWheelSpin 0.35s linear infinite; }

  /* State Banner */
  .state-banner {
    display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; gap: 0.75rem;
    margin-top: 1rem; background: rgba(15, 23, 42, 0.6);
    border: 1px solid var(--border-subtle); border-radius: 10px; padding: 0.75rem 1rem;
  }
  .state-left { display: flex; align-items: center; gap: 0.75rem; }
  .state-badge {
    font-size: 0.78rem; font-weight: 800; font-family: monospace; letter-spacing: 0.05em;
    padding: 0.35rem 0.75rem; border-radius: 6px; border: 1px solid #22c55e; color: #22c55e;
    background: rgba(0, 0, 0, 0.3);
  }
  .state-desc { font-size: 0.8rem; color: var(--text-secondary); }
  .timer-badge {
    font-family: monospace; font-size: 0.75rem; font-weight: 700;
    padding: 0.25rem 0.65rem; border-radius: 999px;
    background: rgba(14, 165, 233, 0.15); color: #38bdf8; border: 1px solid rgba(14, 165, 233, 0.3);
    display: none;
  }

  /* Control Panels */
  .card-panel {
    background: var(--bg-card); border: 1px solid var(--border-subtle);
    border-radius: 14px; padding: 1.25rem; box-shadow: 0 4px 20px rgba(0, 0, 0, 0.4);
    display: flex; flex-direction: column; gap: 1rem;
  }

  /* Mode Switcher Tabs */
  .mode-tabs {
    display: grid; grid-template-columns: 1fr 1fr; background: rgba(0,0,0,0.4);
    padding: 0.3rem; border-radius: 10px; gap: 0.3rem; border: 1px solid var(--border-subtle);
  }
  .mode-tab-btn {
    border: none; background: transparent; color: var(--text-muted); font-size: 0.82rem; font-weight: 700;
    padding: 0.6rem; border-radius: 8px; cursor: pointer; transition: all 0.2s ease;
    display: flex; align-items: center; justify-content: center; gap: 0.45rem;
  }
  .mode-tab-btn.active {
    background: rgba(255, 255, 255, 0.1); color: #fff;
    box-shadow: 0 2px 8px rgba(0,0,0,0.3); border: 1px solid rgba(255, 255, 255, 0.15);
  }

  /* Buttons */
  .btn-primary-action {
    background: linear-gradient(135deg, #f59e0b 0%, #d97706 100%);
    color: #000; border: none; font-weight: 800; font-size: 0.95rem; letter-spacing: 0.04em;
    padding: 1rem 1.5rem; border-radius: 10px; cursor: pointer;
    box-shadow: 0 4px 18px rgba(245, 158, 11, 0.35); transition: all 0.2s ease;
    display: flex; align-items: center; justify-content: center; gap: 0.6rem; width: 100%;
  }
  .btn-primary-action:active { transform: scale(0.98); }
  .btn-load-coal {
    background: linear-gradient(135deg, #0ea5e9 0%, #0284c7 100%);
    color: #fff; border: 1px solid rgba(255, 255, 255, 0.2); font-weight: 800; font-size: 0.9rem;
    padding: 0.85rem 1.25rem; border-radius: 10px; cursor: pointer;
    box-shadow: 0 4px 16px rgba(14, 165, 233, 0.35); transition: all 0.2s ease;
    display: flex; align-items: center; justify-content: center; gap: 0.6rem; width: 100%;
  }
  .btn-load-coal:active { transform: scale(0.98); }
  .btn-pause-action {
    background: linear-gradient(135deg, #eab308 0%, #ca8a04 100%);
    color: #000; border: none; font-weight: 800; font-size: 0.92rem; letter-spacing: 0.04em;
    padding: 0.85rem 1.25rem; border-radius: 10px; cursor: pointer;
    box-shadow: 0 4px 16px rgba(234, 179, 8, 0.35); transition: all 0.2s ease;
    display: flex; align-items: center; justify-content: center; gap: 0.6rem; width: 100%;
  }
  .btn-pause-action:active { transform: scale(0.98); }
  .btn-resume-action {
    background: linear-gradient(135deg, #10b981 0%, #059669 100%);
    color: #fff; border: none; font-weight: 800; font-size: 0.92rem; letter-spacing: 0.04em;
    padding: 0.85rem 1.25rem; border-radius: 10px; cursor: pointer;
    box-shadow: 0 4px 16px rgba(16, 185, 129, 0.4); transition: all 0.2s ease;
    display: flex; align-items: center; justify-content: center; gap: 0.6rem; width: 100%;
    animation: pulseResume 1.8s infinite;
  }
  .btn-resume-action:active { transform: scale(0.98); }
  @keyframes pulseResume {
    0%, 100% { box-shadow: 0 0 14px rgba(16, 185, 129, 0.4); }
    50% { box-shadow: 0 0 24px rgba(16, 185, 129, 0.85); }
  }
  .btn-stop-neutral {
    background: rgba(239, 68, 68, 0.15); border: 1px solid rgba(239, 68, 68, 0.35);
    color: #f87171; font-weight: 700; font-size: 0.9rem; padding: 0.85rem; border-radius: 10px;
    cursor: pointer; display: flex; align-items: center; justify-content: center; gap: 0.5rem; width: 100%;
    transition: all 0.2s ease;
  }
  .btn-stop-neutral:active { transform: scale(0.98); }

  .toggle-row {
    display: flex; justify-content: space-between; align-items: center;
    padding: 0.75rem 0.5rem; border-top: 1px solid var(--border-subtle);
  }

  /* Manual D-Pad */
  .manual-dpad { display: flex; flex-direction: column; gap: 0.75rem; }
  .dpad-row { display: grid; grid-template-columns: 1fr 1fr; gap: 0.75rem; }
  .btn-control {
    background: rgba(255, 255, 255, 0.05); border: 1px solid var(--border-subtle);
    color: #fff; font-weight: 700; font-size: 0.85rem; padding: 0.85rem 1rem;
    border-radius: 10px; cursor: pointer; display: flex; align-items: center; justify-content: center;
    gap: 0.5rem; transition: all 0.15s ease;
  }
  .btn-control:hover { background: rgba(255, 255, 255, 0.1); }
  .btn-control:active { background: var(--amber-primary); color: #000; }

  /* Diagnostic Grid */
  .diag-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 0.75rem; }
  .diag-card {
    background: rgba(0,0,0,0.3); border: 1px solid var(--border-subtle);
    border-radius: 10px; padding: 0.75rem; display: flex; flex-direction: column; gap: 0.25rem;
  }
  .diag-label { font-size: 0.7rem; color: var(--text-muted); text-transform: uppercase; }
  .diag-val { font-size: 0.95rem; font-weight: 700; color: #fff; font-family: monospace; }

  /* Activity Logs Box */
  .log-container {
    background: #050811; border: 1px solid rgba(255,255,255,0.06); border-radius: 10px;
    padding: 0.75rem; height: 130px; overflow-y: auto; font-family: monospace; font-size: 0.73rem;
    display: flex; flex-direction: column; gap: 0.35rem;
  }
  .log-line { line-height: 1.35; color: #94a3b8; }
  .log-line.highlight { color: #f59e0b; font-weight: 600; }
  .log-line.success { color: #34d399; font-weight: 600; }
  .log-time { color: #64748b; margin-right: 0.35rem; }
</style>
</head>
<body>
<div class="dashboard-container">
  <!-- Header Bar -->
  <header class="dashboard-header">
    <div class="brand-group">
      <div class="brand-icon">🚂</div>
      <div>
        <h1 class="brand-title">COAL TRANSPORT</h1>
        <p class="brand-subtitle">ESP32 Autonomous Train Control</p>
      </div>
    </div>
    <div class="header-badges">
      <div id="conn-pill" class="status-pill online">
        <span class="status-dot"></span>
        <span id="conn-text">LIVE</span>
      </div>
    </div>
  </header>

  <!-- Interactive Animated Railway Canvas -->
  <section class="track-canvas-card">
    <div class="card-header-bar">
      <div class="card-title">👁️ Live Track & Telemetry View</div>
      <div class="pos-badge" id="track-pos-badge">Position: 88%</div>
    </div>

    <div class="track-stage">
      <!-- Stations -->
      <div class="track-waypoints">
        <!-- Mining Area Station -->
        <div class="station-node mining" id="station-mining">
          <span class="station-label">⛏️ Mining Station</span>
          <div class="station-sensor-badge" id="sensor-rear-badge">
            <span>Rear US (P33/35):</span>
            <strong id="sensor-rear-text">-- cm</strong>
          </div>
        </div>

        <!-- Unloading Area Station -->
        <div class="station-node unloading active-zone" id="station-unloading">
          <span class="station-label">🏭 Unloading Station</span>
          <div class="station-sensor-badge" id="sensor-front-badge">
            <span>Front US (P32/34):</span>
            <strong id="sensor-front-text">-- cm</strong>
          </div>
        </div>
      </div>

      <!-- SG90 Servo Gate Graphic -->
      <div class="servo-gate-graphic">
        <div class="servo-arm" id="servo-arm"></div>
        <span class="servo-label" id="servo-label">SG90: 90°</span>
      </div>

      <!-- Railway Line -->
      <div class="railway-line">
        <div class="railway-sleepers"></div>

        <!-- Animated Train Sprite -->
        <div class="train-wrapper" id="train-wrapper">
          <div class="train-sprite">
            <!-- Coal Hopper Car -->
            <div class="train-coal-car">
              <div class="coal-pile" id="coal-pile">
                <div class="coal-lump"></div>
                <div class="coal-lump"></div>
                <div class="coal-lump"></div>
              </div>
              <div class="train-wheels">
                <div class="wheel"></div>
                <div class="wheel"></div>
              </div>
            </div>

            <!-- Locomotive -->
            <div class="train-locomotive">
              <div class="loco-window"></div>
              <div class="loco-light"></div>
              <div class="train-wheels">
                <div class="wheel"></div>
                <div class="wheel"></div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- State Status Banner -->
    <div class="state-banner">
      <div class="state-left">
        <div class="state-badge" id="state-badge">PROJECT_COMPLETE</div>
        <span class="state-desc" id="state-desc">Cycle complete. Ready for next command.</span>
      </div>
      <div class="timer-badge" id="timer-badge">Timer: 10s remaining</div>
    </div>
  </section>

  <!-- Command Control Center -->
  <section class="card-panel">
    <div class="card-title">🎛️ Train Command Center</div>

    <!-- Mode Tabs -->
    <div class="mode-tabs">
      <button class="mode-tab-btn active" id="tab-auto" onclick="switchMode('AUTO')">
        🔄 Auto Sequence
      </button>
      <button class="mode-tab-btn" id="tab-manual" onclick="switchMode('MANUAL')">
        🎮 Manual Remote
      </button>
    </div>

    <!-- AUTO MODE VIEW -->
    <div id="view-auto" style="display: flex; flex-direction: column; gap: 0.85rem;">
      <button class="btn-primary-action" onclick="sendCmd('start_auto')">
        ▶ START AUTONOMOUS CYCLE
      </button>

      <button id="btn-pause-auto" class="btn-pause-action" onclick="togglePause()">
        ⏸ PAUSE TRAIN
      </button>

      <button class="btn-load-coal" onclick="sendCmd('load_and_run')">
        ⚡ LOAD COAL & RUN DEFAULT
      </button>

      <button class="btn-stop-neutral" onclick="sendCmd('stop')">
        ⏹ EMERGENCY STOP
      </button>

      <div class="toggle-row">
        <span style="font-size:0.85rem; color:var(--text-secondary);">
          Continuous Auto-Loop (Repeat cycle automatically)
        </span>
        <input type="checkbox" id="chk-continuous" style="width:20px; height:20px; accent-color:#f59e0b; cursor:pointer;" onchange="toggleContinuous(this)">
      </div>
    </div>

    <!-- MANUAL MODE VIEW -->
    <div id="view-manual" style="display: none;" class="manual-dpad">
      <div class="dpad-row">
        <button class="btn-control" onclick="sendCmd('forward')">▲ FORWARD</button>
        <button class="btn-control" onclick="sendCmd('reverse')">▼ REVERSE</button>
      </div>
      <div class="dpad-row">
        <button id="btn-pause-manual" class="btn-pause-action" onclick="togglePause()">⏸ PAUSE</button>
        <button class="btn-stop-neutral" onclick="sendCmd('stop')">⏹ STOP MOTOR</button>
      </div>
      <div class="dpad-row">
        <button class="btn-control" onclick="sendCmd('gate_open')">⏏ OPEN GATE (60°)</button>
        <button class="btn-control" onclick="sendCmd('gate_close')">⏏ CLOSE GATE (90°)</button>
      </div>
      <div class="dpad-row" style="margin-top:0.35rem;">
        <button class="btn-control" style="background:#4338ca; border-color:#6366f1;" onclick="sendCmd('gate_test')">🔄 TEST GATE SWEEP</button>
      </div>
      <div style="display:flex; flex-direction:column; gap:0.35rem; margin-top:0.4rem;">
        <div style="display:flex; justify-content:space-between; font-size:0.75rem; color:var(--text-secondary);">
          <span>Custom Gate Servo Angle</span>
          <span id="slider-val" style="color:var(--amber-primary); font-weight:700;">90°</span>
        </div>
        <input type="range" min="0" max="180" value="90" id="gate-slider" style="width:100%; accent-color:#f59e0b;" oninput="onSliderMove(this.value)">
      </div>
    </div>

    <!-- Train Speed Control Center (Default: 100% full speed, adjustable & decreasable) -->
    <div style="background: rgba(0,0,0,0.35); border: 1px solid var(--border-subtle); border-radius: 10px; padding: 0.85rem; margin-top: 0.6rem; display: flex; flex-direction: column; gap: 0.55rem;">
      <div style="display: flex; justify-content: space-between; align-items: center;">
        <span style="font-size: 0.82rem; font-weight: 700; color: var(--text-secondary); display: flex; align-items: center; gap: 0.35rem;">
          ⚡ Train Motor Speed (Default: 100%)
        </span>
        <span id="speed-display" style="font-family: monospace; font-size: 0.95rem; font-weight: 800; color: var(--amber-primary);">
          100% (255)
        </span>
      </div>

      <!-- Speed Slider -->
      <div style="display: flex; align-items: center; gap: 0.6rem;">
        <span style="font-size: 0.72rem; color: var(--text-muted); font-weight: 700;">30%</span>
        <input type="range" min="70" max="255" value="255" id="train-speed-slider" style="flex: 1; accent-color: #f59e0b; cursor: pointer;" oninput="onSpeedSlider(this.value)" onmousedown="isDraggingSpeed=true" onmouseup="isDraggingSpeed=false" ontouchstart="isDraggingSpeed=true" ontouchend="isDraggingSpeed=false">
        <span style="font-size: 0.72rem; color: var(--text-muted); font-weight: 700;">100%</span>
      </div>

      <!-- Quick Speed Preset Buttons -->
      <div style="display: grid; grid-template-columns: repeat(4, 1fr); gap: 0.35rem;">
        <button id="spd-btn-100" onclick="setSpeed(255)" style="background: rgba(245,158,11,0.25); border: 1px solid #f59e0b; color: #f59e0b; padding: 0.4rem; border-radius: 6px; font-weight: 700; font-size: 0.72rem; cursor: pointer;">
          100% (Def)
        </button>
        <button id="spd-btn-80" onclick="setSpeed(200)" style="background: rgba(255,255,255,0.06); border: 1px solid var(--border-subtle); color: #fff; padding: 0.4rem; border-radius: 6px; font-weight: 700; font-size: 0.72rem; cursor: pointer;">
          80% (Mid)
        </button>
        <button id="spd-btn-60" onclick="setSpeed(150)" style="background: rgba(255,255,255,0.06); border: 1px solid var(--border-subtle); color: #fff; padding: 0.4rem; border-radius: 6px; font-weight: 700; font-size: 0.72rem; cursor: pointer;">
          60% (Slow)
        </button>
        <button id="spd-btn-40" onclick="setSpeed(100)" style="background: rgba(255,255,255,0.06); border: 1px solid var(--border-subtle); color: #fff; padding: 0.4rem; border-radius: 6px; font-weight: 700; font-size: 0.72rem; cursor: pointer;">
          40% (Crawl)
        </button>
      </div>

      <!-- Decrease / Increase Speed Buttons -->
      <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 0.4rem; margin-top: 0.2rem;">
        <button onclick="decreaseSpeed()" style="background: rgba(239, 68, 68, 0.12); border: 1px solid rgba(239, 68, 68, 0.35); color: #f87171; padding: 0.45rem; border-radius: 8px; font-weight: 700; font-size: 0.78rem; cursor: pointer; display: flex; align-items: center; justify-content: center; gap: 0.35rem;">
          ▼ Decrease Speed (-10%)
        </button>
        <button onclick="increaseSpeed()" style="background: rgba(16, 185, 129, 0.12); border: 1px solid rgba(16, 185, 129, 0.35); color: #34d399; padding: 0.45rem; border-radius: 8px; font-weight: 700; font-size: 0.78rem; cursor: pointer; display: flex; align-items: center; justify-content: center; gap: 0.35rem;">
          ▲ Increase Speed (+10%)
        </button>
      </div>
    </div>
  </section>

  <!-- Live Telemetry Diagnostics -->
  <section class="card-panel">
    <div class="card-title">📊 Live Hardware Diagnostics</div>
    <div class="diag-grid">
      <div class="diag-card">
        <span class="diag-label">Rear Ultrasonic (P33/P35)</span>
        <span class="diag-val" id="diag-rear">-- cm</span>
      </div>
      <div class="diag-card">
        <span class="diag-label">Front Ultrasonic (P32/P34)</span>
        <span class="diag-val" id="diag-front">-- cm</span>
      </div>
      <div class="diag-card">
        <span class="diag-label">SG90 Servo Position</span>
        <span class="diag-val" id="diag-servo">90° (Closed)</span>
      </div>
      <div class="diag-card">
        <span class="diag-label">L298N Motor Driver</span>
        <span class="diag-val" id="diag-motor">STOPPED</span>
      </div>
      <div class="diag-card" style="grid-column: span 2;">
        <span class="diag-label">Motor Speed (PWM)</span>
        <span class="diag-val" id="diag-speed" style="color: var(--amber-primary);">100% (255 PWM - Default)</span>
      </div>
    </div>
  </section>

  <!-- Live Activity Console Logs -->
  <section class="card-panel">
    <div class="card-title">📜 Real-time System Event Log</div>
    <div class="log-container" id="log-box">
      <div class="log-line"><span class="log-time">[System]</span> Embedded Rail Controller Initialized. Ready.</div>
    </div>
  </section>
</div>

<script>
  const STATE_CONFIG = {
    REVERSING_TO_MINE: { label: 'REVERSING TO MINE', text: 'Train reversing to mining zone. Waiting for rear Ultrasonic (≤ 5 cm)...', color: '#f59e0b' },
    LOADING_COAL: { label: 'LOADING COAL', text: 'Train docked at mining station. Loading coal...', color: '#0ea5e9' },
    WAITING_TO_MOVE_FORWARD: { label: 'DIRECTION PAUSE', text: 'Coal loaded. Pausing before moving forward...', color: '#a855f7' },
    MOVING_TO_UNLOADING_AREA: { label: 'MOVING TO UNLOADING', text: 'Moving forward to unloading station. Waiting for front Ultrasonic (≤ 5 cm)...', color: '#10b981' },
    WAITING_TO_OPEN_GATE: { label: 'WAITING TO UNLOAD', text: 'Unloading station reached. Waiting 3s before opening gate...', color: '#a855f7' },
    UNLOADING_COAL: { label: 'UNLOADING COAL', text: 'Station reached (≤ 5 cm)! SG90 gate opened (60°). Unloading coal...', color: '#06b6d4' },
    CLOSING_GATE: { label: 'CLOSING GATE', text: 'Coal unloaded. Closing SG90 servo gate (90°)...', color: '#f97316' },
    PROJECT_COMPLETE: { label: 'CYCLE COMPLETED', text: 'Autonomous coal transport cycle finished successfully!', color: '#22c55e' },
    MANUAL_IDLE: { label: 'MANUAL IDLE', text: 'Manual remote mode active. Train stopped.', color: '#94a3b8' },
    MANUAL_FORWARD: { label: 'MANUAL FORWARD', text: 'Manual Forward Active (Motor moving forward)', color: '#10b981' },
    MANUAL_REVERSE: { label: 'MANUAL REVERSE', text: 'Manual Reverse Active (Motor reversing)', color: '#f59e0b' },
    PAUSED: { label: 'PAUSED', text: 'Train operation paused. Click RESUME to continue from this exact point.', color: '#eab308' }
  };

  let continuousLoop = false;
  let currentMode = 'AUTO';
  let lastKnownState = '';
  let countdownTimer = 0;
  let countdownInterval = null;
  let loopAutoTrigger = null;
  let isTrainPaused = false;
  let pausedStateName = '';

  function logEvent(msg, type = 'normal') {
    const box = document.getElementById('log-box');
    const now = new Date();
    const ts = now.toTimeString().split(' ')[0];
    const el = document.createElement('div');
    el.className = 'log-line ' + type;
    el.innerHTML = `<span class="log-time">[${ts}]</span> ${msg}`;
    box.appendChild(el);
    box.scrollTop = box.scrollHeight;
  }

  function togglePause() {
    if (isTrainPaused) {
      logEvent('User clicked RESUME', 'highlight');
      sendCmd('resume');
    } else {
      logEvent('User clicked PAUSE', 'highlight');
      sendCmd('pause');
    }
  }

  function switchMode(m) {
    currentMode = m;
    document.getElementById('tab-auto').classList.toggle('active', m === 'AUTO');
    document.getElementById('tab-manual').classList.toggle('active', m === 'MANUAL');
    document.getElementById('view-auto').style.display = (m === 'AUTO') ? 'flex' : 'none';
    document.getElementById('view-manual').style.display = (m === 'MANUAL') ? 'flex' : 'none';
    logEvent('Switched interface view to ' + m + ' Mode');
  }

  function toggleContinuous(cb) {
    continuousLoop = cb.checked;
    logEvent('Continuous Auto-Loop ' + (continuousLoop ? 'ENABLED' : 'DISABLED'), 'highlight');
  }

  function onSliderMove(val) {
    document.getElementById('slider-val').textContent = val + '°';
    fetch('/api/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'action=gate_angle&angle=' + val
    }).catch(() => {});
  }

  let currentSpeed = 255;
  let isDraggingSpeed = false;

  function onSpeedSlider(val) {
    currentSpeed = parseInt(val);
    updateSpeedUI(currentSpeed);
    fetch('/api/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'action=set_speed&speed=' + currentSpeed
    }).catch(() => {});
  }

  function setSpeed(val) {
    currentSpeed = parseInt(val);
    const slider = document.getElementById('train-speed-slider');
    if (slider) slider.value = currentSpeed;
    updateSpeedUI(currentSpeed);
    sendCmd('set_speed&speed=' + currentSpeed);
  }

  function decreaseSpeed() {
    currentSpeed = Math.max(70, currentSpeed - 25);
    const slider = document.getElementById('train-speed-slider');
    if (slider) slider.value = currentSpeed;
    updateSpeedUI(currentSpeed);
    sendCmd('set_speed&speed=' + currentSpeed);
  }

  function increaseSpeed() {
    currentSpeed = Math.min(255, currentSpeed + 25);
    const slider = document.getElementById('train-speed-slider');
    if (slider) slider.value = currentSpeed;
    updateSpeedUI(currentSpeed);
    sendCmd('set_speed&speed=' + currentSpeed);
  }

  function updateSpeedUI(spd) {
    const pct = Math.round((spd * 100) / 255);
    const disp = document.getElementById('speed-display');
    if (disp) disp.textContent = `${pct}% (${spd})`;
    ['100', '80', '60', '40'].forEach(id => {
      const btn = document.getElementById('spd-btn-' + id);
      if (btn) {
        btn.style.background = 'rgba(255,255,255,0.06)';
        btn.style.borderColor = 'var(--border-subtle)';
        btn.style.color = '#fff';
      }
    });
    let activeId = null;
    if (spd >= 235) activeId = '100';
    else if (spd >= 180) activeId = '80';
    else if (spd >= 130) activeId = '60';
    else if (spd >= 70) activeId = '40';
    if (activeId) {
      const activeBtn = document.getElementById('spd-btn-' + activeId);
      if (activeBtn) {
        activeBtn.style.background = 'rgba(245,158,11,0.25)';
        activeBtn.style.borderColor = '#f59e0b';
        activeBtn.style.color = '#f59e0b';
      }
    }
  }

  async function sendCmd(action) {
    try {
      logEvent('Sending Command: ' + action.toUpperCase(), 'highlight');
      await fetch('/api/cmd', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'action=' + encodeURIComponent(action)
      });
      setTimeout(pollStatus, 150);
    } catch (e) {
      logEvent('Command failed: ' + e.message, 'normal');
    }
  }

  async function pollStatus() {
    try {
      const res = await fetch('/api/status', { signal: AbortSignal.timeout(1200) });
      if (!res.ok) throw new Error('Status error');
      const data = await res.json();

      // Connectivity
      document.getElementById('conn-pill').className = 'status-pill online';
      document.getElementById('conn-text').textContent = 'LIVE (' + (data.rssi || 0) + ' dBm)';

      // Train Position & Wheels Animation
      const pos = (data.trainPosition !== undefined) ? data.trainPosition : 88;
      const trainEl = document.getElementById('train-wrapper');
      trainEl.style.left = pos + '%';

      const isMoving = (data.in1 !== data.in2);
      trainEl.classList.toggle('train-moving', isMoving);

      document.getElementById('track-pos-badge').textContent = 'Position: ' + Math.round(pos) + '%';

      // Coal Pile Animation
      const cLevel = (data.coalLevel !== undefined) ? data.coalLevel : 0;
      const pileEl = document.getElementById('coal-pile');
      pileEl.style.transform = `scaleY(${cLevel / 100})`;
      pileEl.style.opacity = cLevel > 3 ? '1' : '0';

      // SG90 Servo Arm
      const angle = (data.servoAngle !== undefined) ? data.servoAngle : 90;
      document.getElementById('servo-arm').style.transform = `rotate(${90 - angle}deg)`;
      document.getElementById('servo-label').textContent = `SG90: ${angle}°`;

      // Station highlights & Sensors
      document.getElementById('station-mining').classList.toggle('active-zone', pos <= 22);
      document.getElementById('station-unloading').classList.toggle('active-zone', pos >= 78);

      const rActive = Boolean(data.rearIr);
      const fActive = Boolean(data.frontIr);
      const rDistStr = (data.rearDist !== undefined && data.rearDist >= 0) ? `${data.rearDist} cm` : '>300 cm';
      const fDistStr = (data.frontDist !== undefined && data.frontDist >= 0) ? `${data.frontDist} cm` : '>300 cm';

      document.getElementById('sensor-rear-badge').classList.toggle('active', rActive);
      document.getElementById('sensor-rear-text').textContent = rActive ? `${rDistStr} (≤5cm)` : rDistStr;
      document.getElementById('sensor-front-badge').classList.toggle('active', fActive);
      document.getElementById('sensor-front-text').textContent = fActive ? `${fDistStr} (≤5cm)` : fDistStr;

      // Pause State UI handling
      isTrainPaused = Boolean(data.isPaused || data.state === 'PAUSED');
      pausedStateName = data.pausedState || '';

      const btnAutoPause = document.getElementById('btn-pause-auto');
      const btnManualPause = document.getElementById('btn-pause-manual');

      if (isTrainPaused) {
        const resumeText = pausedStateName ? `▶ RESUME (${pausedStateName})` : '▶ RESUME TRAIN';
        if (btnAutoPause) {
          btnAutoPause.className = 'btn-resume-action';
          btnAutoPause.innerHTML = resumeText;
        }
        if (btnManualPause) {
          btnManualPause.className = 'btn-resume-action';
          btnManualPause.innerHTML = '▶ RESUME';
        }
      } else {
        if (btnAutoPause) {
          btnAutoPause.className = 'btn-pause-action';
          btnAutoPause.innerHTML = '⏸ PAUSE TRAIN';
        }
        if (btnManualPause) {
          btnManualPause.className = 'btn-pause-action';
          btnManualPause.innerHTML = '⏸ PAUSE';
        }
      }

      // State Banner
      const st = data.state || 'UNKNOWN';
      const conf = STATE_CONFIG[st] || { label: st, text: 'Active state', color: '#38bdf8' };
      const badgeEl = document.getElementById('state-badge');
      badgeEl.textContent = conf.label;
      badgeEl.style.color = conf.color;
      badgeEl.style.borderColor = conf.color;
      document.getElementById('state-desc').textContent = isTrainPaused && pausedStateName
        ? `Paused at [${pausedStateName}]. Click RESUME to continue.`
        : conf.text;

      // Handle State Transitions & Countdown Timers
      if (st !== lastKnownState) {
        logEvent('State Changed: ' + conf.label, 'highlight');
        if (st === 'LOADING_COAL') {
          startCountdown(10, 'Loading Coal');
        } else if (st === 'WAITING_TO_OPEN_GATE') {
          startCountdown(3, 'Opening Gate');
        } else if (st === 'UNLOADING_COAL') {
          startCountdown(6, 'Unloading Coal');
        } else if (st !== 'PAUSED') {
          stopCountdown();
        }

        // Continuous Loop Trigger
        if (st === 'PROJECT_COMPLETE' && continuousLoop) {
          logEvent('Continuous Loop: Restarting next cycle in 3s...', 'success');
          clearTimeout(loopAutoTrigger);
          loopAutoTrigger = setTimeout(() => {
            if (continuousLoop) sendCmd('start_auto');
          }, 3000);
        }

        lastKnownState = st;
      }

      // Diagnostics Grid
      document.getElementById('diag-rear').textContent = `${rDistStr} (${rActive ? 'DETECTED' : 'CLEAR'})`;
      document.getElementById('diag-front').textContent = `${fDistStr} (${fActive ? 'DETECTED' : 'CLEAR'})`;
      document.getElementById('diag-servo').textContent = `${angle}° (${angle <= 75 ? 'OPEN' : 'CLOSED'})`;
      let mLabel = 'STOPPED';
      if (data.in1 && !data.in2) mLabel = 'FORWARD';
      else if (!data.in1 && data.in2) mLabel = 'REVERSE';
      const spdPct = (data.speedPercent !== undefined) ? data.speedPercent : Math.round(((data.speed || 255) * 100) / 255);
      if (mLabel !== 'STOPPED') mLabel += ` (${spdPct}%)`;
      document.getElementById('diag-motor').textContent = mLabel;

      const diagSpd = document.getElementById('diag-speed');
      if (diagSpd) {
        diagSpd.textContent = `${spdPct}% (${data.speed || 255} PWM${(data.speed === 255 || data.speed === undefined) ? ' - Default' : ''})`;
      }

      if (data.speed !== undefined && !isDraggingSpeed) {
        currentSpeed = data.speed;
        const slider = document.getElementById('train-speed-slider');
        if (slider && document.activeElement !== slider) {
          slider.value = currentSpeed;
        }
        updateSpeedUI(currentSpeed);
      }

    } catch (e) {
      document.getElementById('conn-pill').className = 'status-pill';
      document.getElementById('conn-text').textContent = 'DISCONNECTED';
    }
  }

  function startCountdown(sec, action) {
    stopCountdown();
    countdownTimer = sec;
    const badge = document.getElementById('timer-badge');
    badge.style.display = 'inline-block';
    badge.textContent = `Timer: ${countdownTimer}s remaining`;
    countdownInterval = setInterval(() => {
      if (isTrainPaused) return; // Freeze countdown while paused
      countdownTimer--;
      if (countdownTimer <= 0) {
        stopCountdown();
      } else {
        badge.textContent = `Timer: ${countdownTimer}s remaining`;
      }
    }, 1000);
  }

  function stopCountdown() {
    if (countdownInterval) clearInterval(countdownInterval);
    countdownInterval = null;
    document.getElementById('timer-badge').style.display = 'none';
  }

  setInterval(pollStatus, 400);
  pollStatus();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  setCORS();
  server.send_P(200, "text/html", INDEX_HTML);
}
