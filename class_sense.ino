/*
==================== TEACHER'S PET (FINAL FULL WORKING CODE) ====================

What it does:
- ESP32 creates WiFi AP: TeachersPet / 12345678
- Website at http://192.168.4.1 shows START + STOP, OLED control, Heatmap + scores
- When START pressed:
    - current position becomes (0,0), heading = 0°
    - robot patrols inside 100cm x 100cm boundary
- Quiet scoring:
    - if mic RAW < 700 during patrol step => +1 score to current grid cell
- Noise trigger:
    - if mic RAW >= 1200 => robot stops, rotates 360°, finds loudest direction
    - if that direction would go outside boundary => ignore and continue patrol
    - else robot moves toward loudest direction
- Obstacle handling (1 ultrasonic, best possible):
    - if obstacle ahead: scan right+left distances, choose freer side
    - if both blocked: reverse + U-turn (escape)
- When robot reaches loudest nearby point / can't continue:
    - mark that cell RED for 15s on website (no waiting) + score -10
    - immediately resumes patrol
    - same cell can be re-marked after 15s cooldown

OLED:
- Default always: "STAY QUIET"
- Website can set custom OLED message and can return to default
- STOP button stops motors immediately.

Pins:
TB6612:
  AIN1=26  AIN2=27  PWMA=25
  BIN1=33  BIN2=32  PWMB=14
  STBY=13
HC-SR04:
  TRIG=5  ECHO=18 (ECHO must be divided to 3.3V)
MAX9814 Mic:
  OUT -> GPIO34
OLED (SH1106 I2C):
  SDA=22  SCL=21  Address 0x3C

Board: ESP32 Dev Module 

===============================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <math.h>

// -------------------- PINS --------------------
#define MIC_PIN 34

#define TRIG_PIN 5
#define ECHO_PIN 18

#define AIN1 26
#define AIN2 27
#define PWMA 25

#define BIN1 33
#define BIN2 32
#define PWMB 14

#define STBY 13

#define OLED_SDA 22
#define OLED_SCL 21
#define OLED_ADDR 0x3C

// -------------------- WIFI --------------------
const char* AP_SSID = "TeachersPet";
const char* AP_PASS = "12345678";
WebServer server(80);

// -------------------- OLED --------------------
Adafruit_SH1106G display(128, 64, &Wire, -1);
bool oledOK = false;
bool oledCustom = false;
String oledText = "STAY QUIET";

void oledDraw() {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  String msg = oledCustom ? oledText : String("STAY QUIET");
  msg.trim();
  if (msg.length() == 0) msg = " ";

  display.setTextSize(2);
  display.setCursor(0, 18);
  display.println(msg);
  display.display();
}

// -------------------- PWM (ESP32 LEDC) --------------------
static const int CH_A = 0;
static const int CH_B = 1;
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;  // 0..255

void pwmInit() {
  ledcSetup(CH_A, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, CH_A);
  ledcAttachPin(PWMB, CH_B);
}
void pwmWriteA(int v) { ledcWrite(CH_A, constrain(v, 0, 255)); }
void pwmWriteB(int v) { ledcWrite(CH_B, constrain(v, 0, 255)); }

// -------------------- MOTORS --------------------
int patrolPWM = 120;
int chasePWM  = 175;
int rotatePWM = 155;

void motorsEnable() { digitalWrite(STBY, HIGH); }
void motorsStop()   { pwmWriteA(0); pwmWriteB(0); }

void driveForward(int pwm) {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  pwmWriteA(pwm); pwmWriteB(pwm);
}

void driveBackward(int pwm) {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  pwmWriteA(pwm); pwmWriteB(pwm);
}

void rotateLeft(int pwm) {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  pwmWriteA(pwm); pwmWriteB(pwm);
}

void rotateRight(int pwm) {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  pwmWriteA(pwm); pwmWriteB(pwm);
}

// -------------------- ULTRASONIC --------------------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long us = pulseIn(ECHO_PIN, HIGH, 30000);
  if (us == 0) return -1;
  return (us * 0.0343f) / 2.0f;
}

// -------------------- SOUND --------------------
const int NOISE_THRESHOLD_RAW = 1300;
const int QUIET_THRESHOLD_RAW = 700;

int readRawAvg(int n = 25) {
  long s = 0;
  for (int i = 0; i < n; i++) {
    s += analogRead(MIC_PIN);
    delayMicroseconds(400);
  }
  return (int)(s / n);
}

int getP2PFast(int samples = 140) {
  int minV = 4095, maxV = 0;
  for (int i = 0; i < samples; i++) {
    int v = analogRead(MIC_PIN);
    if (v < minV) minV = v;
    if (v > maxV) maxV = v;
    delayMicroseconds(60);
  }
  return maxV - minV;
}

// smoother reading for echo rooms
int avgP2P(int reps = 4) {
  long s = 0;
  for (int i = 0; i < reps; i++) {
    s += getP2PFast(140);
    delay(5);
  }
  return (int)(s / reps);
}

// -------------------- BOUNDARY + POSE (100 x 100) --------------------
const float X_MAX_CM = 100.0;
const float Y_MAX_CM = 100.0;

// dead-reckoning (tune if needed)
float CM_PER_SEC_PATROL = 10.0;
float CM_PER_SEC_CHASE  = 14.0;
float DEG_PER_SEC_ROT   = 90.0;

float x_cm = 0.0, y_cm = 0.0;
float heading_deg = 0.0; // 0° = +X

float wrapDeg(float d) {
  while (d < 0) d += 360.0;
  while (d >= 360.0) d -= 360.0;
  return d;
}
float degToRad(float d) { return d * 3.1415926f / 180.0f; }

void updateForward(unsigned long ms, float cmPerSec) {
  float sec = ms / 1000.0f;
  float dist = cmPerSec * sec;
  float rad = degToRad(heading_deg);
  x_cm += dist * cos(rad);
  y_cm += dist * sin(rad);
}
void updateRotateLeft(unsigned long ms) {
  float sec = ms / 1000.0f;
  heading_deg = wrapDeg(heading_deg + DEG_PER_SEC_ROT * sec);
}
void updateRotateRight(unsigned long ms) {
  float sec = ms / 1000.0f;
  heading_deg = wrapDeg(heading_deg - DEG_PER_SEC_ROT * sec);
}

bool wouldExitForward(unsigned long ms, float cmPerSec) {
  float sec = ms / 1000.0f;
  float dist = cmPerSec * sec;
  float rad = degToRad(heading_deg);
  float nx = x_cm + dist * cos(rad);
  float ny = y_cm + dist * sin(rad);
  return (nx < 0 || ny < 0 || nx > X_MAX_CM || ny > Y_MAX_CM);
}

void clampPose() {
  if (x_cm < 0) x_cm = 0;
  if (y_cm < 0) y_cm = 0;
  if (x_cm > X_MAX_CM) x_cm = X_MAX_CM;
  if (y_cm > Y_MAX_CM) y_cm = Y_MAX_CM;
}

// -------------------- GRID HEATMAP --------------------
const int GRID_R = 4;
const int GRID_C = 4;
int scores[GRID_R][GRID_C];
unsigned long redUntil[GRID_R][GRID_C];
unsigned long lastNoisyMs[GRID_R][GRID_C];

const unsigned long RED_MS = 15000;
const unsigned long COOLDOWN_MS = 15000;

void gridRC(int &r, int &c) {
  clampPose();
  float cw = X_MAX_CM / GRID_C; // 25cm
  float ch = Y_MAX_CM / GRID_R; // 25cm
  c = (int)(x_cm / cw);
  r = (int)(y_cm / ch);
  if (c < 0) c = 0;
  if (r < 0) r = 0;
  if (c >= GRID_C) c = GRID_C - 1;
  if (r >= GRID_R) r = GRID_R - 1;
}

String cellName(int r, int c) {
  char row = 'A' + r;
  return String(row) + String(c + 1);
}

// -------------------- STATE --------------------
bool robotStarted = false;
bool robotStopped = false;

enum State { WAIT_START, PATROL, SCAN360, TURN_TO_GOAL, CHASE_NOISE };
State state = WAIT_START;

float goalHeading = 0.0;

// -------------------- MOVEMENT TIMINGS --------------------
unsigned long PATROL_STEP_MS = 350;
unsigned long CHASE_STEP_MS  = 280;

float OBSTACLE_CM = 20.0;

// -------------------- BEST 1-ULTRASONIC AVOID --------------------
void rotateRightMs(unsigned long ms) {
  rotateRight(rotatePWM);
  delay(ms);
  motorsStop();
  updateRotateRight(ms);
}

void rotateLeftMs(unsigned long ms) {
  rotateLeft(rotatePWM);
  delay(ms);
  motorsStop();
  updateRotateLeft(ms);
}

void avoidObstacleBest() {
  motorsStop();
  delay(80);

  // back a bit
  driveBackward(patrolPWM);
  delay(220);
  motorsStop();
  delay(80);

  const unsigned long SCAN_TURN_MS = 280; // ~25-35° depending on DEG_PER_SEC_ROT
  const unsigned long UTURN_MS     = 900; // ~90°-150° depending on DEG_PER_SEC_ROT

  // scan right
  rotateRightMs(SCAN_TURN_MS);
  delay(60);
  float dR = readDistanceCm(); if (dR < 0) dR = 0;

  // scan left (past center)
  rotateLeftMs(SCAN_TURN_MS * 2);
  delay(60);
  float dL = readDistanceCm(); if (dL < 0) dL = 0;

  // return to center
  rotateRightMs(SCAN_TURN_MS);

  // both blocked -> escape
  if (dR < OBSTACLE_CM && dL < OBSTACLE_CM) {
    rotateRightMs(UTURN_MS);
    driveBackward(patrolPWM);
    delay(250);
    motorsStop();
    delay(80);
    return;
  }

  // choose freer side
  if (dR > dL) rotateRightMs(SCAN_TURN_MS);
  else        rotateLeftMs(SCAN_TURN_MS);

  // move forward a bit to bypass
  if (!wouldExitForward(300, CM_PER_SEC_PATROL)) {
    driveForward(patrolPWM);
    delay(300);
    motorsStop();
    updateForward(300, CM_PER_SEC_PATROL);
  } else {
    motorsStop();
  }
  delay(80);
}

// -------------------- PATROL STEP (stay inside boundary) --------------------
void boundarySteer() {
  if (wouldExitForward(PATROL_STEP_MS, CM_PER_SEC_PATROL)) {
    // turn right 90°
    unsigned long ms = (unsigned long)((90.0 / DEG_PER_SEC_ROT) * 1000.0);
    rotateRightMs(ms);
  }
}

void patrolStep() {
  boundarySteer();

  float d = readDistanceCm();
  if (d > 0 && d < OBSTACLE_CM) {
    avoidObstacleBest();
    return;
  }

  driveForward(patrolPWM);
  delay(PATROL_STEP_MS);
  motorsStop();
  updateForward(PATROL_STEP_MS, CM_PER_SEC_PATROL);
}

// -------------------- 360 SCAN (best direction) --------------------
float scanLoudestHeading() {
  int bestVal = 0;
  float bestHeading = heading_deg;

  unsigned long totalMs = (unsigned long)((360.0 / DEG_PER_SEC_ROT) * 1000.0);
  unsigned long start = millis();

  while (millis() - start < totalMs) {
    rotateLeft(rotatePWM);
    delay(15);
    updateRotateLeft(15);

    int v = avgP2P(4);
    if (v > bestVal) {
      bestVal = v;
      bestHeading = heading_deg; // absolute
    }
  }

  motorsStop();
  return bestHeading;
}

// rotate shortest direction to target
void rotateToHeading(float target) {
  target = wrapDeg(target);
  float cur = wrapDeg(heading_deg);
  float diff = target - cur;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;

  unsigned long ms = (unsigned long)((fabs(diff) / DEG_PER_SEC_ROT) * 1000.0);

  if (ms < 20) return;

  if (diff > 0) {
    rotateLeft(rotatePWM);
    delay(ms);
    motorsStop();
    updateRotateLeft(ms);
  } else {
    rotateRight(rotatePWM);
    delay(ms);
    motorsStop();
    updateRotateRight(ms);
  }
}

// if goal direction leads outside boundary soon, ignore it
bool goalOutside() {
  // check 2 forward steps in that direction (quick prediction)
  float save = heading_deg;
  heading_deg = goalHeading;
  bool out = wouldExitForward(CHASE_STEP_MS * 2, CM_PER_SEC_CHASE);
  heading_deg = save;
  return out;
}

// -------------------- CHASE TOWARD NOISE --------------------
bool chaseStepTowardNoise() {
  // obstacle?
  float d = readDistanceCm();
  if (d > 0 && d < OBSTACLE_CM) {
    avoidObstacleBest();
    return true;
  }

  // boundary?
  if (wouldExitForward(CHASE_STEP_MS, CM_PER_SEC_CHASE)) {
    // can't go further in this direction
    return false;
  }

  // forward step
  driveForward(chasePWM);
  delay(CHASE_STEP_MS);
  motorsStop();
  updateForward(CHASE_STEP_MS, CM_PER_SEC_CHASE);
  return true;
}

// -------------------- WEB UI --------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Teacher's Pet</title>
<style>
body{font-family:Arial;margin:16px}
.row{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}
button,input{padding:10px 12px;border-radius:12px;border:1px solid #ccc}
.grid{display:grid;grid-template-columns:repeat(4,86px);gap:10px;margin-top:12px}
.cell{height:72px;border-radius:14px;display:flex;flex-direction:column;align-items:center;justify-content:center;
background:#e8f7e8;border:1px solid #bfe3bf;font-weight:700}
.red{background:#ffcccc;border-color:#ff9a9a}
.small{color:#666;font-size:12px}
</style>
</head><body>
<h2>Teacher's Pet</h2>
<div class="small">WiFi: TeachersPet / 12345678 — Open: 192.168.4.1</div>

<div class="row">
  <button onclick="fetch('/start')">START</button>
  <button onclick="fetch('/stop')">STOP</button>
</div>

<div class="row">
  <input id="oled" placeholder="OLED text">
  <button onclick="setOled()">SET OLED</button>
  <button onclick="fetch('/oled_default')">DEFAULT OLED</button>
</div>

<div class="small" id="status">Loading...</div>

<div id="grid" class="grid"></div>

<script>
const rows=["A","B","C","D"];
function cellId(r,c){ return rows[r]+(c+1); }

function setOled(){
  const t = document.getElementById("oled").value || "";
  fetch('/oled?msg='+encodeURIComponent(t));
}

function render(data){
  document.getElementById("status").textContent =
    (data.started ? "Started" : "Stopped") + " | x="+data.x.toFixed(1)+" y="+data.y.toFixed(1)+" heading="+data.h.toFixed(0);

  const g = document.getElementById("grid");
  g.innerHTML="";
  for(let r=0;r<4;r++){
    for(let c=0;c<4;c++){
      const div=document.createElement("div");
      div.className="cell";
      const id=cellId(r,c);
      if(data.red[r][c]) div.classList.add("red");
      div.innerHTML = `<div>${id}</div><div>${data.scores[r][c]}</div>`;
      g.appendChild(div);
    }
  }
}

async function poll(){
  try{
    const res=await fetch('/data');
    const data=await res.json();
    render(data);
  }catch(e){}
  setTimeout(poll,400);
}
poll();
</script>
</body></html>
)HTML";

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void handleStart() {
  robotStarted = true;
  robotStopped = false;
  state = PATROL;

  // set current standing point as (0,0)
  x_cm = 0.0; y_cm = 0.0; heading_deg = 0.0;

  server.send(200, "text/plain", "STARTED");
}

void handleStop() {
  robotStopped = true;
  state = WAIT_START;
  motorsStop();
  server.send(200, "text/plain", "STOPPED");
}

void handleOled() {
  if (server.hasArg("msg")) {
    oledText = server.arg("msg");
    oledCustom = true;
    oledDraw();
  }
  server.send(200, "text/plain", "OK");
}

void handleOledDefault() {
  oledCustom = false;
  oledDraw();
  server.send(200, "text/plain", "OK");
}

void handleData() {
  // JSON: {started,x,y,h,scores:[[..]],red:[[..]]}
  String out = "{";
  out += "\"started\":"; out += (robotStarted && !robotStopped) ? "true" : "false";
  out += ",\"x\":" + String(x_cm, 1);
  out += ",\"y\":" + String(y_cm, 1);
  out += ",\"h\":" + String(heading_deg, 0);

  out += ",\"scores\":[";
  for (int r=0;r<GRID_R;r++){
    out += "[";
    for (int c=0;c<GRID_C;c++){
      out += String(scores[r][c]);
      if (c<GRID_C-1) out += ",";
    }
    out += "]";
    if (r<GRID_R-1) out += ",";
  }
  out += "]";

  unsigned long now = millis();
  out += ",\"red\":[";
  for (int r=0;r<GRID_R;r++){
    out += "[";
    for (int c=0;c<GRID_C;c++){
      bool isRed = (now < redUntil[r][c]);
      out += isRed ? "true" : "false";
      if (c<GRID_C-1) out += ",";
    }
    out += "]";
    if (r<GRID_R-1) out += ",";
  }
  out += "]";

  out += "}";

  server.send(200, "application/json", out);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  // init arrays
  for (int r=0;r<GRID_R;r++){
    for (int c=0;c<GRID_C;c++){
      scores[r][c] = 0;
      redUntil[r][c] = 0;
      lastNoisyMs[r][c] = 0;
    }
  }

  // pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  pwmInit();
  motorsEnable();
  motorsStop();

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  oledOK = display.begin(OLED_ADDR, true);
  oledCustom = false;
  oledDraw();

  // WiFi AP + web
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/oled", handleOled);
  server.on("/oled_default", handleOledDefault);
  server.on("/data", handleData);
  server.begin();

  state = WAIT_START;
}

// -------------------- LOOP --------------------
void loop() {
  server.handleClient();

  // keep OLED alive
  static unsigned long lastOled = 0;
  if (millis() - lastOled > 800) {
    lastOled = millis();
    oledDraw();
  }

  if (!robotStarted || robotStopped) {
    motorsStop();
    state = WAIT_START;
    return;
  }

  // always know current cell
  int r=0,c=0;
  gridRC(r,c);

  int raw = readRawAvg(25);

  switch (state) {

    case WAIT_START:
      motorsStop();
      break;

    case PATROL: {
      // quiet scoring
      if (raw < QUIET_THRESHOLD_RAW) {
        scores[r][c] += 1;
      }

      patrolStep();

      // noise trigger
      raw = readRawAvg(20);
      if (raw >= NOISE_THRESHOLD_RAW) {
        state = SCAN360;
      }
    } break;

    case SCAN360: {
      motorsStop();
      goalHeading = scanLoudestHeading();
      state = TURN_TO_GOAL;
    } break;

    case TURN_TO_GOAL: {
      rotateToHeading(goalHeading);

      // if goal is outside boundary, ignore noise
      if (goalOutside()) {
        state = PATROL;
      } else {
        state = CHASE_NOISE;
      }
    } break;

    case CHASE_NOISE: {
      // chase until sound stops increasing OR can't continue
      int prev = avgP2P(4);
      int blockedCount = 0;

      for (int steps=0; steps<70; steps++) {
        if (robotStopped) { motorsStop(); state = WAIT_START; return; }

        // if heading drifted, gently re-align toward goal
        rotateToHeading(goalHeading);

        bool moved = chaseStepTowardNoise();
        if (!moved) { // boundary stop / can't continue
          blockedCount++;
          if (blockedCount > 2) break;
        }

        int now = avgP2P(4);
        if (now + 1 < prev) { // peak reached
          break;
        }
        prev = now;

        // if raw noise disappeared, stop searching further
        int rraw = readRawAvg(15);
        if (rraw < NOISE_THRESHOLD_RAW) break;
      }

      // mark noisy region (cooldown 15s)
      gridRC(r,c);
      unsigned long nowMs = millis();
      if (nowMs - lastNoisyMs[r][c] >= COOLDOWN_MS) {
        lastNoisyMs[r][c] = nowMs;
        scores[r][c] += -10;
        redUntil[r][c] = nowMs + RED_MS;
      }

      // immediately resume patrol (no waiting)
      state = PATROL;
    } break;
  }
}