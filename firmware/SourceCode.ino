// ===== Ball Balancing Table + WiFi AP + Web Dashboard (Position + FFT + PID) =====

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------- WiFi / Web ----------
const char* AP_SSID = "Ball Balancing Table";
const char* AP_PASS = "12345678";

WebServer server(80);
WebSocketsServer webSocket(81);

// Streaming globals to send to dashboard
volatile float g_x_raw   = 0.0f;
volatile float g_y_raw   = 0.0f;
volatile float g_x_filt  = 0.0f;   // always computed (pos filter output)
volatile float g_y_filt  = 0.0f;   // always computed (pos filter output)
volatile float g_servoX  = 0.0f;
volatile float g_servoY  = 0.0f;

// Filter usage in CONTROL calculations only
volatile bool g_usePosFilter = false; // default: raw in calculations
volatile bool g_useVelFilter = false; // default: raw velocity in calculations

// Forward declarations
void handleCommandLine(String line);
void printStatus();
void printHelp();
void recomputeServoLimits();
void handleRoot();
extern const char INDEX_HTML[] PROGMEM;

// ---------- PID State ----------
struct PIDState {
  float i = 0.0f;
};

static float pid_step(PIDState &s,
                      float e_cm,
                      float de_dt_cm_s,
                      float Kp, float Ki, float Kd, float Kaw,
                      float u_min, float u_max,
                      float dt);

// ---------- Pins ----------
#define XP 32
#define XM 25
#define YP 33
#define YM 26
const int PROBE = 2;

const int SERVO_X_PIN = 18;   // MG996R X
const int SERVO_Y_PIN = 5;    // MG996R Y

// ---------- Servo ----------
Servo servoX, servoY;
const int   SERVO_HZ = 50;
const int   US_MIN   = 500;
const int   US_MAX   = 2500;

// Flat angles (level plate)
float SERVO_X_FLAT = 79.00f;
float SERVO_Y_FLAT = 94.00f;
const float SERVO_SPAN = 60.0f;

// Servo angle limits
float SERVO_X_MIN  = 0.0f;
float SERVO_X_MAX  = 180.0f;
float SERVO_Y_MIN  = 0.0f;
float SERVO_Y_MAX  = 180.0f;

// Axis inversion
bool SERVO_INVERT_X = true;
bool SERVO_INVERT_Y = false;

// ---------- Touch calibration ----------
int Xmin = 510,  Xmax = 3340;
int Ymin = 240,  Ymax = 4000;

const int X_CENTER_RAW = 1860;
const int Y_CENTER_RAW = 1950;

//---------- Touch dimensions ----------
const float X_HALF_RANGE_CM = 18.45f * 0.5f;
const float Y_HALF_RANGE_CM = 24.55f * 0.5f;

// ---------- Control loop timing ----------
const float    CTRL_HZ    = 200.0f;
const float    DT_TARGET  = 1.0f / CTRL_HZ;
const uint32_t CTRL_DT_US = (uint32_t)(DT_TARGET * 1e6f);
uint32_t last_us = 0;

// ---------- Velocity filter ----------
float FC_VEL           = 40.0f;
const float VEL_CLIP     = 200.0f;
const float VEL_RAW_CLIP = 600.0f;

inline float alpha_vel(float dt) {
  float a = 1.0f - expf(-2.0f * (float)M_PI * FC_VEL * dt);
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  return a;
}

// ---------- PID ----------
PIDState pidX, pidY;

float Kp_x = 3.0f, Ki_x = 0.0f, Kd_x = 1.1f;
float Kp_y = 3.0f, Ki_y = 0.0f, Kd_y = 1.1f;

float Kaw_x = 1.0f / 3.0f;
float Kaw_y = 1.0f / 3.0f;

float EPS_ERR_CM = 0.2f;

float ALPHA_POS = 1.0f;

// ---------- Path-following / target mode ----------
enum ControlMode {
  MODE_TARGET   = 0,
  MODE_SQUARE   = 1,
  MODE_CIRCLE   = 2,
  MODE_INFINITY = 3
};

volatile int g_mode = MODE_TARGET;

// Target coordinates (cm) for MODE_TARGET
volatile float g_targetX_cm = 0.0f;
volatile float g_targetY_cm = 0.0f;

// Mode timing
uint32_t modeStartMs = 0;

// Shape parameters (all in cm and Hz)
float shapeFreqHz      = 0.10f;
float circleRadiusCm   = 3.0f;
float squareHalfCm     = 3.0f;
float infAmpX_cm       = 3.0f;
float infAmpY_cm       = 3.0f;

// ---------- Helpers ----------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void recomputeServoLimits() {
  SERVO_X_MIN  = max(0.0f,   SERVO_X_FLAT - SERVO_SPAN);
  SERVO_X_MAX  = min(180.0f, SERVO_X_FLAT + SERVO_SPAN);
  SERVO_Y_MIN  = max(0.0f,   SERVO_Y_FLAT - SERVO_SPAN);
  SERVO_Y_MAX  = min(180.0f, SERVO_Y_FLAT + SERVO_SPAN);
}

// ---------- Raw touchscreen reads ----------
int readX_raw() {
  pinMode(XP, OUTPUT);  digitalWrite(XP, HIGH);
  pinMode(XM, OUTPUT);  digitalWrite(XM, LOW);
  pinMode(YP, INPUT);
  pinMode(YM, INPUT);
  delayMicroseconds(80);
  (void)analogRead(YP);
  return analogRead(YP);
}

int readY_raw() {
  pinMode(YP, OUTPUT);  digitalWrite(YP, HIGH);
  pinMode(YM, OUTPUT);  digitalWrite(YM, LOW);
  pinMode(XP, INPUT);
  pinMode(XM, INPUT);
  delayMicroseconds(80);
  (void)analogRead(XP);
  return analogRead(XP);
}

// ---------- Touch detect ----------
bool touchedRaw(int xRaw, int yRaw) {
  const int MARGIN = 100;
  bool inX = (xRaw > Xmin - MARGIN) && (xRaw < Xmax + MARGIN);
  bool inY = (yRaw > Ymin - MARGIN) && (yRaw < Ymax + MARGIN);
  return (inX && inY);
}

// ---------- Axis mapping RAW → cm ----------
float mapAxis_cm(int vRaw, int vMin, int vMax, int vCenter, float halfRangeCm) {
  if (vRaw < vMin) vRaw = vMin;
  if (vRaw > vMax) vRaw = vMax;

  if (vCenter <= vMin || vCenter >= vMax) return 0.0f;
  if (vRaw == vCenter) return 0.0f;

  if (vRaw < vCenter) {
    float num = (float)(vRaw - vCenter);
    float den = (float)(vMin - vCenter);
    if (den == 0.0f) return 0.0f;
    float t = num / den;
    return -halfRangeCm * t;
  } else {
    float num = (float)(vRaw - vCenter);
    float den = (float)(vMax - vCenter);
    if (den == 0.0f) return 0.0f;
    float t = num / den;
    return +halfRangeCm * t;
  }
}

// ---------- PID step ----------
static float pid_step(PIDState &s,
                      float e_cm,
                      float de_dt_cm_s,
                      float Kp, float Ki, float Kd, float Kaw,
                      float u_min, float u_max,
                      float dt)
{
  float up = Kp * e_cm;
  float ui = s.i + Ki * e_cm * dt;
  float ud = Kd * de_dt_cm_s;
  float u  = up + ui + ud;

  float u_sat = clampf(u, u_min, u_max);

  float clamp_err = u_sat - u;
  s.i = ui + Kaw * clamp_err * dt;

  return u_sat;
}

// ---------- Status / Help ----------
void printStatus() {
  const char* modeName = "TARGET";
  if (g_mode == MODE_SQUARE)        modeName = "SQUARE";
  else if (g_mode == MODE_CIRCLE)   modeName = "CIRCLE";
  else if (g_mode == MODE_INFINITY) modeName = "INFINITY";

  Serial.printf(
    "STATUS:\n"
    "  Mode           : %s\n"
    "  Target (cm)    : X=%.2f Y=%.2f\n"
    "  ShapeFreq (Hz) : %.3f\n"
    "  Circle R (cm)  : %.2f\n"
    "  Square L (cm)  : %.2f (half-size)\n"
    "  Inf Amp X/Y    : %.2f / %.2f cm\n"
    "  PID X: Kp=%.3f Ki=%.3f Kd=%.3f\n"
    "  PID Y: Kp=%.3f Ki=%.3f Kd=%.3f\n"
    "  deadband       = %.3f cm\n"
    "  flatX=%.2f  flatY=%.2f\n"
    "  FC_VEL         = %.2f Hz\n"
    "  ALPHA_POS      = %.3f\n"
    "  usePosFilter   = %d\n"
    "  useVelFilter   = %d\n"
    "  limits: X[%.1f, %.1f]  Y[%.1f, %.1f]\n",
    modeName,
    g_targetX_cm, g_targetY_cm,
    shapeFreqHz,
    circleRadiusCm,
    squareHalfCm,
    infAmpX_cm, infAmpY_cm,
    Kp_x, Ki_x, Kd_x,
    Kp_y, Ki_y, Kd_y,
    EPS_ERR_CM,
    SERVO_X_FLAT, SERVO_Y_FLAT,
    FC_VEL,
    ALPHA_POS,
    g_usePosFilter ? 1 : 0,
    g_useVelFilter ? 1 : 0,
    SERVO_X_MIN, SERVO_X_MAX,
    SERVO_Y_MIN, SERVO_Y_MAX
  );
}

void printHelp() {
  Serial.println(
    "Commands:\n"
    "  kpx <val>\n"
    "  kix <val>\n"
    "  kdx <val>\n"
    "  kpy <val>\n"
    "  kiy <val>\n"
    "  kdy <val>\n"
    "  deadband <cm>\n"
    "  flatx <deg>\n"
    "  flaty <deg>\n"
    "  fcvel <Hz>\n"
    "  alphap <0-1>\n"
    "  usepos <0|1>\n"
    "  usevel <0|1>\n"
    "  mode target|square|circle|infinity\n"
    "  target <x_cm> <y_cm>\n"
    "  shape_f <Hz>\n"
    "  circle_r <cm>\n"
    "  square_h <cm>\n"
    "  infax <cm>\n"
    "  infay <cm>\n"
    "  s\n"
    "  cmd"
  );
}

// ------------ Command parser (shared for Serial + WebSocket) -----------
void handleCommandLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  String lower = line;
  lower.toLowerCase();

  int spaceIndex = lower.indexOf(' ');
  String cmd, arg;
  if (spaceIndex == -1) {
    cmd = lower;
    arg = "";
  } else {
    cmd = lower.substring(0, spaceIndex);
    arg = lower.substring(spaceIndex + 1);
  }

  float val = arg.toFloat();
  bool ok = true;

  if (cmd == "s")   { printStatus(); return; }
  if (cmd == "cmd") { printHelp();   return; }

  if      (cmd == "kpx") Kp_x = val;
  else if (cmd == "kix") Ki_x = val;
  else if (cmd == "kdx") Kd_x = val;
  else if (cmd == "kpy") Kp_y = val;
  else if (cmd == "kiy") Ki_y = val;
  else if (cmd == "kdy") Kd_y = val;

  else if (cmd == "deadband") {
    if (arg.length() == 0) ok = false;
    else { if (val < 0) val = 0; EPS_ERR_CM = val; }
  }
  else if (cmd == "flatx") {
    if (arg.length() == 0) ok = false;
    else { SERVO_X_FLAT = val; recomputeServoLimits(); }
  }
  else if (cmd == "flaty") {
    if (arg.length() == 0) ok = false;
    else { SERVO_Y_FLAT = val; recomputeServoLimits(); }
  }
  else if (cmd == "fcvel") {
    if (arg.length() == 0) ok = false;
    else { if (val < 1) val = 1; if (val > 200) val = 200; FC_VEL = val; }
  }
  else if (cmd == "alphap") {
    if (arg.length() == 0) ok = false;
    else { if (val < 0) val = 0; if (val > 1) val = 1; ALPHA_POS = val; }
  }
  else if (cmd == "usepos") {
    arg.trim();
    if (arg != "0" && arg != "1") ok = false;
    else g_usePosFilter = (arg == "1");
  }
  else if (cmd == "usevel") {
    arg.trim();
    if (arg != "0" && arg != "1") ok = false;
    else g_useVelFilter = (arg == "1");
  }

  else if (cmd == "mode") {
    arg.trim();
    if      (arg == "target")   g_mode = MODE_TARGET;
    else if (arg == "square")   g_mode = MODE_SQUARE;
    else if (arg == "circle")   g_mode = MODE_CIRCLE;
    else if (arg == "infinity" || arg == "inf") g_mode = MODE_INFINITY;
    else ok = false;

    if (ok) {
      modeStartMs = millis();
      pidX.i = 0.0f;
      pidY.i = 0.0f;
    }
  }
  else if (cmd == "target") {
    arg.trim();
    int sp = arg.indexOf(' ');
    if (sp <= 0) ok = false;
    else {
      float tx = arg.substring(0, sp).toFloat();
      float ty = arg.substring(sp + 1).toFloat();
      g_targetX_cm = tx;
      g_targetY_cm = ty;
    }
  }
  else if (cmd == "shape_f") {
    if (arg.length() == 0) ok = false;
    else { if (val < 0.01f) val = 0.01f; if (val > 2.0f) val = 2.0f; shapeFreqHz = val; }
  }
  else if (cmd == "circle_r") {
    if (arg.length() == 0) ok = false;
    else {
      float maxR = min(X_HALF_RANGE_CM, Y_HALF_RANGE_CM);
      if (val < 0.5f) val = 0.5f;
      if (val > maxR) val = maxR;
      circleRadiusCm = val;
    }
  }
  else if (cmd == "square_h") {
    if (arg.length() == 0) ok = false;
    else {
      float maxL = min(X_HALF_RANGE_CM, Y_HALF_RANGE_CM);
      if (val < 0.5f) val = 0.5f;
      if (val > maxL) val = maxL;
      squareHalfCm = val;
    }
  }
  else if (cmd == "infax") {
    if (arg.length() == 0) ok = false;
    else {
      float maxA = X_HALF_RANGE_CM;
      if (val < 0.5f) val = 0.5f;
      if (val > maxA) val = maxA;
      infAmpX_cm = val;
    }
  }
  else if (cmd == "infay") {
    if (arg.length() == 0) ok = false;
    else {
      float maxA = Y_HALF_RANGE_CM;
      if (val < 0.5f) val = 0.5f;
      if (val > maxA) val = maxA;
      infAmpY_cm = val;
    }
  }
  else ok = false;

  if (ok) printStatus();
  else    printHelp();
}

// ------------ Serial Commands Wrapper --------------
void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  handleCommandLine(line);
}

// ---------- WebSocket event ----------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type != WStype_TEXT) return;
  char tmp[256];
  size_t n = (length < sizeof(tmp)-1) ? length : (sizeof(tmp)-1);
  memcpy(tmp, payload, n);
  tmp[n] = '\0';
  handleCommandLine(String(tmp));
}

// ---------- HTTP handler ----------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// ---------- Setup ----------
void setup() {
  pinMode(PROBE, OUTPUT);
  digitalWrite(PROBE, LOW);

  Serial.begin(921600);
  delay(200);
  Serial.println("\n=== Ball Balancing Table: Touch → PID → Servos + WiFi Dashboard ===");

  recomputeServoLimits();

  servoX.setPeriodHertz(SERVO_HZ);
  servoY.setPeriodHertz(SERVO_HZ);
  servoX.attach(SERVO_X_PIN, US_MIN, US_MAX);
  servoY.attach(SERVO_Y_PIN, US_MIN, US_MAX);

  servoX.write((int)roundf(SERVO_X_FLAT));
  servoY.write((int)roundf(SERVO_Y_FLAT));

  last_us = micros();
  modeStartMs = millis();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.begin();

  webSocket.begin();
  webSocket.enableHeartbeat(15000, 3000, 2);
  webSocket.onEvent(webSocketEvent);

  Serial.println("HTTP + WebSocket ready.");
}

// ---------- Main Loop ----------
void loop() {
  server.handleClient();
  webSocket.loop();
  handleSerialCommands();

  uint32_t now = micros();
  uint32_t du  = now - last_us;

  if (du >= CTRL_DT_US) {
    last_us = now;

    float dt = du * 1e-6f;
    if (dt < 1e-4f) dt = 1e-4f;
    if (dt > 0.05f) dt = 0.05f;

    digitalWrite(PROBE, HIGH);

    int xRaw = readX_raw();
    int yRaw = readY_raw();
    bool isTouched = touchedRaw(xRaw, yRaw);

    static bool have_raw = false;
    static bool have_filt = false;

    static float x_raw_last = 0.0f, y_raw_last = 0.0f;
    static float x_filt_last = 0.0f, y_filt_last = 0.0f;

    static float x_filt = 0.0f, y_filt = 0.0f;

    static float vx_lp_raw = 0.0f, vy_lp_raw = 0.0f;
    static float vx_lp_filt = 0.0f, vy_lp_filt = 0.0f;

    float x_cm = 0.0f, y_cm = 0.0f;

    float vx_raw_raw = 0.0f, vy_raw_raw = 0.0f;
    float vx_raw_filt = 0.0f, vy_raw_filt = 0.0f;

    if (isTouched) {
      x_cm = mapAxis_cm(xRaw, Xmin, Xmax, X_CENTER_RAW, X_HALF_RANGE_CM);
      float y_tmp = mapAxis_cm(yRaw, Ymin, Ymax, Y_CENTER_RAW, Y_HALF_RANGE_CM);
      y_cm = -y_tmp;

      // Always compute position-filtered signal (for plotting + optional control)
      float a = ALPHA_POS;
      if (a < 0.0f) a = 0.0f;
      if (a > 1.0f) a = 1.0f;

      if (!have_filt) {
        x_filt = x_cm;
        y_filt = y_cm;
        have_filt = true;
      } else {
        x_filt = x_filt + a * (x_cm - x_filt);
        y_filt = y_filt + a * (y_cm - y_filt);
      }

      // Raw velocity from raw position
      if (have_raw) {
        vx_raw_raw = (x_cm - x_raw_last) / dt;
        vy_raw_raw = (y_cm - y_raw_last) / dt;
      }
      x_raw_last = x_cm;
      y_raw_last = y_cm;
      have_raw = true;

      // Raw velocity from filtered position
      if (have_filt) {
        vx_raw_filt = (x_filt - x_filt_last) / dt;
        vy_raw_filt = (y_filt - y_filt_last) / dt;
      }
      x_filt_last = x_filt;
      y_filt_last = y_filt;

      yield();
    } else {
      x_cm = 0.0f;
      y_cm = 0.0f;
      x_filt = 0.0f;
      y_filt = 0.0f;

      vx_raw_raw  = 0.0f; vy_raw_raw  = 0.0f;
      vx_raw_filt = 0.0f; vy_raw_filt = 0.0f;

      have_raw = false;
      have_filt = false;
    }

    vx_raw_raw  = clampf(vx_raw_raw,  -VEL_RAW_CLIP, VEL_RAW_CLIP);
    vy_raw_raw  = clampf(vy_raw_raw,  -VEL_RAW_CLIP, VEL_RAW_CLIP);
    vx_raw_filt = clampf(vx_raw_filt, -VEL_RAW_CLIP, VEL_RAW_CLIP);
    vy_raw_filt = clampf(vy_raw_filt, -VEL_RAW_CLIP, VEL_RAW_CLIP);

    // Always compute vel-filtered signals
    float Av = alpha_vel(dt);
    vx_lp_raw  += Av * (vx_raw_raw  - vx_lp_raw);
    vy_lp_raw  += Av * (vy_raw_raw  - vy_lp_raw);
    vx_lp_filt += Av * (vx_raw_filt - vx_lp_filt);
    vy_lp_filt += Av * (vy_raw_filt - vy_lp_filt);

    float vx_used = 0.0f, vy_used = 0.0f;
    float x_used  = 0.0f, y_used  = 0.0f;

    // Default: raw in calculations, unless toggles enabled
    x_used = g_usePosFilter ? x_filt : x_cm;
    y_used = g_usePosFilter ? y_filt : y_cm;

    if (g_useVelFilter) {
      if (g_usePosFilter) { vx_used = vx_lp_filt; vy_used = vy_lp_filt; }
      else                { vx_used = vx_lp_raw;  vy_used = vy_lp_raw;  }
    } else {
      if (g_usePosFilter) { vx_used = vx_raw_filt; vy_used = vy_raw_filt; }
      else                { vx_used = vx_raw_raw;  vy_used = vy_raw_raw;  }
    }

    float vx = clampf(vx_used, -VEL_CLIP, VEL_CLIP);
    float vy = clampf(vy_used, -VEL_CLIP, VEL_CLIP);

    // ---------- choose target tx, ty based on mode ----------
    float tx = 0.0f;
    float ty = 0.0f;

    uint32_t msCtrl = millis();
    float t_mode_s = (msCtrl - modeStartMs) * 0.001f;

    float f = shapeFreqHz;
    if (f < 0.01f) f = 0.01f;
    if (f > 2.0f)  f = 2.0f;
    float T = 1.0f / f;
    float omega = 2.0f * (float)M_PI * f;

    if (g_mode == MODE_TARGET) {
      tx = g_targetX_cm;
      ty = g_targetY_cm;
    }
    else if (g_mode == MODE_CIRCLE) {
      float theta = omega * t_mode_s;
      tx = circleRadiusCm * cosf(theta);
      ty = circleRadiusCm * sinf(theta);
    }
    else if (g_mode == MODE_SQUARE) {
      float phase = fmodf(t_mode_s, T) / T;
      float L = squareHalfCm;
      float seg = 0.25f;
      if (phase < seg) {
        float p = phase / seg;
        tx = -L + 2.0f*L*p;  ty = -L;
      } else if (phase < 2*seg) {
        float p = (phase - seg) / seg;
        tx = +L;             ty = -L + 2.0f*L*p;
      } else if (phase < 3*seg) {
        float p = (phase - 2*seg) / seg;
        tx = +L - 2.0f*L*p;  ty = +L;
      } else {
        float p = (phase - 3*seg) / seg;
        tx = -L;             ty = +L - 2.0f*L*p;
      }
    }
    else if (g_mode == MODE_INFINITY) {
      float x0 = infAmpX_cm * sinf(omega * t_mode_s);
      float y0 = infAmpY_cm * sinf(2.0f * omega * t_mode_s);
      tx = -y0;  // rotate 90 deg
      ty =  x0;
    }

    float ex = tx - x_used;
    float ey = ty - y_used;

    if (fabsf(ex) < EPS_ERR_CM) ex = 0.0f;
    if (fabsf(ey) < EPS_ERR_CM) ey = 0.0f;

    float dex = -vx;
    float dey = -vy;

    if (SERVO_INVERT_X) { ex = -ex; dex = -dex; }
    if (SERVO_INVERT_Y) { ey = -ey; dey = -dey; }

    float u_x_deg = 0.0f;
    float u_y_deg = 0.0f;

    if (isTouched && (have_raw || have_filt)) {
      u_x_deg = pid_step(pidX, ex, dex, Kp_x, Ki_x, Kd_x, Kaw_x, -SERVO_SPAN, SERVO_SPAN, dt);
      u_y_deg = pid_step(pidY, ey, dey, Kp_y, Ki_y, Kd_y, Kaw_y, -SERVO_SPAN, SERVO_SPAN, dt);
    } else {
      pidX.i *= 0.98f;
      pidY.i *= 0.98f;
      u_x_deg = 0.0f;
      u_y_deg = 0.0f;
    }

    float servoX_deg = SERVO_X_FLAT + u_x_deg;
    float servoY_deg = SERVO_Y_FLAT + u_y_deg;

    servoX_deg = clampf(servoX_deg, SERVO_X_MIN, SERVO_X_MAX);
    servoY_deg = clampf(servoY_deg, SERVO_Y_MIN, SERVO_Y_MAX);

    servoX.write((int)roundf(servoX_deg));
    servoY.write((int)roundf(servoY_deg));

    // Push to globals for dashboard
    g_x_raw  = x_cm;
    g_y_raw  = y_cm;
    g_x_filt = x_filt;
    g_y_filt = y_filt;
    g_servoX = servoX_deg;
    g_servoY = servoY_deg;

    digitalWrite(PROBE, LOW);

    static uint32_t last_print_ms = 0;
    uint32_t ms = millis();
    if (isTouched && (ms - last_print_ms >= 50)) {
      last_print_ms = ms;
      Serial.printf(
        "dt=%.4f  x=%.2f y=%.2f  xf=%.2f yf=%.2f  vx=%.2f vy=%.2f  tx=%.2f ty=%.2f  ex=%.2f ey=%.2f  uX=%.1f uY=%.1f  servoX=%.1f servoY=%.1f\n",
        dt, x_cm, y_cm, x_filt, y_filt, vx, vy, tx, ty, ex, ey, u_x_deg, u_y_deg, servoX_deg, servoY_deg
      );
    }
  }

  // WebSocket broadcast at ~20 Hz
  static uint32_t lastWsMs = 0;
  uint32_t msNow = millis();
  if (msNow - lastWsMs >= 50) {
    lastWsMs = msNow;

    float t  = msNow * 0.001f;
    float xr = g_x_raw;
    float yr = g_y_raw;
    float xf = g_x_filt;
    float yf = g_y_filt;

    char buf[200];
    snprintf(buf, sizeof(buf),
             "{\"t\":%.3f,\"xr\":%.3f,\"yr\":%.3f,\"xf\":%.3f,\"yf\":%.3f}",
             t, xr, yr, xf, yf);
    webSocket.broadcastTXT(buf);
  }
}

// ================== HTML DASHBOARD ==================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Ball Balancing Table Dashboard</title>
<style>
  body { font-family: Arial, sans-serif; margin:0; padding:0; background:#111; color:#eee; }
  .container { max-width:960px; margin:0 auto; padding:10px; }
  .header { display:flex; justify-content:space-between; align-items:center; background:#222; padding:8px 12px; border-radius:4px; margin-bottom:10px; }
  .ws-status { font-size:12px; }
  .block { background:#1b1b1b; margin-bottom:10px; padding:10px; border-radius:6px; }
  .block h3 { margin-top:0; margin-bottom:8px; font-size:16px; }
  .row-inline { display:flex; flex-wrap:wrap; gap:10px; align-items:center; }
  .label { font-size:12px; color:#aaa; }
  .value { font-size:14px; }
  input[type=range] { width:200px; }
  input[type=number] { width:80px; margin-left:4px; background:#111; color:#eee; border:1px solid #444; border-radius:3px; padding:2px 4px; }
  canvas { background:#000; border:1px solid #444; border-radius:4px; width:100%; max-width:900px; }
  .pid-group { margin-bottom:10px; }
  .pid-group h4 { margin:4px 0; font-size:14px; }
  .pid-row { display:flex; align-items:center; flex-wrap:wrap; gap:6px; margin-bottom:4px; }
  .pid-row .label { min-width:90px; }
  .stats-text { font-family: monospace; font-size: 12px; white-space: pre; }
  button { background:#2979ff; color:#fff; border:none; border-radius:4px; padding:6px 12px; cursor:pointer; font-size:13px; }
  button:hover { background:#1c54b2; }
  .pill { display:inline-flex; align-items:center; gap:6px; padding:3px 8px; border:1px solid #333; border-radius:999px; background:#141414; }
  .dot { width:10px; height:10px; border-radius:50%; display:inline-block; }
</style>
</head>
<body>
<div class="container">

  <div class="header">
    <div>Ball Balancing Table Live Dashboard</div>
    <div class="ws-status">
      WebSocket: <span id="wsStatus" style="color:#f55;">DISCONNECTED</span>
    </div>
  </div>

  <div class="block" id="block-mode">
    <h3>Control Mode, Target & Shape Settings</h3>
    <div class="row-inline">
      <label><input type="radio" name="mode" value="target" checked onchange="onModeChange(this.value)"> Target point</label>
      <label><input type="radio" name="mode" value="square" onchange="onModeChange(this.value)"> Square path</label>
      <label><input type="radio" name="mode" value="circle" onchange="onModeChange(this.value)"> Circle path</label>
      <label><input type="radio" name="mode" value="infinity" onchange="onModeChange(this.value)"> Infinity (∞) path</label>
    </div>
    <div style="height:8px;"></div>
    <div class="row-inline">
      <div class="label">Target X (cm):</div>
      <input type="number" id="targetX" step="0.1" value="0.0">
      <div class="label">Target Y (cm):</div>
      <input type="number" id="targetY" step="0.1" value="0.0">
      <button onclick="applyTarget()">Set Target</button>
    </div>
    <div style="height:8px;"></div>
    <div class="row-inline">
      <div class="label">Shape frequency (Hz):</div>
      <input type="number" id="shapeHz" step="0.01" value="0.10">
    </div>
    <div class="row-inline">
      <div class="label">Circle radius (cm):</div>
      <input type="number" id="circleR" step="0.1" value="3.0">
      <div class="label">Square half-size L (cm):</div>
      <input type="number" id="squareH" step="0.1" value="3.0">
    </div>
    <div class="row-inline">
      <div class="label">Infinity Amp X (cm):</div>
      <input type="number" id="infAx" step="0.1" value="3.0">
      <div class="label">Infinity Amp Y (cm):</div>
      <input type="number" id="infAy" step="0.1" value="3.0">
      <button onclick="applyShapeParams()">Apply Shape Params</button>
    </div>
  </div>

  <div class="block" id="block-settings">
    <h3>Plot Settings & Last Sample</h3>
    <div class="row-inline">
      <div>
        <div class="label">Last sample:</div>
        <div class="value" id="lastVals">t=0.000s</div>
      </div>
    </div>
    <div style="height:6px;"></div>

    <div class="row-inline">
      <div class="label">Time window (s):</div>
      <input type="range" id="winSlider" min="2" max="40" step="1" value="20" oninput="onWinChange(this.value)">
      <div class="value" id="winLabel">20 s</div>
    </div>
    <div class="row-inline">
      <div class="label">Y range (±cm):</div>
      <input type="range" id="yrSlider" min="0.5" max="13" step="0.5" value="13" oninput="onYRangeChange(this.value)">
      <div class="value" id="yrLabel">±13 cm</div>
    </div>

    <div style="height:8px;"></div>
    <div class="row-inline">
      <div class="pill"><span class="dot" style="background:#00e5ff;"></span><label><input type="checkbox" id="showXR" checked> X (raw)</label></div>
      <div class="pill"><span class="dot" style="background:#ffff00;"></span><label><input type="checkbox" id="showYR" checked> Y (raw)</label></div>
      <div class="pill"><span class="dot" style="background:#7CFC00;"></span><label><input type="checkbox" id="showXF" checked> X (filtered)</label></div>
      <div class="pill"><span class="dot" style="background:#ff00ff;"></span><label><input type="checkbox" id="showYF" checked> Y (filtered)</label></div>
    </div>
  </div>

  <div class="block" id="block-pos">
    <h3>Position vs Time</h3>
    <canvas id="posCanvas" width="900" height="320"></canvas>
    <div class="label" style="margin-top:4px;">
      Y-axis in cm (grid step = 0.5 cm). Horizontal line at 0 cm.
    </div>
  </div>

  <div class="block" id="block-pid">
    <h3>PID Tuning</h3>

    <div class="pid-group">
      <h4>X Axis PID</h4>
      <div class="pid-row">
        <span class="label">Kp_x</span>
        <input type="range" min="0" max="10" step="0.1" value="3.0" id="kpx" oninput="onParamSlider('kpx')">
        <input type="number" step="0.01" id="kpx_val" value="3.0" onchange="onParamInput('kpx')">
      </div>
      <div class="pid-row">
        <span class="label">Ki_x</span>
        <input type="range" min="0" max="5" step="0.01" value="0.0" id="kix" oninput="onParamSlider('kix')">
        <input type="number" step="0.001" id="kix_val" value="0.0" onchange="onParamInput('kix')">
      </div>
      <div class="pid-row">
        <span class="label">Kd_x</span>
        <input type="range" min="0" max="10" step="0.1" value="1.1" id="kdx" oninput="onParamSlider('kdx')">
        <input type="number" step="0.01" id="kdx_val" value="1.1" onchange="onParamInput('kdx')">
      </div>
    </div>

    <div class="pid-group">
      <h4>Y Axis PID</h4>
      <div class="pid-row">
        <span class="label">Kp_y</span>
        <input type="range" min="0" max="10" step="0.1" value="3.0" id="kpy" oninput="onParamSlider('kpy')">
        <input type="number" step="0.01" id="kpy_val" value="3.0" onchange="onParamInput('kpy')">
      </div>
      <div class="pid-row">
        <span class="label">Ki_y</span>
        <input type="range" min="0" max="5" step="0.01" value="0.0" id="kiy" oninput="onParamSlider('kiy')">
        <input type="number" step="0.001" id="kiy_val" value="0.0" onchange="onParamInput('kiy')">
      </div>
      <div class="pid-row">
        <span class="label">Kd_y</span>
        <input type="range" min="0" max="10" step="0.1" value="1.1" id="kdy" oninput="onParamSlider('kdy')">
        <input type="number" step="0.01" id="kdy_val" value="1.1" onchange="onParamInput('kdy')">
      </div>
    </div>

    <div class="pid-group">
      <h4>Other Parameters</h4>
      <div class="pid-row">
        <span class="label">deadband (cm)</span>
        <input type="range" min="0" max="2" step="0.01" value="0.2" id="deadband" oninput="onParamSlider('deadband')">
        <input type="number" step="0.01" id="deadband_val" value="0.2" onchange="onParamInput('deadband')">
      </div>
      <div class="pid-row">
        <span class="label">ALPHA_POS</span>
        <input type="range" min="0" max="1" step="0.01" value="1.0" id="alphap" oninput="onParamSlider('alphap')">
        <input type="number" step="0.01" id="alphap_val" value="1.0" onchange="onParamInput('alphap')">
        <label class="pill" style="margin-left:8px;">
          <input type="checkbox" id="usePos" onchange="onUseToggle()"> use in control
        </label>
      </div>
      <div class="pid-row">
        <span class="label">FC_VEL (Hz)</span>
        <input type="range" min="1" max="100" step="1" value="40" id="fcvel" oninput="onParamSlider('fcvel')">
        <input type="number" step="1" id="fcvel_val" value="40" onchange="onParamInput('fcvel')">
        <label class="pill" style="margin-left:8px;">
          <input type="checkbox" id="useVel" onchange="onUseToggle()"> use in control
        </label>
      </div>
      <div class="pid-row">
        <span class="label">flatX (deg)</span>
        <input type="range" min="40" max="120" step="0.1" value="79" id="flatx" oninput="onParamSlider('flatx')">
        <input type="number" step="0.1" id="flatx_val" value="79" onchange="onParamInput('flatx')">
      </div>
      <div class="pid-row">
        <span class="label">flatY (deg)</span>
        <input type="range" min="40" max="120" step="0.1" value="94" id="flaty" oninput="onParamSlider('flaty')">
        <input type="number" step="0.1" id="flaty_val" value="94" onchange="onParamInput('flaty')">
      </div>
    </div>

    <div style="text-align:right; margin-top:8px;">
      <button id="applyBtn">Apply all parameters</button>
    </div>
  </div>

  <div class="block" id="block-fft">
    <h3>Frequency Analysis & Error Stats</h3>

    <div class="row-inline">
      <div class="label">FFT window (s):</div>
      <input type="range" id="fftWinSlider" min="2" max="20" step="1" value="10" oninput="onFFTWindowChange(this.value)">
      <div class="value" id="fftWinLabel">10 s</div>
    </div>
    <div class="row-inline">
      <div class="label">FFT max frequency (Hz):</div>
      <input type="range" id="fmaxSlider" min="1" max="5" step="1" value="5" oninput="onFmaxChange(this.value)">
      <div class="value" id="fmaxLabel">5 Hz</div>
    </div>

    <div style="height:8px;"></div>
    <div class="row-inline">
      <div class="pill"><span class="dot" style="background:#00e5ff;"></span><label><input type="checkbox" id="fftXR" checked> X (raw)</label></div>
      <div class="pill"><span class="dot" style="background:#ffff00;"></span><label><input type="checkbox" id="fftYR" checked> Y (raw)</label></div>
      <div class="pill"><span class="dot" style="background:#7CFC00;"></span><label><input type="checkbox" id="fftXF" checked> X (filtered)</label></div>
      <div class="pill"><span class="dot" style="background:#ff00ff;"></span><label><input type="checkbox" id="fftYF" checked> Y (filtered)</label></div>
    </div>

    <div style="height:6px;"></div>
    <canvas id="fftCanvas" width="900" height="260"></canvas>
    <div class="stats-text" id="fftInfo">Waiting for data...</div>
  </div>

</div>

<script>
let ws = null;

let tBuf  = [];
let xrBuf = [];
let yrBuf = [];
let xfBuf = [];
let yfBuf = [];

const MAX_SAMPLES = 500;

let windowSec = 20;
let yRange    = 13;
let fftFmax   = 5;
let fftWindowSec = 10;

function onWinChange(v) {
  windowSec = parseFloat(v);
  document.getElementById('winLabel').textContent = v + " s";
}
function onYRangeChange(v) {
  yRange = parseFloat(v);
  document.getElementById('yrLabel').textContent = "±" + v + " cm";
}
function onFmaxChange(v) {
  fftFmax = parseFloat(v);
  document.getElementById('fmaxLabel').textContent = v + " Hz";
}
function onFFTWindowChange(v) {
  fftWindowSec = parseFloat(v);
  document.getElementById('fftWinLabel').textContent = v + " s";
}

function connectWS() {
  const url = "ws://192.168.4.1:81/";
  console.log("Connecting WS to:", url);

  ws = new WebSocket(url);

  ws.onopen = () => {
    console.log("WS open");
    const el = document.getElementById('wsStatus');
    el.textContent = "CONNECTED";
    el.style.color = "#4caf50";
  };
  ws.onclose = () => {
    console.log("WS close");
    const el = document.getElementById('wsStatus');
    el.textContent = "DISCONNECTED";
    el.style.color = "#f55";
    setTimeout(connectWS, 2000);
  };
  ws.onerror = (e) => {
    console.log("WS error", e);
  };

  ws.onmessage = (ev) => {
    try {
      const d = JSON.parse(ev.data);
      const t  = d.t;
      const xr = d.xr;
      const yr = d.yr;
      const xf = d.xf;
      const yf = d.yf;

      tBuf.push(t);
      xrBuf.push(xr);
      yrBuf.push(yr);
      xfBuf.push(xf);
      yfBuf.push(yf);

      while (tBuf.length > MAX_SAMPLES) {
        tBuf.shift();
        xrBuf.shift();
        yrBuf.shift();
        xfBuf.shift();
        yfBuf.shift();
      }

      document.getElementById('lastVals').textContent =
        `t=${t.toFixed(3)}s  X(raw)=${xr.toFixed(2)}  Y(raw)=${yr.toFixed(2)}  X(filt)=${xf.toFixed(2)}  Y(filt)=${yf.toFixed(2)}`;

    } catch (e) {
      console.log("WS message parse error", e);
    }
  };
}

function sendRaw(msg) {
  if (!ws || ws.readyState !== 1) return;
  ws.send(msg);
}

function sendParam(name, val) {
  if (!ws || ws.readyState !== 1) return;
  if (val === undefined || val === null || val === "") sendRaw(name);
  else sendRaw(name + " " + val);
}

function onModeChange(val) { sendRaw("mode " + val); }

function applyTarget() {
  const tx = parseFloat(document.getElementById('targetX').value);
  const ty = parseFloat(document.getElementById('targetY').value);
  if (isNaN(tx) || isNaN(ty)) return;
  sendRaw("target " + tx + " " + ty);
}

function applyShapeParams() {
  const hz = parseFloat(document.getElementById('shapeHz').value);
  const r  = parseFloat(document.getElementById('circleR').value);
  const s  = parseFloat(document.getElementById('squareH').value);
  const ax = parseFloat(document.getElementById('infAx').value);
  const ay = parseFloat(document.getElementById('infAy').value);

  if (!isNaN(hz)) sendRaw("shape_f " + hz);
  if (!isNaN(r))  sendRaw("circle_r " + r);
  if (!isNaN(s))  sendRaw("square_h " + s);
  if (!isNaN(ax)) sendRaw("infax " + ax);
  if (!isNaN(ay)) sendRaw("infay " + ay);
}

function onParamSlider(name) {
  const slider = document.getElementById(name);
  const box    = document.getElementById(name + "_val");
  const v = parseFloat(slider.value);
  box.value = v;
  sendParam(name, v);
}

function onParamInput(name) {
  const slider = document.getElementById(name);
  const box    = document.getElementById(name + "_val");
  let v = parseFloat(box.value);
  if (isNaN(v)) return;
  slider.value = v;
  sendParam(name, v);
}

function onUseToggle() {
  const usePos = document.getElementById('usePos').checked ? 1 : 0;
  const useVel = document.getElementById('useVel').checked ? 1 : 0;
  sendRaw("usepos " + usePos);
  sendRaw("usevel " + useVel);
  sendRaw("s");
}

document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('applyBtn').onclick = () => {
    const names = ["kpx","kix","kdx","kpy","kiy","kdy","deadband","alphap","fcvel","flatx","flaty"];
    for (const n of names) {
      const box = document.getElementById(n + "_val");
      if (!box) continue;
      const v = parseFloat(box.value);
      if (!isNaN(v)) sendParam(n, v);
    }
    onUseToggle();
  };
});

// ===== POSITION PLOT =====
function drawPosition() {
  const canvas = document.getElementById('posCanvas');
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;

  ctx.fillStyle = "#000";
  ctx.fillRect(0,0,w,h);

  if (tBuf.length < 2) return;

  const tEnd   = tBuf[tBuf.length - 1];
  const tStart = Math.max(tEnd - windowSec, tBuf[0]);

  const yMin   = -yRange;
  const yMax   =  yRange;

  function tx(t) {
    const den = (tEnd - tStart) || 1;
    return (t - tStart) / den * w;
  }
  function ty(y) {
    const den = (yMax - yMin) || 1;
    return h - (y - yMin) / den * h;
  }

  ctx.strokeStyle = "#222";
  ctx.lineWidth = 1;
  const step = 0.5;
  for (let v = yMin; v <= yMax + 1e-6; v += step) {
    const yPix = ty(v);
    ctx.beginPath();
    ctx.moveTo(0, yPix);
    ctx.lineTo(w, yPix);
    ctx.stroke();

    if (Math.abs(v - Math.round(v)) < 1e-6) {
      ctx.fillStyle = "#888";
      ctx.font = "11px Arial";
      ctx.fillText(v.toFixed(0), 2, yPix - 2);
    }
  }

  const y0 = ty(0);
  ctx.strokeStyle = "#777777";
  ctx.lineWidth = 1;             
  ctx.beginPath();
  ctx.moveTo(0, y0);
  ctx.lineTo(w, y0);
  ctx.stroke();


  ctx.fillStyle = "#aaa";
  ctx.font = "11px Arial";
  const ticks = 5;
  for (let i = 0; i <= ticks; i++) {
    const tt = tStart + (tEnd - tStart) * (i / ticks);
    const x = tx(tt);
    const label = (tt - tStart).toFixed(1);
    ctx.fillText(label, x - 8, h - 4);
  }

  const showXR = document.getElementById('showXR').checked;
  const showYR = document.getElementById('showYR').checked;
  const showXF = document.getElementById('showXF').checked;
  const showYF = document.getElementById('showYF').checked;

  function plotLine(buf, color, lw) {
    ctx.strokeStyle = color;
    ctx.lineWidth = lw;
    ctx.beginPath();
    let first = true;
    for (let i = 0; i < tBuf.length; i++) {
      if (tBuf[i] < tStart) continue;
      const x = tx(tBuf[i]);
      const y = ty(buf[i]);
      if (first) { ctx.moveTo(x,y); first = false; }
      else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }

  if (showXR) plotLine(xrBuf, "#00e5ff", 1.8);
  if (showYR) plotLine(yrBuf, "#ffff00", 1.8);
  if (showXF) plotLine(xfBuf, "#7CFC00", 1.8);
  if (showYF) plotLine(yfBuf, "#ff00ff", 1.8);
}

// ===== FFT + STATS =====
function drawFFT() {
  const canvas = document.getElementById('fftCanvas');
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  const infoEl = document.getElementById('fftInfo');

  ctx.fillStyle = "#000";
  ctx.fillRect(0,0,w,h);

  if (tBuf.length < 32) {
    infoEl.textContent = "Not enough data yet (need >= 32 samples).";
    return;
  }

  const tLast = tBuf[tBuf.length - 1];
  const tStart = tLast - fftWindowSec;

  let startIdx = 0;
  for (let i = 0; i < tBuf.length; i++) {
    if (tBuf[i] >= tStart) { startIdx = i; break; }
  }

  let tSeg  = tBuf.slice(startIdx);
  let xrSeg = xrBuf.slice(startIdx);
  let yrSeg = yrBuf.slice(startIdx);
  let xfSeg = xfBuf.slice(startIdx);
  let yfSeg = yfBuf.slice(startIdx);

  let N = tSeg.length;
  if (N < 32) {
    infoEl.textContent = "Not enough data in window.";
    return;
  }

  if (N > 256) {
    tSeg  = tSeg.slice(N - 256);
    xrSeg = xrSeg.slice(N - 256);
    yrSeg = yrSeg.slice(N - 256);
    xfSeg = xfSeg.slice(N - 256);
    yfSeg = yfSeg.slice(N - 256);
    N = tSeg.length;
  }

  // sampling from timestamps
  let dt = 0;
  for (let i = 1; i < N; i++) dt += (tSeg[i] - tSeg[i-1]);
  dt /= (N - 1);
  if (dt <= 0) { infoEl.textContent = "Invalid dt for FFT."; return; }
  const fs = 1.0 / dt;
  const fRes = fs / N;

  function hann(n) { return 0.5 - 0.5 * Math.cos(2*Math.PI*n/(N-1)); }

  function dftMag(sig) {
    const half = Math.floor(N/2);
    const mags = new Array(half).fill(0);

    for (let k = 0; k < half; k++) {
      let re = 0, im = 0;
      const ang = -2 * Math.PI * k / N;
      for (let n = 0; n < N; n++) {
        const phi = ang * n;
        const c = Math.cos(phi);
        const s = Math.sin(phi);
        const v = sig[n] * hann(n);
        re += v * c;
        im += v * s;
      }
      mags[k] = Math.sqrt(re*re + im*im);
    }
    return mags;
  }

  function topTwoPeaks(mags) {
    let best1 = {k: 1, val: mags[1] || 0};
    let best2 = {k: 2, val: mags[2] || 0};
    if (best2.val > best1.val) { const tmp = best1; best1 = best2; best2 = tmp; }
    for (let k = 3; k < mags.length; k++) {
      const v = mags[k];
      if (v > best1.val) { best2 = best1; best1 = {k, val:v}; }
      else if (v > best2.val && k !== best1.k) best2 = {k, val:v};
    }
    return [best1, best2];
  }

  // Stats (based on filtered by default)
  let sumX2 = 0, sumY2 = 0, sumAbsX = 0, sumAbsY = 0, maxAbsX = 0, maxAbsY = 0;
  for (let i = 0; i < N; i++) {
    const x = xfSeg[i];
    const y = yfSeg[i];
    const ax = Math.abs(x), ay = Math.abs(y);
    sumX2 += x*x; sumY2 += y*y;
    sumAbsX += ax; sumAbsY += ay;
    if (ax > maxAbsX) maxAbsX = ax;
    if (ay > maxAbsY) maxAbsY = ay;
  }
  const rmsX = Math.sqrt(sumX2 / N);
  const rmsY = Math.sqrt(sumY2 / N);
  const meanAbsX = sumAbsX / N;
  const meanAbsY = sumAbsY / N;

  const showXR = document.getElementById('fftXR').checked;
  const showYR = document.getElementById('fftYR').checked;
  const showXF = document.getElementById('fftXF').checked;
  const showYF = document.getElementById('fftYF').checked;

  const half = Math.floor(N/2);

  let magsXR=null, magsYR=null, magsXF=null, magsYF=null;
  if (showXR) magsXR = dftMag(xrSeg);
  if (showYR) magsYR = dftMag(yrSeg);
  if (showXF) magsXF = dftMag(xfSeg);
  if (showYF) magsYF = dftMag(yfSeg);

  // peak info always from FILTERED 
  const magsXF_peaks = dftMag(xfSeg);
  const magsYF_peaks = dftMag(yfSeg);
  
  let peaksText = "";
  {
    const [p1, p2] = topTwoPeaks(magsXF_peaks);
    peaksText += `   X peaks: f1≈${(p1.k*fRes).toFixed(2)} Hz, f2≈${(p2.k*fRes).toFixed(2)} Hz\n`;
  }
  {
    const [p1, p2] = topTwoPeaks(magsYF_peaks);
    peaksText += `   Y peaks: f1≈${(p1.k*fRes).toFixed(2)} Hz, f2≈${(p2.k*fRes).toFixed(2)} Hz\n`;
  }

  infoEl.textContent =
    `Window: N=${N}, fs≈${fs.toFixed(1)} Hz, T≈${(N*dt).toFixed(2)} s\n` +
    `Stats (filtered):\n` +
    `   X RMS=${rmsX.toFixed(3)} cm, max=${maxAbsX.toFixed(3)} cm, mean|X|=${meanAbsX.toFixed(3)} cm\n` +
    `   Y RMS=${rmsY.toFixed(3)} cm, max=${maxAbsY.toFixed(3)} cm, mean|Y|=${meanAbsY.toFixed(3)} cm\n` +
    peaksText +
    `\n` +
    `N = number of samples used in FFT\n` +
    `fs = sampling frequency (samples/second)\n` +
    `T ≈ N / fs = time window length`;

  // Draw axes
  ctx.strokeStyle = "#333";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(40, 10);
  ctx.lineTo(40, h-25);
  ctx.lineTo(w-10, h-25);
  ctx.stroke();

  ctx.fillStyle = "#aaa";
  ctx.font = "11px Arial";
  ctx.fillText("Freq (Hz)", w/2 - 25, h-8);

  const plotX0 = 40;
  const plotY0 = 10;
  const plotW  = w - 50;
  const plotH  = h - 35;

  const fMax = fftFmax;

  function maxOf(m) { return m ? Math.max(...m) : 0; }
  const maxMag = Math.max(maxOf(magsXR), maxOf(magsYR), maxOf(magsXF), maxOf(magsYF)) || 1;

  // Grid
  ctx.strokeStyle = "#222";
  const xTicks = 5;
  for (let i = 0; i <= xTicks; i++) {
    const f = fMax * i / xTicks;
    const x = plotX0 + (f / fMax) * plotW;
    ctx.beginPath();
    ctx.moveTo(x, plotY0);
    ctx.lineTo(x, plotY0 + plotH);
    ctx.stroke();
    ctx.fillStyle = "#888";
    ctx.fillText(f.toFixed(1), x - 8, plotY0 + plotH + 12);
  }
  for (let j = 0; j <= 4; j++) {
    const frac = j / 4;
    const yPix = plotY0 + plotH - frac * plotH;
    ctx.strokeStyle = "#222";
    ctx.beginPath();
    ctx.moveTo(plotX0, yPix);
    ctx.lineTo(plotX0 + plotW, yPix);
    ctx.stroke();
  }

  function plotFFTLine(mags, color) {
    if (!mags) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    let first = true;
    for (let k = 1; k < half; k++) {
      const f = k * fRes;
      if (f > fMax) break;
      const normMag = mags[k] / maxMag;
      const x = plotX0 + (f / fMax) * plotW;
      const y = plotY0 + plotH - normMag * plotH;
      if (first) { ctx.moveTo(x,y); first = false; }
      else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }

  plotFFTLine(magsXR, "#00e5ff");
  plotFFTLine(magsYR, "#ffff00");
  plotFFTLine(magsXF, "#7CFC00");
  plotFFTLine(magsYF, "#ff00ff");
}

function tick() {
  drawPosition();
  drawFFT();
  requestAnimationFrame(tick);
}

window.addEventListener('load', () => {
  connectWS();
  onWinChange(document.getElementById('winSlider').value);
  onYRangeChange(document.getElementById('yrSlider').value);
  onFmaxChange(document.getElementById('fmaxSlider').value);
  onFFTWindowChange(document.getElementById('fftWinSlider').value);
  tick();
});
</script>
</body>
</html>
)rawliteral";
