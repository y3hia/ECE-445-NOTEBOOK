// =====================================================
// Solar Scrubber — Iteration 5 (FINAL)
// Combined MPPT + panel cleaner controller
//
// Adds on top of Iteration 4:
//   - Per-panel voltage-based dirt detection (more reliable
//     than power-based when current sense is noisy)
//   - 5-reading streak debouncing to ignore brief shadows
//   - Snapshot-on-dirty pre-drop voltage tracking
//   - Wiggle motion overlaid on vertical sweeps
//   - Manual WASD override mode via serial
//   - Post-clean verification with 10-second settling timer
//   - Idle timeout fallback for periodic deep cleans
// =====================================================

const int VSENSE_1_PIN = 4;
const int VSENSE_2_PIN = 5;
const int VSENSE_3_PIN = 6;
const int ISENSE_PIN   = 7;
const int PWM_OUT_PIN  = 13;

const float SCALE_V1   = 4.2;
const float SCALE_V2   = 9.2;
const float SCALE_V3   = 15.0;
const float ADC_VREF   = 3.3f;
const float ADC_MAX    = 4095.0f;
const float ISENSE_VREF = 0.95f;

const float RSENSE     = 0.003f;
const float AMP_GAIN   = 20.0f;

const int DCDC_PWM_FREQ = 50000;
const int DCDC_PWM_RES  = 8;

#define MIN_DUTY   0.05f
#define MAX_DUTY   0.5f
#define STEP_SIZE  0.005f
const unsigned long MPPT_INTERVAL_MS = 200;

static float mpptDuty    = 0.25f;
static float prevPower   = 0.0f;
static int   perturbDir  = 1;
static bool  initialized = false;
unsigned long lastMpptTime = 0;

float lastP1 = 0.0f;
float lastP2 = 0.0f;

float baselineP1 = 0.0f;
float baselineP2 = 0.0f;
const float BASELINE_ALPHA = 0.1f;
bool rebaselineNext = false;

const float DIRT_RATIO_THRESHOLD = 0.40f;
const float MIN_POWER_FOR_COMPARE = 0.5f;

const unsigned long IDLE_DEEP_CLEAN_MS = 30000;
unsigned long lastActivityTime = 0;

const int DIRT_CONFIRM_COUNT = 5;
int  panel1Streak = 0;
int  panel2Streak = 0;
float panel1Drop  = 0.0f;
float panel2Drop  = 0.0f;

float recentHighP1 = 0.0f;
float recentHighP2 = 0.0f;
const float HIGH_DECAY = 0.995f;

float preDropP1 = 0.0f;
float preDropP2 = 0.0f;
bool  awaitingPostCleanCheck = false;
unsigned long postCleanReadyTime = 0;
const unsigned long POST_CLEAN_WAIT_MS = 10000;

#define NUM_PANELS 3
const int PWM_PIN   = 38;
const int DIR_PIN   = 39;
const int PWM_PIN_V = 41;
const int DIR_PIN_V = 42;
const int MOTOR_PWM_FREQ = 4000;
const int MOTOR_PWM_RES  = 8;
const int MOTOR_PWM_DUTY = 128;

const float SPEED_IPS = 2.5;
const unsigned long H_TIME_REDUCTION_MS = 500;

const float panelWidth[NUM_PANELS]  = { 13.0, 13.0, 13.0 };
const float panelHeight[NUM_PANELS] = { 22.0, 22.0, 22.0 };
const float panelStartX[NUM_PANELS] = { 0.0, 13.0, 26.0 };

const float REST_X = 0.0;
const float REST_Y = 22.0;
const float GRID_MAX_X = 39.0;
const float GRID_MAX_Y = 22.0;
const float MANUAL_STEP = 4.0;

const float WIGGLE_AMOUNT  = 5.0f;
const int   WIGGLE_SEGMENTS = 6;

bool dirty[NUM_PANELS] = { false, false, false };

float posX = REST_X;
float posY = REST_Y;

struct Move { bool isH; float target; };
#define MAX_MOVES 150
Move moveQueue[MAX_MOVES];
int queueLen = 0;
int queueIdx = 0;

enum MotorState { IDLE, MOVING, DONE, MANUAL } motorState = IDLE;
unsigned long moveStart, moveDur;
bool movingH;
float moveTarget;

float lastV1 = 0.0f;
float lastV2 = 0.0f;
float baselineV1 = 0.0f;
float baselineV2 = 0.0f;
float recentHighV1 = 0.0f;
float recentHighV2 = 0.0f;
float preDropV1 = 0.0f;
float preDropV2 = 0.0f;

int readADCavg(int pin, int samples = 16) {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(pin);
  return sum / samples;
}

float readNodeVoltage(int pin, float scale) {
  int raw = readADCavg(pin);
  return (raw / ADC_MAX) * ADC_VREF * scale;
}

float readCurrent(int &rawOut, float &vOut) {
  rawOut = readADCavg(ISENSE_PIN, 32);
  vOut   = (rawOut / ADC_MAX) * ISENSE_VREF;
  return vOut / (AMP_GAIN * RSENSE);
}

void setDuty(float duty) {
  duty = constrain(duty, MIN_DUTY, MAX_DUTY);
  mpptDuty = duty;
  uint32_t raw = (uint32_t)(duty * ((1 << DCDC_PWM_RES) - 1));
  ledcWrite(PWM_OUT_PIN, raw);
}

void runMPPT() {
  float node1 = readNodeVoltage(VSENSE_1_PIN, SCALE_V1);
  float node2 = readNodeVoltage(VSENSE_2_PIN, SCALE_V2);

  float v1 = node1;
  float v2 = node2 - node1;
  if (v2 < 0) v2 = 0;
  float totalV = v1 + v2;

  int   iRaw;
  float iVolts;
  float i = readCurrent(iRaw, iVolts);

  float p1 = v1 * i;
  float p2 = v2 * i;
  float power = totalV * i;

  lastP1 = p1;
  lastP2 = p2;
  lastV1 = v1;
  lastV2 = v2;

  recentHighV1 = max(recentHighV1 * HIGH_DECAY, v1);
  recentHighV2 = max(recentHighV2 * HIGH_DECAY, v2);
  recentHighP1 = max(recentHighP1 * HIGH_DECAY, p1);
  recentHighP2 = max(recentHighP2 * HIGH_DECAY, p2);

  if (!initialized) {
    prevPower    = power;
    baselineP1   = p1;
    baselineP2   = p2;
    baselineV1   = v1;
    baselineV2   = v2;
    recentHighP1 = p1;
    recentHighP2 = p2;
    recentHighV1 = v1;
    recentHighV2 = v2;
    initialized  = true;
    return;
  }

  if (rebaselineNext) {
    baselineP1     = p1;
    baselineP2     = p2;
    baselineV1     = v1;
    baselineV2     = v2;
    rebaselineNext = false;
    Serial.printf("[REBASE] baseV1=%.2f baseV2=%.2f (fresh start)\n",
                  baselineV1, baselineV2);
  }

  if (power > prevPower) {
    mpptDuty += perturbDir * STEP_SIZE;
  } else if (power < prevPower) {
    perturbDir = -perturbDir;
    mpptDuty  += perturbDir * STEP_SIZE;
  }

  mpptDuty  = constrain(mpptDuty, MIN_DUTY, MAX_DUTY);
  prevPower = power;
  setDuty(mpptDuty);

  Serial.printf("[MPPT] V1=%.2f V2=%.2f Vtot=%.2f | I=%.4f | P1=%.2f P2=%.2f Ptot=%.2f | duty=%.3f\n",
                v1, v2, totalV, i, p1, p2, power, mpptDuty);
  Serial.printf("[BASE] baseV1=%.2f baseV2=%.2f hiV1=%.2f hiV2=%.2f\n",
                baselineV1, baselineV2, recentHighV1, recentHighV2);
}

void resetStreaks() {
  panel1Streak = 0;
  panel2Streak = 0;
  panel1Drop   = 0.0f;
  panel2Drop   = 0.0f;
}

void detectDirtyPanels(int outIdx[NUM_PANELS], int &outCount) {
  outCount = 0;

  if (rebaselineNext) {
    resetStreaks();
    return;
  }

  float drop1 = (baselineV1 > 0.5f) ? (baselineV1 - lastV1) / baselineV1 : 0.0f;
  float drop2 = (baselineV2 > 0.5f) ? (baselineV2 - lastV2) / baselineV2 : 0.0f;

  bool p1Dirty = (drop1 >= DIRT_RATIO_THRESHOLD);
  bool p2Dirty = (drop2 >= DIRT_RATIO_THRESHOLD);

  if (p1Dirty) {
    if (panel1Streak == 0) preDropV1 = recentHighV1;
    panel1Streak++;
    panel1Drop = drop1;
  } else {
    panel1Streak = 0;
    baselineV1 = (1.0f - BASELINE_ALPHA) * baselineV1 + BASELINE_ALPHA * lastV1;
  }

  if (p2Dirty) {
    if (panel2Streak == 0) preDropV2 = recentHighV2;
    panel2Streak++;
    panel2Drop = drop2;
  } else {
    panel2Streak = 0;
    baselineV2 = (1.0f - BASELINE_ALPHA) * baselineV2 + BASELINE_ALPHA * lastV2;
  }

  if (p1Dirty || p2Dirty) {
    Serial.printf("[DETECT-V] V1=%.2f vs %.2f drop=%.1f%% (%d/%d) | V2=%.2f vs %.2f drop=%.1f%% (%d/%d)\n",
                  lastV1, baselineV1, drop1 * 100.0f, panel1Streak, DIRT_CONFIRM_COUNT,
                  lastV2, baselineV2, drop2 * 100.0f, panel2Streak, DIRT_CONFIRM_COUNT);
  }

  bool p1Confirmed = (panel1Streak >= DIRT_CONFIRM_COUNT);
  bool p2Confirmed = (panel2Streak >= DIRT_CONFIRM_COUNT);

  if (!p1Confirmed && !p2Confirmed) return;

  if (p1Confirmed && p2Confirmed) {
    if (panel1Drop >= panel2Drop) {
      outIdx[0] = 0;
      outIdx[1] = 1;
    } else {
      outIdx[0] = 1;
      outIdx[1] = 0;
    }
    outCount = 2;
  } else if (p1Confirmed) {
    outIdx[0] = 0;
    outCount = 1;
  } else {
    outIdx[0] = 1;
    outCount = 1;
  }
}

void verifyPostClean(int outIdx[NUM_PANELS], int &outCount) {
  outCount = 0;
  if (!awaitingPostCleanCheck) return;
  if (millis() < postCleanReadyTime) return;

  awaitingPostCleanCheck = false;

  float postDrop1 = (preDropV1 > 0.5f) ? (preDropV1 - lastV1) / preDropV1 : 0.0f;
  float postDrop2 = (preDropV2 > 0.5f) ? (preDropV2 - lastV2) / preDropV2 : 0.0f;

  Serial.printf("[POSTCHK-V] vs pre-drop: V1 %.2f->%.2f (%.1f%%) | V2 %.2f->%.2f (%.1f%%)\n",
                preDropV1, lastV1, postDrop1 * 100.0f,
                preDropV2, lastV2, postDrop2 * 100.0f);

  bool p1StillBad = (postDrop1 >= DIRT_RATIO_THRESHOLD);
  bool p2StillBad = (postDrop2 >= DIRT_RATIO_THRESHOLD);

  if (!p1StillBad && !p2StillBad) {
    Serial.println("[POSTCHK] Cleaning successful, panels recovered.");
    return;
  }

  Serial.println("[POSTCHK] Cleaning incomplete, queuing another cycle.");

  if (p1StillBad && p2StillBad) {
    if (postDrop1 >= postDrop2) { outIdx[0] = 0; outIdx[1] = 1; }
    else                        { outIdx[0] = 1; outIdx[1] = 0; }
    outCount = 2;
  } else if (p1StillBad) {
    outIdx[0] = 0; outCount = 1;
  } else {
    outIdx[0] = 1; outCount = 1;
  }
}

void enqueue(bool isH, float target) {
  if (queueLen < MAX_MOVES)
    moveQueue[queueLen++] = { isH, target };
}

float clampX(float x) {
  if (x < 0)          return 0;
  if (x > GRID_MAX_X) return GRID_MAX_X;
  return x;
}

float clampY(float y) {
  if (y < 0)          return 0;
  if (y > GRID_MAX_Y) return GRID_MAX_Y;
  return y;
}

void buildCleanPath(int cleanOrder[], int orderCount) {
  queueLen = 0;
  queueIdx = 0;
  float bx = posX, by = posY;

  for (int k = 0; k < orderCount; k++) {
    int p = cleanOrder[k];
    float panelX = panelStartX[p];
    float topY   = panelHeight[p];

    if (abs(bx - panelX) > 0.01f) {
      enqueue(true, panelX);
      bx = panelX;
    }

    float startY  = by;
    float endY    = (by >= topY / 2.0f) ? 0.0f : topY;
    float segStep = (endY - startY) / WIGGLE_SEGMENTS;

    for (int s = 0; s < WIGGLE_SEGMENTS; s++) {
      float ySeg = startY + segStep * (s + 1);
      enqueue(false, ySeg);

      if (s < WIGGLE_SEGMENTS - 1) {
        enqueue(true, bx + WIGGLE_AMOUNT);
        enqueue(true, bx);
      }
    }
    by = endY;
    Serial.print("[CLEAN] Queuing panel "); Serial.println(p + 1);
  }

  enqueue(false, REST_Y);
  enqueue(true,  REST_X);
}

void executeNextMove() {
  while (queueIdx < queueLen) {
    Move m = moveQueue[queueIdx];
    float cur  = m.isH ? posX : posY;
    float dist = abs(m.target - cur);

    if (dist < 0.01f) {
      if (m.isH) posX = m.target;
      else       posY = m.target;
      queueIdx++;
      continue;
    }

    movingH    = m.isH;
    moveTarget = m.target;

    unsigned long rawDur = (unsigned long)(dist / SPEED_IPS * 1000.0f);
    if (m.isH && rawDur > H_TIME_REDUCTION_MS) {
      rawDur -= H_TIME_REDUCTION_MS;
    }
    moveDur   = rawDur;
    moveStart = millis();

    if (m.isH) {
      digitalWrite(DIR_PIN, m.target > posX ? LOW : HIGH);
      ledcWrite(PWM_PIN, MOTOR_PWM_DUTY);
    } else {
      digitalWrite(DIR_PIN_V, m.target > posY ? HIGH : LOW);
      ledcWrite(PWM_PIN_V, MOTOR_PWM_DUTY);
    }

    motorState = MOVING;
    Serial.print(m.isH ? "[CLEAN] H -> " : "[CLEAN] V -> ");
    Serial.println(m.target, 2);
    return;
  }

  motorState = IDLE;
  Serial.println("[CLEAN] Done. Back at rest.");
  for (int i = 0; i < NUM_PANELS; i++) dirty[i] = false;
  resetStreaks();
  rebaselineNext = true;
  lastActivityTime = millis();

  awaitingPostCleanCheck = true;
  postCleanReadyTime = millis() + POST_CLEAN_WAIT_MS;
  Serial.printf("[POSTCHK] Will verify in %lu seconds...\n", POST_CLEAN_WAIT_MS / 1000);
}

void runMotor() {
  if (motorState != MOVING) return;
  if (millis() - moveStart >= moveDur) {
    if (movingH) { ledcWrite(PWM_PIN,   0); posX = moveTarget; }
    else         { ledcWrite(PWM_PIN_V, 0); posY = moveTarget; }
    queueIdx++;
    executeNextMove();
  }
}

void maybeStartCleaning() {
  if (motorState != IDLE) return;
  int cleanOrder[NUM_PANELS];
  int count = 0;

  verifyPostClean(cleanOrder, count);
  if (count == 0) detectDirtyPanels(cleanOrder, count);

  if (count == 0) {
    if (millis() - lastActivityTime >= IDLE_DEEP_CLEAN_MS) {
      Serial.println("[CTRL] Idle timeout reached -> deep clean (all panels)");
      for (int i = 0; i < NUM_PANELS; i++) cleanOrder[i] = i;
      count = NUM_PANELS;
      lastActivityTime = millis();
    } else {
      return;
    }
  } else {
    lastActivityTime = millis();
  }

  for (int i = 0; i < NUM_PANELS; i++) dirty[i] = false;
  for (int k = 0; k < count; k++) dirty[cleanOrder[k]] = true;

  Serial.print("[CTRL] Cleaning order: ");
  for (int k = 0; k < count; k++) {
    Serial.print("Panel ");
    Serial.print(cleanOrder[k] + 1);
    if (k < count - 1) Serial.print(" -> ");
  }
  Serial.println();

  resetStreaks();
  buildCleanPath(cleanOrder, count);
  executeNextMove();
}

void manualMove(bool isH, float target) {
  target = isH ? clampX(target) : clampY(target);
  float cur  = isH ? posX : posY;
  float dist = abs(target - cur);
  if (dist < 0.01f) return;

  unsigned long dur = (unsigned long)(dist / SPEED_IPS * 1000.0f);
  if (isH && dur > H_TIME_REDUCTION_MS) dur -= H_TIME_REDUCTION_MS;

  if (isH) {
    digitalWrite(DIR_PIN, target > posX ? LOW : HIGH);
    ledcWrite(PWM_PIN, MOTOR_PWM_DUTY);
    delay(dur);
    ledcWrite(PWM_PIN, 0);
    posX = target;
  } else {
    digitalWrite(DIR_PIN_V, target > posY ? HIGH : LOW);
    ledcWrite(PWM_PIN_V, MOTOR_PWM_DUTY);
    delay(dur);
    ledcWrite(PWM_PIN_V, 0);
    posY = target;
  }

  Serial.printf("[MANUAL] Pos: (%.2f, %.2f)\n", posX, posY);
}

void handleSerialInput() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r') return;

  switch (c) {
    case 'm': case 'M':
      if (motorState == MANUAL) {
        Serial.println("[MANUAL] Exiting manual mode. MPPT resuming.");
        motorState = IDLE;
        resetStreaks();
        lastActivityTime = millis();
      } else {
        Serial.println("[MANUAL] Entering manual mode. WASD = move, X = stop, M = exit.");
        ledcWrite(PWM_PIN, 0);
        ledcWrite(PWM_PIN_V, 0);
        motorState = MANUAL;
      }
      break;
    case 'w': case 'W':
      if (motorState == MANUAL) { manualMove(false, posY + MANUAL_STEP); }
      break;
    case 's': case 'S':
      if (motorState == MANUAL) { manualMove(false, posY - MANUAL_STEP); }
      break;
    case 'a': case 'A':
      if (motorState == MANUAL) { manualMove(true,  posX - MANUAL_STEP); }
      break;
    case 'd': case 'D':
      if (motorState == MANUAL) { manualMove(true,  posX + MANUAL_STEP); }
      break;
    case 'x': case 'X':
      Serial.println("[MANUAL] STOP");
      ledcWrite(PWM_PIN, 0);
      ledcWrite(PWM_PIN_V, 0);
      break;
    case 'p': case 'P':
      Serial.printf("[MANUAL] Pos: (%.2f, %.2f)\n", posX, posY);
      break;
    case 'b': case 'B':
      Serial.println("[BASELINE] Manual rebase requested");
      rebaselineNext = true;
      resetStreaks();
      break;
    case 'h': case 'H':
      Serial.println("Commands: M / WASD / X / P / B / H");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(VSENSE_1_PIN, INPUT);
  pinMode(VSENSE_2_PIN, INPUT);
  pinMode(VSENSE_3_PIN, INPUT);
  pinMode(ISENSE_PIN,   INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ISENSE_PIN, ADC_0db);

  bool ok = ledcAttach(PWM_OUT_PIN, DCDC_PWM_FREQ, DCDC_PWM_RES);
  Serial.printf("[MPPT] ledcAttach: %s\n", ok ? "OK" : "FAILED");
  setDuty(MAX_DUTY);

  pinMode(DIR_PIN,   OUTPUT);
  pinMode(DIR_PIN_V, OUTPUT);
  bool okH = ledcAttach(PWM_PIN,   MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  bool okV = ledcAttach(PWM_PIN_V, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  Serial.printf("[CLEAN] ledcAttach H:%s V:%s\n",
                okH ? "OK" : "FAILED", okV ? "OK" : "FAILED");
  ledcWrite(PWM_PIN,   0);
  ledcWrite(PWM_PIN_V, 0);

  lastActivityTime = millis();
  motorState = IDLE;
  Serial.println("[CTRL] Idle. Monitoring power... Press H for commands.");
}

void loop() {
  handleSerialInput();
  unsigned long now = millis();
  if (motorState == IDLE && now - lastMpptTime >= MPPT_INTERVAL_MS) {
    lastMpptTime = now;
    runMPPT();
    maybeStartCleaning();
  }
  runMotor();
}
