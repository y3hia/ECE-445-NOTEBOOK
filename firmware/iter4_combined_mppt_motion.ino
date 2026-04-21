// =====================================================
// Solar Scrubber — Iteration 4
// MPPT + locomotion integration
//
// Wires together:
//   - Real ADC reads for V1, V2, current
//   - P&O MPPT on GPIO13
//   - A 2-axis non-blocking move queue
//   - Hardcoded panel grid (3 panels in a row)
//   - Targeted cleaning when a panel is flagged dirty
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

const float RSENSE     = 0.01f;
const float AMP_GAIN   = 200.0f;

const int DCDC_PWM_FREQ = 50000;
const int DCDC_PWM_RES  = 10;

#define MIN_DUTY   0.05f
#define MAX_DUTY   0.36f
#define STEP_SIZE  0.005f
const unsigned long MPPT_INTERVAL_MS = 200;

static float mpptDuty    = 0.25f;
static float prevPower   = 0.0f;
static int   perturbDir  = 1;
static bool  initialized = false;
unsigned long lastMpptTime = 0;

#define NUM_PANELS 3

const int PWM_PIN   = 38;
const int DIR_PIN   = 39;
const int PWM_PIN_V = 41;
const int DIR_PIN_V = 42;
const int MOTOR_PWM_FREQ = 4000;
const int MOTOR_PWM_RES  = 8;
const int MOTOR_PWM_DUTY = 128;

const float SPEED_IPS = 2.5;
const float panelWidth[NUM_PANELS]  = { 13.0, 13.0, 11.8 };
const float panelHeight[NUM_PANELS] = { 22.0, 22.0, 16.5 };

const float REST_X = 37.8;
const float REST_Y = 22.0;

bool dirty[NUM_PANELS] = { true, true, true };

float posX = REST_X;
float posY = REST_Y;

struct Move { bool isH; float target; };
#define MAX_MOVES 50
Move moveQueue[MAX_MOVES];
int queueLen = 0;
int queueIdx = 0;

enum MotorState { IDLE, MOVING, DONE } motorState = IDLE;
unsigned long moveStart, moveDur;
bool movingH;
float moveTarget;

int readADCavg(int pin, int samples = 16) {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(pin);
  return sum / samples;
}

float readNodeVoltage(int pin, float scale) {
  int raw = readADCavg(pin);
  return (raw / ADC_MAX) * ADC_VREF * scale;
}

float readCurrent() {
  int raw = readADCavg(ISENSE_PIN, 32);
  float v = (raw / ADC_MAX) * ISENSE_VREF;
  return v / (AMP_GAIN * RSENSE);
}

void setDuty(float duty) {
  duty = constrain(duty, MIN_DUTY, MAX_DUTY);
  mpptDuty = duty;
  uint32_t raw = (uint32_t)(duty * ((1 << DCDC_PWM_RES) - 1));
  ledcWrite(PWM_OUT_PIN, raw);
}

void runMPPT() {
  float v1 = readNodeVoltage(VSENSE_1_PIN, SCALE_V1);
  float v2 = readNodeVoltage(VSENSE_2_PIN, SCALE_V2);
  float v3 = readNodeVoltage(VSENSE_3_PIN, SCALE_V3);
  float i  = readCurrent();

  float totalV = v1 + v2 + v3;
  float power  = totalV * i;

  if (!initialized) {
    prevPower   = power;
    initialized = true;
    return;
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

  Serial.printf("V1=%.2f V2=%.2f V3=%.2f Vtot=%.2f I=%.3f P=%.2f duty=%.3f\n",
                v1, v2, v3, totalV, i, power, mpptDuty);
}

float getPanelRightX(int p) {
  float x = 0;
  for (int i = 0; i <= p; i++) x += panelWidth[i];
  return x;
}

void enqueue(bool isH, float target) {
  if (queueLen < MAX_MOVES)
    moveQueue[queueLen++] = { isH, target };
}

void buildCleanPath() {
  queueLen = 0;
  queueIdx = 0;
  float by = posY;

  for (int p = NUM_PANELS - 1; p >= 0; p--) {
    if (!dirty[p]) continue;
    float rx   = getPanelRightX(p);
    float topY = panelHeight[p];

    enqueue(true, rx);

    float targetY = (by >= topY / 2.0f) ? 0.0f : topY;
    enqueue(false, targetY);
    by = targetY;
  }

  enqueue(true,  REST_X);
  enqueue(false, REST_Y);
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
    moveDur    = (unsigned long)(dist / SPEED_IPS * 1000.0f);
    moveStart  = millis();

    if (m.isH) {
      digitalWrite(DIR_PIN, m.target > posX ? HIGH : LOW);
      ledcWrite(PWM_PIN, MOTOR_PWM_DUTY);
    } else {
      digitalWrite(DIR_PIN_V, m.target > posY ? HIGH : LOW);
      ledcWrite(PWM_PIN_V, MOTOR_PWM_DUTY);
    }

    motorState = MOVING;
    return;
  }
  motorState = DONE;
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

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(VSENSE_1_PIN, INPUT);
  pinMode(VSENSE_2_PIN, INPUT);
  pinMode(VSENSE_3_PIN, INPUT);
  pinMode(ISENSE_PIN,   INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ISENSE_PIN, ADC_0db);

  ledcAttach(PWM_OUT_PIN, DCDC_PWM_FREQ, DCDC_PWM_RES);
  setDuty(0.25f);

  pinMode(DIR_PIN,   OUTPUT);
  pinMode(DIR_PIN_V, OUTPUT);
  ledcAttach(PWM_PIN,   MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttach(PWM_PIN_V, MOTOR_PWM_FREQ, MOTOR_PWM_RES);

  buildCleanPath();
  executeNextMove();
}

void loop() {
  unsigned long now = millis();
  if (now - lastMpptTime >= MPPT_INTERVAL_MS) {
    lastMpptTime = now;
    runMPPT();
  }
  runMotor();
}
