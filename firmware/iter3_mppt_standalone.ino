// =====================================================
// Solar Scrubber — Iteration 3
// Standalone Perturb-and-Observe MPPT test
//
// Drives a PWM on GPIO13 to control the buck converter,
// reads dummy V/I data (or simulated solar curve), and
// runs the classic P&O algorithm to find the maximum
// power point.
// =====================================================

#include <math.h>

const int PWM_OUT_PIN   = 4;
const int PWM_FREQ      = 100000;  // 100 kHz
const int PWM_RES       = 8;       // 8-bit resolution required at 100 kHz

#define MIN_DUTY  0.10f
#define MAX_DUTY  0.90f
#define STEP_SIZE 0.01f

const unsigned long MPPT_INTERVAL_MS = 200;

// Simulated panel I-V curve so we can verify the algorithm
// without dragging the physical panel out into the sun.
const float VOC  = 12.0;
const float ISC  = 2.2;
const float VMPP = 10.0;
const float IMPP = 2.0;

float mpptDuty   = 0.50f;
float prevPower  = 0.0f;
int   perturbDir = 1;
bool  initialized = false;
unsigned long lastMpptTime = 0;

float simVoltage(float duty) {
  float v = VOC * (1.0 - duty);
  if (v < 0) v = 0;
  if (v > VOC) v = VOC;
  return v;
}

float simCurrent(float voltage) {
  float Vt = (VOC - VMPP) / log(ISC / (ISC - IMPP));
  float current = ISC * (1.0 - exp((voltage - VOC) / Vt));
  if (current < 0) current = 0;
  if (current > ISC) current = ISC;
  return current;
}

void setDuty(float duty) {
  duty = constrain(duty, MIN_DUTY, MAX_DUTY);
  mpptDuty = duty;
  uint32_t raw = (uint32_t)(duty * ((1 << PWM_RES) - 1));
  ledcWrite(PWM_OUT_PIN, raw);
}

void runMPPT() {
  float voltage = simVoltage(mpptDuty);
  float current = simCurrent(voltage);
  float power   = voltage * current;

  if (!initialized) {
    prevPower   = power;
    initialized = true;
    setDuty(mpptDuty);
    return;
  }

  if (power > prevPower) {
    mpptDuty += perturbDir * STEP_SIZE;
  } else if (power < prevPower) {
    perturbDir = -perturbDir;
    mpptDuty  += perturbDir * STEP_SIZE;
  } else {
    // Power unchanged — nudge once to avoid getting stuck
    mpptDuty += perturbDir * STEP_SIZE;
  }

  mpptDuty  = constrain(mpptDuty, MIN_DUTY, MAX_DUTY);
  prevPower = power;

  setDuty(mpptDuty);

  Serial.printf("V=%.2f  I=%.2f  P=%.2f  duty=%.3f  dir=%+d\n",
                voltage, current, power, mpptDuty, perturbDir);
}

void setup() {
  Serial.begin(115200);
  ledcAttach(PWM_OUT_PIN, PWM_FREQ, PWM_RES);
  setDuty(mpptDuty);
  Serial.println("MPPT simulated panel test on GPIO4");
  Serial.printf("Expected MPP: V=%.1f I=%.1f P=%.1f W\n",
                VMPP, IMPP, VMPP * IMPP);
}

void loop() {
  unsigned long now = millis();
  if (now - lastMpptTime >= MPPT_INTERVAL_MS) {
    lastMpptTime = now;
    runMPPT();
  }
}
