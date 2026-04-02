// =====================================================
// Solar Scrubber — Iteration 2
// Two-pin voltage scaling test
//
// After calibrating the dividers outdoors:
//   Panel 1: 10.32 V real / 0.912 V at pin = 11.316x
//   Panel 1+2 series: 18.33 V real / 0.734 V = 24.973x
// Panel 2 is derived by subtracting the recovered V1
// from the recovered series voltage.
// =====================================================

const int PULSE_PIN = 6;
const int DIR_PIN = 7;
const int Panel_voltage_1 = 1;    // ADC sees stepped-down V1
const int Panel_voltage_2 = 0;    // ADC sees stepped-down (V1 + V2)

const int PWM_FREQ = 4000;
const int PWM_RESOLUTION = 8;
const int DUTY_CYCLE = 127;

const float SCALE_PANEL_1 = 11.316;
const float SCALE_PANEL_2 = 24.973;

float readRealVoltage(int pin, float scale) {
  const int SAMPLES = 16;
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(pin);
  }
  float adcAvg = (float)sum / SAMPLES;
  float vPin = (adcAvg / 4095.0) * 3.3;
  float vReal = vPin * scale;
  return vReal;
}

void setup() {
  Serial.begin(115200);

  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  pinMode(Panel_voltage_1, INPUT);
  pinMode(Panel_voltage_2, INPUT);

  bool success = ledcAttach(PULSE_PIN, PWM_FREQ, PWM_RESOLUTION);

  if (success) {
    Serial.println("System Ready. PWM Configured for 4 kHz.");
  } else {
    Serial.println("Error: PWM setup failed!");
  }
}

void loop() {
  float vPanel1 = readRealVoltage(Panel_voltage_1, SCALE_PANEL_1);
  float vSeries = readRealVoltage(Panel_voltage_2, SCALE_PANEL_2);
  float vPanel2 = vSeries - vPanel1;
  if (vPanel2 < 0) vPanel2 = 0;

  Serial.printf("Panel 1: %.2f V | Series: %.2f V | Panel 2: %.2f V\n",
                vPanel1, vSeries, vPanel2);

  delay(500);
}
