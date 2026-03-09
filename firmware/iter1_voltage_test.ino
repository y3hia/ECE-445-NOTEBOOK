// =====================================================
// Solar Scrubber — Iteration 1
// Initial breadboard test on ESP32-C3 Super Mini
//
// Just reads voltage off a single GPIO and prints to serial
// to verify the ADC and divider work end-to-end.
// =====================================================

const int PULSE_PIN = 6;
const int DIR_PIN = 7;
const int Panel_voltage_1 = 0;   // ADC1 channel on the C3

const int PWM_FREQ = 4000;
const int PWM_RESOLUTION = 8;
const int DUTY_CYCLE = 127;

void setup() {
  Serial.begin(115200);

  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  pinMode(Panel_voltage_1, INPUT);

  bool success = ledcAttach(PULSE_PIN, PWM_FREQ, PWM_RESOLUTION);

  if (success) {
    Serial.println("System Ready. PWM Configured for 4 kHz.");
  } else {
    Serial.println("Error: PWM setup failed!");
  }

  Serial.println("Starting Sweep Sequence...");
}

void loop() {
  int rawADC = analogRead(Panel_voltage_1);
  float voltage = (rawADC / 4095.0) * 3.3;
  Serial.printf("Pin %d -> Raw ADC: %d | Voltage: %.3f V\n",
                Panel_voltage_1, rawADC, voltage);

  // simple back-and-forth motor sweep for sanity check
  Serial.println("Moving Forward...");
  digitalWrite(DIR_PIN, HIGH);
  ledcWrite(PULSE_PIN, DUTY_CYCLE);
  delay(5000);

  ledcWrite(PULSE_PIN, 0);
  Serial.println("Stopped at far end.");
  delay(1000);

  Serial.println("Moving Reverse...");
  digitalWrite(DIR_PIN, LOW);
  ledcWrite(PULSE_PIN, DUTY_CYCLE);
  delay(5000);

  ledcWrite(PULSE_PIN, 0);
  Serial.println("Returned to home.");
  delay(2000);
}
