#include <Adafruit_MCP4725.h>

#define DEBUG false

// thermistor
int sda_pin = 18;
int scl_pin = 19;
int thermistor_pin = 14; // 14-23 all can do analog read
int thermistor = 0;

//-----------------------------------------//

/******** DAC Variables***********/
//-----------------------------------------//
Adafruit_MCP4725 dac;

float voltage_low = 0; // lower bound on voltage
float voltage_high = 3.3; // upper bound on voltage
float dac_value = 0; // Float to track desired DAC value

// Step size applied per correction interval. Kept small and slow so baseline
// re-centering doesn't itself create a visible kick in the 1-20Hz breathing
// signal riding on top of the baseline. Tune once the amp's DAC->output gain
// is known on the bench.
float dac_correction_amount = 0.005; // 5 mV

int therm_lower_bound = 200; // filtered baseline below this -> compensate
int therm_upper_bound = 900; // filtered baseline above this -> compensate

// The raw ADC reading contains both the fast (1-20Hz) breathing rhythm and slow
// baseline drift (thermistor self-heating, ambient temp, animal settling, etc).
// baseline_filtered is a single-pole low-pass estimate of the drift only; the DAC
// correction reacts to it instead of to raw samples so it can't be triggered by
// individual breaths, even when the baseline has drifted close to the bounds.
float baseline_tau_s = 5.0; // filter time constant, seconds (>> longest breath period)
float baseline_filtered = 0;
bool baseline_initialized = false;

unsigned long last_sample_us = 0;
unsigned long last_correction_ms = 0;
const unsigned long correction_interval_ms = 150; // how often a correction step may fire
//-----------------------------------------//

/******** Status LED ***********/
//-----------------------------------------//
const int status_led_pin = LED_BUILTIN;
const float dac_safe_low = 0.3;   // below this, DAC is too close to the rail
const float dac_safe_high = 3.0;  // above this, DAC is too close to the rail
const unsigned long blink_interval_ms = 997; // toggle period when out of range

unsigned long last_blink_ms = 0;
bool led_state = false;
//-----------------------------------------//


void setup() {

  pinMode(thermistor_pin, INPUT);
  pinMode(status_led_pin, OUTPUT);
  digitalWrite(status_led_pin, LOW);

  Wire.setSDA(sda_pin);
  Wire.setSCL(scl_pin);
  dac.begin(0x62);

  thermistor = analogRead(thermistor_pin);
  baseline_filtered = thermistor;
  baseline_initialized = true;
  last_sample_us = micros();
  last_correction_ms = millis();

  #ifdef DEBUG
    Serial.begin(9600);
    Serial.print(5);
    Serial.print(",");
    Serial.print(5);
    Serial.print(",");
    Serial.println((int) 5);
  #endif
}

void loop() {
    /*** THERMISTOR AND DAC CODE ****/
    //--------------------------------------------//
    thermistor = analogRead(thermistor_pin);
    update_baseline(thermistor);

    if (millis() - last_correction_ms >= correction_interval_ms) {
      last_correction_ms = millis();
      dac_value = update_dac(baseline_filtered, dac_value);
    }

    update_status_led(dac_value);

    #ifdef DEBUG
      Serial.print(200);
      Serial.print(",");
      Serial.print(300);
      Serial.print(",");
      Serial.print((int) thermistor);
      Serial.print(",");
      Serial.println((int) baseline_filtered);
      delay(10);
    #endif
}
//////////////////////////HELPER FUNCTIONS ///////////////
///////////////////////////////////////////////////////////

float ClampVoltage(float v) {
  if (v < voltage_low)  v = voltage_low;
  if (v > voltage_high) v = voltage_high;
  return v;
}

uint16_t ConvertDACFloatToInt(float v) {
  v = ClampVoltage(v);
  return (uint16_t)lroundf((v / voltage_high) * 4095.0f);
  // Equivalent: (uint16_t)lroundf(v / 0.0008058608f);
}

// Single-pole low-pass filter, updated every loop iteration from the raw ADC
// sample. alpha is derived from the actual elapsed time so the time constant
// stays correct regardless of loop jitter.
void update_baseline(int sample) {
  unsigned long now_us = micros();
  float dt = (now_us - last_sample_us) / 1e6f;
  last_sample_us = now_us;

  if (!baseline_initialized) {
    baseline_filtered = sample;
    baseline_initialized = true;
    return;
  }

  float alpha = dt / (baseline_tau_s + dt);
  baseline_filtered += alpha * ((float)sample - baseline_filtered);
}

// Nudges dac_value to bring the slow baseline back within bounds. Runs on its
// own slow cadence (correction_interval_ms) with small steps so each
// correction is effectively invisible to the fast breathing signal.
float update_dac(float baseline, float dac_value) {
  if (baseline < therm_lower_bound) {
    dac_value += dac_correction_amount;
  } else if (baseline > therm_upper_bound) {
    dac_value -= dac_correction_amount;
  } else {
    return dac_value; // baseline is within the target window, nothing to do
  }

  dac_value = ClampVoltage(dac_value);
  dac.setVoltage(ConvertDACFloatToInt(dac_value), false);
  return dac_value;
}

// Blinks status_led_pin at 1Hz (toggle every blink_interval_ms) whenever the
// DAC is outside [dac_safe_low, dac_safe_high]; stays off otherwise. Non-blocking.
void update_status_led(float dac_value) {
  bool out_of_range = (dac_value < dac_safe_low) || (dac_value > dac_safe_high);

  if (!out_of_range) {
    led_state = false;
    digitalWrite(status_led_pin, LOW);
    return;
  }

  unsigned long now = millis();
  if (now - last_blink_ms >= blink_interval_ms) {
    last_blink_ms = now;
    led_state = !led_state;
    digitalWrite(status_led_pin, led_state ? HIGH : LOW);
  }
}
