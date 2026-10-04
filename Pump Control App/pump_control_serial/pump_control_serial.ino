// Pump control + colour sensor over USB serial  (v3)
//
// Lets the "Pump Control" app on the computer drive the three pumps of the
// dye-concentration rig, read the SEN0101 colour sensor live, and store a
// Beer-Lambert calibration. Pins match WIRING.md and the repo sketches.
//
// The pumps are still controlled by hand from the app. Closed-loop control
// from the sensor (pumps reacting to concentration) comes at a later stage.
//
// Hardware (Arduino UNO R4 Minima):
//   Pumps - 2 x DRV8833 motor drivers on the 9 V battery rail
//     Dye pump   : Driver 1 IN1/IN2 -> D3 (PWM) / D5 (held LOW)
//     Clear pump : Driver 1 IN3/IN4 -> D6 (PWM) / D9 (held LOW)
//     Waste pump : Driver 2 IN1/IN2 -> D10 (PWM) / D11 (held LOW)
//   Colour sensor - SEN0101 / TCS3200, powered from Arduino 5V, OE tied to GND
//     OUT -> D2, S0 -> D4, S1 -> D7, S2 -> D8, S3 -> D12
//
// Sensor method (from the repo sketches):
//   - The TCS3200 outputs a square wave whose frequency rises with light
//     intensity. Frequency is averaged over several full periods
//     (pulseIn LOW + HIGH), as in test_rgb.ino / dye_concentration_controller.ino.
//   - Beer-Lambert (BeerLambertCalibrator.ino):
//       absorbance  A = -log10(f / f_clear)
//       concentration C = beta0 + beta1 * A     (mL food dye / mL water)
//     f_clear is the frequency through 350 mL of clear water. Standards are
//     made by adding drops (0.05 mL each) of the same dyed reservoir solution
//     the pump uses to 350 mL of water.
//   - Use the colour filter the dye absorbs most. Green dye absorbs red light,
//     so the RED filter is the default (BeerLambertCalibrator maps g -> red).
//   - Keep the sensor, flashlight, 350 mL volume and box closed and unchanged
//     between calibration and use. Any change to lighting invalidates it.
//
// Serial: 115200 baud, one command per line (newline terminated):
//   Pumps
//     SET <D|C|W> <0 or 55-80>     run a pump at that speed until stopped or capped
//     RUN <D|C|W> <55-80> <secs>   run a pump for a set time (up to its cap)
//     STOP                         stop every pump at once
//   Sensor
//     CHAN <R|G|B|C>               colour filter used for concentration
//     SCALE <2|20|100>             TCS3200 output-frequency scaling, in %
//     SAMPLE <n>                   average n readings of the chosen filter
//   Calibration (saved in the Arduino's EEPROM, survives power-off)
//     CAL SET <f_clear> <beta0> <beta1> <A_min> <A_max> <R|G|B|C> <scale>
//     CAL GET                      report the stored calibration
//     CAL CLEAR                    erase it (falls back to the spreadsheet calibration)
//   Link
//     PING                         keep-alive from the app
//     STATUS                       report pump state now
//
// The Arduino sends:
//   READY pump_control_serial v3
//   STATE D=140,0 C=0,0 W=140,8500     (pwm, ms left on a timed run; 0 = untimed)
//   SENS ch=R sc=20 r=.. g=.. b=.. c=.. f=.. a=.. conc=..   (Hz; nan if no signal)
//   SAMPLE ch=R sc=20 n=30 ok=30 mean=.. sd=.. min=.. max=..
//   CAL f0=.. b0=.. b1=.. amin=.. amax=.. ch=R sc=20   or   CAL none
//   OK ... / ERR ... / SAFETY ...
//
// Safety:
//   - All pumps start OFF.
//   - If no command or PING arrives for WATCHDOG_MS, every pump stops.
//   - Speed is 0 (off) or SPEED_MIN..SPEED_MAX (55..80 of 255). Anything else is
//     rejected. The 9 V supply overdrives the 3 V pumps at higher settings.
//   - Each pump has its own maximum single run, sized from its measured flow so
//     one run cannot overflow the tank (350 mL working, 450 mL limit).
//   - There is no level sensor: watch the tank and keep it under 450 mL.

#include <EEPROM.h>
#include <math.h>

// ---------------- Pins ----------------
const uint8_t DYE_PUMP_PWM_PIN   = 3;  // Driver 1 IN1
const uint8_t DYE_PUMP_LOW_PIN   = 5;  // Driver 1 IN2
const uint8_t CLEAR_PUMP_PWM_PIN = 6;  // Driver 1 IN3
const uint8_t CLEAR_PUMP_LOW_PIN = 9;  // Driver 1 IN4
const uint8_t WASTE_PUMP_PWM_PIN = 10; // Driver 2 IN1
const uint8_t WASTE_PUMP_LOW_PIN = 11; // Driver 2 IN2

const uint8_t SENSOR_OUT_PIN = 2;
const uint8_t SENSOR_S0_PIN  = 4;
const uint8_t SENSOR_S1_PIN  = 7;
const uint8_t SENSOR_S2_PIN  = 8;
const uint8_t SENSOR_S3_PIN  = 12;

// ---------------- Timing / safety ----------------
const unsigned long BAUD = 115200;
const unsigned long WATCHDOG_MS = 3000;       // stop pumps if the app goes quiet
// Allowed speeds (PWM out of 255). 0 = off.
const int SPEED_MIN = 55;
const int SPEED_MAX = 80;

// Measured flow at speed 60 (bench test, 3 Oct 2026), mL/s:
//   Dye 8.9   Clear 7.95   Waste (flush) 9.4
// Flow at other speeds is estimated in proportion (flow * speed / 60) until
// more speeds are measured.
const float FLOW60_DYE   = 8.9f;
const float FLOW60_CLEAR = 7.95f;
const float FLOW60_WASTE = 9.4f;

// The longest single run is sized from the flow so one run can never move more
// than this volume. From the 350 mL working level:
//   inflow (dye, clear): +95 mL  -> ~445 mL, under the 450 mL limit
//   waste             : -250 mL -> ~100 mL
//   e.g. speed 60: dye 10.7 s, clear 11.9 s, waste 26.6 s
//        speed 80: dye  8.0 s, clear  9.0 s, waste 20.0 s
const float MAX_INFLOW_PER_RUN_ML = 95.0f;
const float MAX_WASTE_PER_RUN_ML  = 250.0f;
const unsigned long STATE_REPORT_MS = 250;    // how often STATE lines are sent
const unsigned long SENSOR_REPORT_MS = 500;   // how often SENS lines are sent

// ---------------- Sensor settings ----------------
// pulseIn timeout per half-period. At 2 % scaling and low light a half-period
// can be tens of ms, so this is generous; a reading aborts on the first timeout.
const unsigned long SENSOR_PULSE_TIMEOUT_US = 60000;
const uint8_t LIVE_PERIODS   = 8;   // periods averaged per live reading
const uint8_t SAMPLE_PERIODS = 12;  // periods averaged per SAMPLE reading
const uint8_t MAX_SAMPLE_N   = 60;

char sensorChannel = 'R';   // R G B C  (red filter by default for green dye)
uint8_t sensorScale = 100;  // 2, 20 or 100 % (100 % matches BeerLambertCalibrator / ConcentrationRegulator)

// ---------------- Calibration (EEPROM) ----------------
const uint32_t CAL_MAGIC = 0x44594531; // "DYE1"
struct Calibration {
  uint32_t magic;
  float fClear;   // Hz through clear water
  float beta0;    // C = beta0 + beta1 * A
  float beta1;
  float aMin;     // absorbance range covered by the standards
  float aMax;
  char channel;   // filter used
  uint8_t scale;  // scaling used
};
Calibration cal;
bool calValid = false;

// ---------------- Pumps ----------------
struct Pump {
  char id;               // 'D', 'C' or 'W'
  uint8_t pwmPin;
  uint8_t lowPin;
  int pwm;               // 0..255
  unsigned long startMs; // when it was switched on
  unsigned long runMs;   // 0 = untimed (still capped by maxRunMs)
  float flow60;          // measured mL/s at speed 60
  float maxRunMl;        // most volume one run may move
  unsigned long maxRunMs;// cap for the current speed (set when switched on)
};

Pump pumps[3] = {
  {'D', DYE_PUMP_PWM_PIN,   DYE_PUMP_LOW_PIN,   0, 0, 0, FLOW60_DYE,   MAX_INFLOW_PER_RUN_ML, 0},
  {'C', CLEAR_PUMP_PWM_PIN, CLEAR_PUMP_LOW_PIN, 0, 0, 0, FLOW60_CLEAR, MAX_INFLOW_PER_RUN_ML, 0},
  {'W', WASTE_PUMP_PWM_PIN, WASTE_PUMP_LOW_PIN, 0, 0, 0, FLOW60_WASTE, MAX_WASTE_PER_RUN_ML,  0},
};

unsigned long lastCommandMs = 0;
unsigned long lastReportMs = 0;
unsigned long lastSensorMs = 0;
bool watchdogTripped = false;

void enforceTimers(bool checkWatchdog); // defined below

char lineBuf[128];
uint8_t lineLen = 0;

// ======================================================================
// Pumps
// ======================================================================
Pump *findPump(char id) {
  id = toupper(id);
  for (uint8_t i = 0; i < 3; ++i) {
    if (pumps[i].id == id) return &pumps[i];
  }
  return nullptr;
}

void applyPump(Pump &p, int pwm, unsigned long runMs) {
  pwm = constrain(pwm, 0, 255);
  p.pwm = pwm;
  p.startMs = millis();
  // Estimated flow at this speed, and the longest run that stays within maxRunMl.
  const float flow = p.flow60 * pwm / 60.0f;
  p.maxRunMs = (pwm > 0 && flow > 0) ? (unsigned long)(p.maxRunMl / flow * 1000.0f) : 0;
  p.runMs = (pwm > 0) ? min(runMs, p.maxRunMs) : 0;
  digitalWrite(p.lowPin, LOW);
  analogWrite(p.pwmPin, pwm);
}

void stopAll() {
  for (uint8_t i = 0; i < 3; ++i) applyPump(pumps[i], 0, 0);
}

unsigned long msLeft(const Pump &p) {
  if (p.pwm == 0 || p.runMs == 0) return 0;
  const unsigned long elapsed = millis() - p.startMs;
  return (elapsed >= p.runMs) ? 0 : p.runMs - elapsed;
}

void reportState() {
  Serial.print(F("STATE"));
  for (uint8_t i = 0; i < 3; ++i) {
    Serial.print(' ');
    Serial.print(pumps[i].id);
    Serial.print('=');
    Serial.print(pumps[i].pwm);
    Serial.print(',');
    Serial.print(msLeft(pumps[i]));
  }
  Serial.println();
}

// ======================================================================
// Sensor (TCS3200 / SEN0101)
// ======================================================================
void applyScale(uint8_t scale) {
  //  s0 | s1 | Output Frequency Scaling
  //   L | L  | Power Off
  //   L | H  | 2%
  //   H | L  | 20%
  //   H | H  | 100%
  if (scale == 2)        { digitalWrite(SENSOR_S0_PIN, LOW);  digitalWrite(SENSOR_S1_PIN, HIGH); }
  else if (scale == 100) { digitalWrite(SENSOR_S0_PIN, HIGH); digitalWrite(SENSOR_S1_PIN, HIGH); }
  else                   { digitalWrite(SENSOR_S0_PIN, HIGH); digitalWrite(SENSOR_S1_PIN, LOW); scale = 20; }
  sensorScale = scale;
}

void selectFilter(char ch) {
  //  s2 | s3 | Photo Diode Type
  //   L | L  | RED
  //   L | H  | BLUE
  //   H | L  | CLEAR
  //   H | H  | GREEN
  switch (ch) {
    case 'R': digitalWrite(SENSOR_S2_PIN, LOW);  digitalWrite(SENSOR_S3_PIN, LOW);  break;
    case 'B': digitalWrite(SENSOR_S2_PIN, LOW);  digitalWrite(SENSOR_S3_PIN, HIGH); break;
    case 'C': digitalWrite(SENSOR_S2_PIN, HIGH); digitalWrite(SENSOR_S3_PIN, LOW);  break;
    default:  digitalWrite(SENSOR_S2_PIN, HIGH); digitalWrite(SENSOR_S3_PIN, HIGH); break; // 'G'
  }
  delay(3); // let the photodiode filter selection settle
}

// Average frequency over `periods` full square-wave periods. NAN if no signal.
float readFrequencyHz(char ch, uint8_t periods) {
  selectFilter(ch);
  unsigned long totalUs = 0;
  for (uint8_t i = 0; i < periods; ++i) {
    const unsigned long lowUs  = pulseIn(SENSOR_OUT_PIN, LOW,  SENSOR_PULSE_TIMEOUT_US);
    if (lowUs == 0) return NAN;
    const unsigned long highUs = pulseIn(SENSOR_OUT_PIN, HIGH, SENSOR_PULSE_TIMEOUT_US);
    if (highUs == 0) return NAN;
    totalUs += lowUs + highUs;
  }
  if (totalUs == 0) return NAN;
  return 1000000.0f * periods / totalUs;
}

bool calMatchesSettings() {
  return calValid && cal.channel == sensorChannel && cal.scale == sensorScale;
}

float absorbanceFrom(float f) {
  if (!calMatchesSettings() || isnan(f) || f <= 0 || cal.fClear <= 0) return NAN;
  return -log10(f / cal.fClear);
}

float concentrationFrom(float a) {
  if (isnan(a)) return NAN;
  float c = cal.beta0 + cal.beta1 * a;
  if (c < 0) c = 0; // as in ConcentrationRegulator.ino
  return c;
}

void printHz(const __FlashStringHelper *key, float v) {
  Serial.print(key);
  if (isnan(v)) Serial.print(F("nan")); else Serial.print(v, 2);
}

void reportSensor() {
  const float r = readFrequencyHz('R', LIVE_PERIODS);
  const float g = readFrequencyHz('G', LIVE_PERIODS);
  const float b = readFrequencyHz('B', LIVE_PERIODS);
  const float c = readFrequencyHz('C', LIVE_PERIODS);
  float f = NAN;
  switch (sensorChannel) {
    case 'R': f = r; break;
    case 'G': f = g; break;
    case 'B': f = b; break;
    default:  f = c; break;
  }
  const float a = absorbanceFrom(f);
  const float conc = concentrationFrom(a);

  Serial.print(F("SENS ch="));
  Serial.print(sensorChannel);
  Serial.print(F(" sc="));
  Serial.print(sensorScale);
  printHz(F(" r="), r);
  printHz(F(" g="), g);
  printHz(F(" b="), b);
  printHz(F(" c="), c);
  printHz(F(" f="), f);
  Serial.print(F(" a="));
  if (isnan(a)) Serial.print(F("nan")); else Serial.print(a, 5);
  Serial.print(F(" conc="));
  if (isnan(conc)) Serial.print(F("nan")); else Serial.print(conc, 8);
  Serial.println();
}

// SAMPLE: average n readings of the chosen filter (used by calibration).
void takeSample(int n) {
  n = constrain(n, 1, MAX_SAMPLE_N);
  int ok = 0;
  double sum = 0, sumSq = 0;
  float lo = 1e9, hi = 0;
  for (int i = 0; i < n; ++i) {
    const float f = readFrequencyHz(sensorChannel, SAMPLE_PERIODS);
    enforceTimers(false); // keep timed pump runs honest while sampling
    if (isnan(f)) continue;
    ++ok;
    sum += f;
    sumSq += (double)f * f;
    if (f < lo) lo = f;
    if (f > hi) hi = f;
  }
  lastCommandMs = millis(); // the app was waiting on us, not silent

  Serial.print(F("SAMPLE ch="));
  Serial.print(sensorChannel);
  Serial.print(F(" sc="));
  Serial.print(sensorScale);
  Serial.print(F(" n="));
  Serial.print(n);
  Serial.print(F(" ok="));
  Serial.print(ok);
  if (ok == 0) {
    Serial.println(F(" mean=nan sd=nan min=nan max=nan"));
    return;
  }
  const double mean = sum / ok;
  const double var = (ok > 1) ? max(0.0, (sumSq - ok * mean * mean) / (ok - 1)) : 0.0;
  Serial.print(F(" mean="));
  Serial.print((float)mean, 3);
  Serial.print(F(" sd="));
  Serial.print((float)sqrt(var), 3);
  Serial.print(F(" min="));
  Serial.print(lo, 3);
  Serial.print(F(" max="));
  Serial.println(hi, 3);
}

// ======================================================================
// Calibration storage
// ======================================================================
// Default calibration (3 Oct 2026, 16:56), used when nothing has been saved to
// EEPROM from the app. Known C = drops*0.05/350, A = -log10(f/f0), f0 = 6166.46 Hz.
// Curve over standards 1-5 (4th estimated from the others, measured value was
// faulty): A = 390.2948 C + 0.365568, R^2 = 0.9555; valid for 1-5 drops.
bool calIsPreset = false;
void loadPresetCalibration() {
  cal.magic   = CAL_MAGIC;
  cal.fClear  = 6166.46f;
  cal.beta0   = -0.0009366470f;
  cal.beta1   = 0.0025621658f;
  cal.aMin    = 0.412f;
  cal.aMax    = 0.630f;
  cal.channel = 'R';
  cal.scale   = 100;
  calValid = true;
  calIsPreset = true;
}

void loadCalibration() {
  EEPROM.get(0, cal);
  calValid = (cal.magic == CAL_MAGIC) && cal.fClear > 0 && !isnan(cal.beta1);
  calIsPreset = false;
  if (!calValid) loadPresetCalibration();
}

void reportCalibration() {
  if (!calValid) {
    Serial.println(F("CAL none"));
    return;
  }
  Serial.print(F("CAL f0="));
  Serial.print(cal.fClear, 3);
  Serial.print(F(" b0="));
  Serial.print(cal.beta0, 10);
  Serial.print(F(" b1="));
  Serial.print(cal.beta1, 10);
  Serial.print(F(" amin="));
  Serial.print(cal.aMin, 6);
  Serial.print(F(" amax="));
  Serial.print(cal.aMax, 6);
  Serial.print(F(" ch="));
  Serial.print(cal.channel);
  Serial.print(F(" sc="));
  Serial.print(cal.scale);
  Serial.println(calIsPreset ? F(" src=preset") : F(" src=saved"));
}

// ======================================================================
// Commands
// ======================================================================
void handleLine(char *line) {
  for (char *c = line; *c; ++c) *c = toupper(*c);

  char original[sizeof(lineBuf)];
  strncpy(original, line, sizeof(original) - 1);
  original[sizeof(original) - 1] = '\0';

  // Split into words. (strtok/atol/atof avoid sscanf float support, which is
  // not always linked on Arduino boards.)
  char *w[10] = {nullptr};
  int n = 0;
  for (char *tok = strtok(line, " \t"); tok && n < 10; tok = strtok(nullptr, " \t")) {
    w[n++] = tok;
  }
  if (n < 1) return;
  const char *cmd = w[0];

  lastCommandMs = millis();
  if (watchdogTripped) {
    watchdogTripped = false;
    Serial.println(F("OK link restored"));
  }

  if (strcmp(cmd, "PING") == 0) return;
  if (strcmp(cmd, "STATUS") == 0) { reportState(); return; }
  if (strcmp(cmd, "STOP") == 0) {
    stopAll();
    Serial.println(F("OK STOP all pumps off"));
    reportState();
    return;
  }

  if (strcmp(cmd, "SET") == 0 || strcmp(cmd, "RUN") == 0) {
    const bool timed = strcmp(cmd, "RUN") == 0;
    Pump *p = (n >= 2) ? findPump(w[1][0]) : nullptr;
    const long pwm = (n >= 3) ? atol(w[2]) : -1;
    const float secs = (n >= 4) ? (float)atof(w[3]) : -1.0f;
    const bool speedOk = pwm == 0 || (pwm >= SPEED_MIN && pwm <= SPEED_MAX);
    if (p && n >= 3 && !speedOk) {
      Serial.print(F("ERR speed must be 0 or "));
      Serial.print(SPEED_MIN);
      Serial.print('-');
      Serial.print(SPEED_MAX);
      Serial.print(F(": "));
      Serial.println(original);
      return;
    }
    if (!p || n < 3 || (timed && (n < 4 || secs <= 0))) {
      Serial.print(F("ERR bad command: "));
      Serial.println(original);
      return;
    }
    const unsigned long runMs = timed ? (unsigned long)(secs * 1000.0f) : 0;
    applyPump(*p, (int)pwm, runMs);
    Serial.print(F("OK "));
    Serial.print(cmd);
    Serial.print(' ');
    Serial.print(p->id);
    Serial.print(F(" pwm="));
    Serial.print(p->pwm);
    if (timed) {
      Serial.print(F(" ms="));
      Serial.print(p->runMs);
      if (p->runMs < runMs) Serial.print(F(" (capped)"));
    }
    Serial.println();
    reportState();
    return;
  }

  if (strcmp(cmd, "CHAN") == 0) {
    const char ch = (n >= 2) ? w[1][0] : 0;
    if (ch != 'R' && ch != 'G' && ch != 'B' && ch != 'C') {
      Serial.print(F("ERR bad command: "));
      Serial.println(original);
      return;
    }
    sensorChannel = ch;
    Serial.print(F("OK CHAN "));
    Serial.println(sensorChannel);
    return;
  }

  if (strcmp(cmd, "SCALE") == 0) {
    const long s = (n >= 2) ? atol(w[1]) : -1;
    if (s != 2 && s != 20 && s != 100) {
      Serial.print(F("ERR bad command: "));
      Serial.println(original);
      return;
    }
    applyScale((uint8_t)s);
    Serial.print(F("OK SCALE "));
    Serial.println(sensorScale);
    return;
  }

  if (strcmp(cmd, "SAMPLE") == 0) {
    takeSample((n >= 2) ? (int)atol(w[1]) : 20);
    return;
  }

  if (strcmp(cmd, "CAL") == 0) {
    const char *sub = (n >= 2) ? w[1] : "";
    if (strcmp(sub, "GET") == 0) { reportCalibration(); return; }
    if (strcmp(sub, "CLEAR") == 0) {
      cal.magic = 0;
      EEPROM.put(0, cal);
      loadPresetCalibration();
      sensorChannel = cal.channel;
      applyScale(cal.scale);
      Serial.println(F("OK CAL cleared, back to the spreadsheet calibration"));
      reportCalibration();
      return;
    }
    if (strcmp(sub, "SET") == 0 && n >= 9) {
      Calibration c;
      c.magic  = CAL_MAGIC;
      c.fClear = (float)atof(w[2]);
      c.beta0  = (float)atof(w[3]);
      c.beta1  = (float)atof(w[4]);
      c.aMin   = (float)atof(w[5]);
      c.aMax   = (float)atof(w[6]);
      c.channel = w[7][0];
      c.scale  = (uint8_t)atol(w[8]);
      const bool chOk = c.channel == 'R' || c.channel == 'G' || c.channel == 'B' || c.channel == 'C';
      const bool scOk = c.scale == 2 || c.scale == 20 || c.scale == 100;
      if (c.fClear <= 0 || c.beta1 == 0 || !chOk || !scOk) {
        Serial.print(F("ERR bad calibration: "));
        Serial.println(original);
        return;
      }
      cal = c;
      EEPROM.put(0, cal);
      calValid = true;
      calIsPreset = false;
      // Use the filter and scaling the calibration was made with.
      sensorChannel = cal.channel;
      applyScale(cal.scale);
      Serial.println(F("OK CAL saved"));
      reportCalibration();
      return;
    }
    Serial.print(F("ERR bad command: "));
    Serial.println(original);
    return;
  }

  Serial.print(F("ERR unknown command: "));
  Serial.println(original);
}

void readSerial() {
  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0; // overlong line: drop it
      Serial.println(F("ERR line too long"));
    }
  }
}

void enforceTimers(bool checkWatchdog) {
  const unsigned long now = millis();

  // Timed runs and the hard cap.
  for (uint8_t i = 0; i < 3; ++i) {
    Pump &p = pumps[i];
    if (p.pwm == 0) continue;
    const unsigned long elapsed = now - p.startMs;
    const unsigned long limit = (p.runMs > 0) ? p.runMs : p.maxRunMs;
    if (elapsed >= limit) {
      const bool timedRun = p.runMs > 0;
      applyPump(p, 0, 0);
      Serial.print(timedRun ? F("OK DONE ") : F("SAFETY max run reached, stopped "));
      Serial.println(p.id);
      reportState();
    }
  }

  // Lost link to the app.
  if (checkWatchdog && !watchdogTripped && now - lastCommandMs > WATCHDOG_MS) {
    bool anyOn = false;
    for (uint8_t i = 0; i < 3; ++i) anyOn |= pumps[i].pwm > 0;
    stopAll();
    watchdogTripped = true;
    if (anyOn) Serial.println(F("SAFETY no message from app, all pumps off"));
  }
}

// ======================================================================
void setup() {
  // Preload safe pump states before enabling outputs.
  for (uint8_t i = 0; i < 3; ++i) {
    digitalWrite(pumps[i].pwmPin, LOW);
    digitalWrite(pumps[i].lowPin, LOW);
    pinMode(pumps[i].pwmPin, OUTPUT);
    pinMode(pumps[i].lowPin, OUTPUT);
  }
  stopAll();

  pinMode(SENSOR_OUT_PIN, INPUT);
  pinMode(SENSOR_S0_PIN, OUTPUT);
  pinMode(SENSOR_S1_PIN, OUTPUT);
  pinMode(SENSOR_S2_PIN, OUTPUT);
  pinMode(SENSOR_S3_PIN, OUTPUT);
  applyScale(100); // same scaling as the repo's Beer-Lambert sketches

  loadCalibration();
  if (calValid) {
    sensorChannel = cal.channel;
    applyScale(cal.scale);
  }

  Serial.begin(BAUD);
  while (!Serial && millis() < 3000) {}
  lastCommandMs = millis();
  Serial.println(F("READY pump_control_serial v3"));
  reportCalibration();
  reportState();
}

void loop() {
  readSerial();
  enforceTimers(true);

  const unsigned long now = millis();
  if (now - lastReportMs >= STATE_REPORT_MS) {
    lastReportMs = now;
    reportState();
  }
  if (now - lastSensorMs >= SENSOR_REPORT_MS) {
    lastSensorMs = now;
    reportSensor();
  }
}
