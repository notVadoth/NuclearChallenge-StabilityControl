// Created by 3thanaut
//
// ---------------------------------------------------------------------------
// ADDITIONS (3 Oct 2026) - the original calibration steps are unchanged.
//   1. After calibration the sketch keeps reading the sensor once a second and
//      prints frequency, absorbance and concentration from the fitted model:
//        A = -log10(f / clear_frequency),  C = beta0 + beta1 * A
//      (C in mL food dye / mL water, as in the original; negatives clamp to 0
//      like ConcentrationRegulator.ino). Readings outside the absorbance range
//      of your standards are flagged, since the fit should not be extrapolated.
//   2. Each frequency is the average of FREQ_SAMPLES readings of the original
//      formula 1000000 / (2 * pulseIn(out, HIGH)) instead of a single reading.
//   3. Serial.begin(9600) added (Serial Monitor at 9600 baud).
//   4. The pump driver pins are held LOW so the pumps cannot twitch while this
//      sketch runs (the original left them floating).
//   5. The chosen colour filter is kept for live readings (the loop used to
//      reset it to the clear filter).
//   6. Enter C at any time after calibrating to start a new calibration.
//   7. SAVED CALIBRATION: with USE_SAVED_CALIBRATION = true the sketch starts
//      straight into live readings using the previous calibration from
//      the 3 Oct 2026 calibration (4th standard estimated). Enter B to re-measure clear water (do this
//      whenever the light, phone or tank position may have changed), or C to
//      run the full calibration above instead.
//
// Rig set-up (WIRING.md): sensor taped outside the control tank wall, phone
// flashlight directly opposite at the same height, inside the closed box.
// Light passes THROUGH the liquid, so the white-paper step in the original
// prompt is not needed for this rig. Keep 350 mL in the tank for every sample.
//
// Measured pump flows at speed 60 (for reference): dye 8.9 mL/s,
// clear 7.95 mL/s, waste 9.4 mL/s. Pumps must stay within speed 55-80.
// ---------------------------------------------------------------------------

// Pump driver pins (WIRING.md). Held LOW here: this sketch never runs pumps.
const int PUMP_PINS[] = {3, 5, 6, 9, 10, 11};

// Number of readings averaged for every frequency measurement.
const int FREQ_SAMPLES = 20;
// pulseIn timeout (microseconds) so a dark or disconnected sensor cannot hang.
const unsigned long PULSE_TIMEOUT_US = 100000;

// Filter chosen during calibration, re-applied for live readings.
int chosen_s2 = HIGH;
int chosen_s3 = LOW;

// Colour Sensor input and output pins.
int out = 2;
int s0 = 4;
int s1 = 7;
int s2 = 8;
int s3 = 12;

// user input solution information
int solution_count = 0;
int num_drops = 0;
int solution_i = 0; // special iterative so we can break out of for loop early on user command.

// used to assure calibration only happens once.
// can't put calibration function in void setup or else pinmodes dont establish and all readings are zero.
bool calibrated = false;

// ---------------- Saved calibration (3 Oct 2026, 16:56) --------------------
// Known concentration C = drops * 0.05 / 350 (mL/mL); A = -log10(f / f0),
// f0 = 6166.46 Hz (clear water, red filter, 100 % scaling).
//   drops   f (Hz)     A
//     1    2386.79   0.41222
//     2    2102.23   0.46736
//     3    1676.50   0.56563
//     4    1590.16   0.58859   <- measured 1770.60 was faulty; value estimated
//                                 from the line through standards 1, 2, 3, 5
//     5    1444.28   0.63038
// Calibration curve over standards 1-5:  A = 390.2948 * C + 0.365568,
// R^2 = 0.9555. The clear-water blank (A = 0) does not lie on this line
// (1 drop already gives A = 0.41), so it is not included and readings are only
// valid between 1 and 5 drops (A 0.412 to 0.630).
// Read as C = beta0 + beta1 * A.
const bool  USE_SAVED_CALIBRATION   = true;
const float SAVED_CLEAR_FREQUENCY   = 6166.46;
const float SAVED_BETA0             = -0.0009366470;
const float SAVED_BETA1             = 0.0025621658;
const float SAVED_A_MIN             = 0.412;  // 1 drop
const float SAVED_A_MAX             = 0.630;  // 5 drops

// Absorbance range covered by the calibration in use.
float range_min = 0;
float range_max = 0;
bool saved_loaded = false;

float minAbsorption();  // defined below
float maxAbsorption();

// state machine bools.
bool _isContinuing             = false;
bool _isChoosingColour         = false;
bool _isCountingDrops          = false;

// beta values for simple linear regression of form y = beta0 + beta1 * x
float beta0;
float beta1;

float clear_frequency = 0;

// records important information about solutions.
// kept global so it can be used to record bounds for interpolation. (avoiding extrapolation because of potential innacuracy).
float solution_frequencies[10]    = {};
float solution_absorptions[10]    = {};
float solution_concentrations[10] = {};

// Average of FREQ_SAMPLES readings of the original formula
// frequency = 1000000 / (2 * pulseIn(out, HIGH)). Returns 0 if no signal.
float readFrequency() {
  float total = 0;
  int good = 0;
  for (int i = 0; i < FREQ_SAMPLES; i++) {
    unsigned long high_us = pulseIn(out, HIGH, PULSE_TIMEOUT_US);
    if (high_us > 0) {
      total += 1000000.0 / (2.0 * high_us);
      good++;
    }
  }
  if (good == 0) return 0;
  return total / good;
}

// pauses program to allow user to follow steps and confirm when they are done by entering Y.
// if notify is true, continuously prints "enter y to start"
void query_user(bool notify) {

  while (true) {

    if (_isContinuing) {

      if (Serial.available() > 0) {
        char input = Serial.read();

        if (input == 'y' || input == 'Y') {
          break;
        }
      }
    }

    if (_isCountingDrops) {
      if (Serial.available() > 0) {

        // reads the users input number.
        char c = Serial.peek();

        if (isDigit(c)) {
          num_drops = Serial.parseInt();
          Serial.println(num_drops);
          solution_count += 1;
          break;
        }else if (isAlpha(c)) {
          if (c == 'y' || c == 'Y') {

            solution_i = 10;
            break;
          }
        }else{
          Serial.read();
        }
      }
    }

    // Chooses the colour for the food dye.
    // Filter is chosen based on colour most absorbed by the solution.
    // Red food dye will absorb the most green light, causing most noticable green light intensity change.
    if (_isChoosingColour) {
      if (Serial.available() > 0) {
        char input = Serial.read();

        if (input == 'r') {
          
          // GREEN FILTER
          digitalWrite(s2, HIGH);
          digitalWrite(s3, HIGH);
          chosen_s2 = HIGH; chosen_s3 = HIGH;

          break;
        }else if (input == 'g') {

          // RED FILTER
          digitalWrite(s2, LOW);
          digitalWrite(s3, LOW);
          chosen_s2 = LOW; chosen_s3 = LOW;

          break;
        }else if (input == 'b') {

            // RED FILTER
          digitalWrite(s2, LOW);
          digitalWrite(s3, LOW);
          chosen_s2 = LOW; chosen_s3 = LOW;

          break;
        }
      }
    }

    if (notify == true) {
      delay(1000);
      Serial.println("Enter Y to start.");
    }
  }
}


void calibrate() {

  Serial.println("What colour do you want to use?");
  Serial.println("r = red");
  Serial.println("g = green");
  Serial.println("b = blue");

  _isChoosingColour = true;
  query_user(false);
  _isChoosingColour = false;

  Serial.println("The sensor needs to know what clear water looks like.");
  Serial.println("Fill the control tank with 350mL of clear water, then put a piece of white paper underneath it.");
  Serial.println("(This rig shines the flashlight through the tank onto the sensor, so no paper is needed. Keep the box closed.)");
  Serial.println("Once this is under the sensor, enter Y to continue!");

  _isContinuing = true;
  query_user(false);
  _isContinuing = false;

  // the reciprocal of the period in seconds of the square wave function represents the frequency.
  // the frequency and intensity are directly proportional in terms of how they scale, this means absorption can be found with frequency.
  // Original single reading:
  //   float clear_pulse = pulseIn(out, HIGH);
  //   clear_frequency = 1000000/(2 * clear_pulse);
  clear_frequency = readFrequency();
  if (clear_frequency <= 0) {
    Serial.println("WARNING: no signal from the sensor. Check OUT->D2, OE->GND, 5V, GND and the flashlight.");
  }

  Serial.print("Clear water frequency recorded!:  ");
  Serial.println(clear_frequency);

  Serial.println("Now, we need more known points to graph concentration against absorption.");
  Serial.println("Prepare at least 5 solutions (you can do up to 10) for the creation of the beer lamberts model.");
  Serial.println("For each solution, you will be asked to enter the INTEGER number of drops of food dye it contains.");
  Serial.println("Entering a number will store that number, and read its frequency. Then it will ask for the next solution.");
  Serial.println("Enter Y to proceed to the process of creating the model.");

  _isContinuing = true;
  query_user(false);
  _isContinuing = false;

  Serial.println("Enter Y when you are happy with the number of solutions you have scanned!");


  // Query loop to let the user add solutions.
  _isCountingDrops = true;
  for (solution_i = 0; solution_i < 10; solution_i++) {
    num_drops = 0;
    Serial.print("Enter number of drops for solution ");
    Serial.println(solution_count + 1);

    // wait for user to input number of drops.
    query_user(false);

    if (num_drops != 0) {
      // reads the wave length for the given solution.
      // Original single reading: 1000000/(2 * pulseIn(out, HIGH))
      solution_frequencies[solution_count - 1] = readFrequency();
      Serial.print("  frequency: ");
      Serial.println(solution_frequencies[solution_count - 1]);

      // converts the number of drops to a volume and then a concentration (mL food dye / mL water).
      solution_concentrations[solution_count - 1] = (num_drops * 0.05)/(350);
    }else{
      Serial.println("Clear solution already used in calibration! Need solution with some new concentration of dye.");
      solution_i -= 1;
    }
  }
  _isCountingDrops = false;

  Serial.println("All solutions prepared! Calculating the calibration curve...");

  // Calibration curve: the concentration of each standard is KNOWN
  //   C = (drops * 0.05 mL) / 350 mL
  // and absorbance is MEASURED
  //   A = -log10(f / f0),  f0 = clear-water frequency.
  // Fit the straight line A = slope * C + intercept (absorbance on the y-axis,
  // concentration on the x-axis), with the clear-water point (C = 0, A = 0)
  // included as the blank.
  for (int i = 0; i < solution_count; i++) {
    solution_absorptions[i] = -log10(solution_frequencies[i]/clear_frequency);
  }

  Serial.println();
  Serial.println("drops   concentration (mL/mL)   frequency (Hz)   absorbance");
  Serial.print("0       0.0000000000            ");
  Serial.print(clear_frequency, 2);
  Serial.println("          0.0000000000");
  float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;   // x = C, y = A (blank adds zeros)
  int n = solution_count + 1;
  for (int i = 0; i < solution_count; i++) {
    float c = solution_concentrations[i];
    float a = solution_absorptions[i];
    sumX += c; sumY += a; sumXX += c * c; sumXY += c * a;
    Serial.print(c * 350 / 0.05, 0);
    Serial.print("       ");
    Serial.print(c, 10);
    Serial.print("            ");
    Serial.print(solution_frequencies[i], 2);
    Serial.print("          ");
    Serial.println(a, 10);
  }

  float slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
  float intercept = (sumY / n) - slope * (sumX / n);

  // R squared of A against C (blank included).
  float meanA = sumY / n;
  float ssRes = intercept * intercept;          // blank point: A = 0 at C = 0
  float ssTot = meanA * meanA;
  for (int i = 0; i < solution_count; i++) {
    float fitted = slope * solution_concentrations[i] + intercept;
    ssRes += (solution_absorptions[i] - fitted) * (solution_absorptions[i] - fitted);
    ssTot += (solution_absorptions[i] - meanA) * (solution_absorptions[i] - meanA);
  }

  Serial.println();
  Serial.println("Calibration curve:  A = slope * C + intercept");
  Serial.print("  slope     = "); Serial.println(slope, 4);
  Serial.print("  intercept = "); Serial.println(intercept, 6);
  Serial.print("  R squared = "); Serial.println(ssTot > 0 ? 1 - ssRes / ssTot : 0, 6);

  // To read an unknown, the same line is used the other way round:
  //   C = (A - intercept) / slope  =  beta0 + beta1 * A
  beta1 = 1.0 / slope;
  beta0 = -intercept / slope;
  Serial.print("beta1: ");
  Serial.println(beta1, 10);
  Serial.print("beta0: ");
  Serial.println(beta0, 10);

  Serial.println();
  Serial.println("Paste into ConcentrationRegulator.ino:");
  Serial.print("float clear_frequency = ");
  Serial.print(clear_frequency, 2);
  Serial.println(";");
  Serial.print("float beta0 = ");
  Serial.print(beta0, 10);
  Serial.println(";");
  Serial.print("float beta1 = ");
  Serial.print(beta1, 10);
  Serial.println(";");
  Serial.println();
  Serial.println("Live readings follow (once a second). Enter C to calibrate again.");

  range_min = minAbsorption();
  range_max = maxAbsorption();
  calibrated = true;
}

// Lowest and highest absorbance among the standards (the calibrated range).
float minAbsorption() {
  float m = solution_absorptions[0];
  for (int i = 1; i < solution_count; i++) if (solution_absorptions[i] < m) m = solution_absorptions[i];
  return m;
}
float maxAbsorption() {
  float m = solution_absorptions[0];
  for (int i = 1; i < solution_count; i++) if (solution_absorptions[i] > m) m = solution_absorptions[i];
  return m;
}

// Use the saved calibration from the spreadsheet (green dye -> red filter).
void loadSavedCalibration() {
  clear_frequency = SAVED_CLEAR_FREQUENCY;
  beta0 = SAVED_BETA0;
  beta1 = SAVED_BETA1;
  range_min = SAVED_A_MIN;
  range_max = SAVED_A_MAX;
  chosen_s2 = LOW;  // RED FILTER, as calibrate() selects for green dye ('g')
  chosen_s3 = LOW;
  calibrated = true;
  saved_loaded = true;

  Serial.println("Using the SAVED calibration (3 Oct 2026, 4th standard estimated):");
  Serial.print("  clear_frequency = "); Serial.println(clear_frequency, 2);
  Serial.print("  beta0 = "); Serial.println(beta0, 10);
  Serial.print("  beta1 = "); Serial.println(beta1, 10);
  Serial.println("  valid for absorbance 0.412 to 0.630 (1 to 5 drops in 350 mL)");
  Serial.println("Enter B to re-measure clear water, C for a full new calibration.");
  Serial.println();
}

// Re-measure only the clear-water frequency (keeps beta0 and beta1).
void remeasureClear() {
  Serial.println("Fill the tank with 350 mL of clear water, close the box, then enter Y.");
  while (Serial.available() > 0) Serial.read();
  _isContinuing = true;
  query_user(false);
  _isContinuing = false;

  digitalWrite(s2, chosen_s2);
  digitalWrite(s3, chosen_s3);
  delay(5);
  float f = readFrequency();
  if (f <= 0) {
    Serial.println("No signal from the sensor; clear frequency NOT changed.");
    return;
  }
  clear_frequency = f;
  Serial.print("Clear water frequency recorded!:  ");
  Serial.println(clear_frequency, 2);
  Serial.print("Paste into ConcentrationRegulator.ino: float clear_frequency = ");
  Serial.print(clear_frequency, 2);
  Serial.println(";");
}

// One live reading: frequency -> absorbance -> concentration.
void printLiveReading() {
  digitalWrite(s2, chosen_s2);
  digitalWrite(s3, chosen_s3);
  delay(5);

  float frequency = readFrequency();
  if (frequency <= 0) {
    Serial.println("No signal from the sensor. Check the wiring and flashlight.");
    return;
  }
  float absorption = -log10(frequency / clear_frequency);
  float concentration = beta0 + beta1 * absorption;
  if (concentration < 0) concentration = 0;

  Serial.print("frequency: ");
  Serial.print(frequency, 2);
  Serial.print(" Hz   absorbance: ");
  Serial.print(absorption, 5);
  Serial.print("   concentration: ");
  Serial.print(concentration, 10);
  Serial.print(" mL/mL (");
  Serial.print(concentration * 1000, 4);
  Serial.print(" mL/L, ~");
  Serial.print(concentration * 350 / 0.05, 1);
  Serial.print(" drops in 350 mL)");
  if (range_max > range_min && (absorption < range_min - 0.005 || absorption > range_max + 0.005)) {
    Serial.print("   [outside calibrated range]");
  }
  Serial.println();
}


void setup() {
  // put your setup code here, to run once:

  // Keep every pump off while calibrating.
  for (int i = 0; i < 6; i++) {
    digitalWrite(PUMP_PINS[i], LOW);
    pinMode(PUMP_PINS[i], OUTPUT);
  }

  Serial.begin(9600);
  while (!Serial && millis() < 3000) {}
  
  pinMode(out, INPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
    s0 | s1 | Output Frequency Scaling
     L | L  | Power Off 
     L | H  | 2%
     H | L  | 20%
     H | H  | 100%
  */
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);

  /*
    s2 | s3 | Photo Diode Type
     L | L  | RED
     L | H  | BLUE
     H | L  | CLEAR
     H | H  | GREEN
  */
  // start with clear filter
  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);

  if (!calibrated && USE_SAVED_CALIBRATION && !saved_loaded) {
    loadSavedCalibration();
    return;
  }

  if (!calibrated) {

    _isContinuing = true;
    query_user(true);
    _isContinuing = false;
    calibrate();
    Serial.println("Calibration complete! Please copy your ABSORPTION and CONCENTRATION values and move to the concentration regulator program!");
  }else{
    // Live concentration from the measured frequency, once a second.
    printLiveReading();

    // Enter C to calibrate again.
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == 'c' || input == 'C') {
        calibrated = false;
        solution_count = 0;
        Serial.println("Starting a new calibration.");
        return;
      }
      if (input == 'b' || input == 'B') {
        remeasureClear();
        return;
      }
    }
    delay(1000);
    return;
  }
  delay(1000);
}
