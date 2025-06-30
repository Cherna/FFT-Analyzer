#include <Arduino.h>
#include <arduinoFFT.h>
#include <EEPROM.h>

// Debug configuration - Uncomment to enable
#define PRINT_RAW_AUDIO false      // Print raw audio samples
#define PRINT_FFT_PEAKS false      // Print strongest FFT peaks
#define PRINT_DETAILED_BANDS false // Print detailed band information
#define PRINT_BANDS false          // Print final band values for plotting
#define PRINT_GAIN false          // Print gain changes

// Previous debug flags
#define DEBUG_RAW_FFT false       // Print raw FFT values
#define DEBUG_FREQUENCY_BANDS false // Print frequency band calculations
#define DEBUG_RAW_MAGNITUDES false // Print raw magnitude values
#define DEBUG_FINAL_VALUES false   // Print final values before LED output

// Hardware configuration
#define CONFIG_SWITCH_PIN 16       // Switch to enter config mode
#define POTENTIOMETER_PIN 33
#define AUDIO_INPUT_PIN 32
#define BAND_UP_PIN 34
#define BAND_DOWN_PIN 35
#define LED_ARR_LEN 8              // Number of indicator LEDs
const int INDICATOR_LED_PINS[LED_ARR_LEN] = {21, 22, 23, 25, 26, 27, 5, 17};

// FFT configuration
#define NUM_SAMPLES 512          // Number of samples to collect before performing FFT
#define SAMPLING_RATE 24000       // Sampling rate in Hz
#define FFT_SIZE NUM_SAMPLES      // Size of the FFT (must be a power of 2)
#define FREQ_BANDS_LEN 9
#define OUTPUT_PINS_LEN 8
const int FREQ_BANDS[FREQ_BANDS_LEN] = {60, 250, 500, 1000, 2000, 4000, 6000, 8000, 10000}; // 9 values for 8 bands
const int OUTPUT_PINS[OUTPUT_PINS_LEN] = {2, 4, 12, 13, 14, 15, 18, 19};
#define NUM_BINS 8

#define samplingPeriodUs round(1000000 * (1.0 / SAMPLING_RATE))

// Button state variables
bool lastUpButtonState = false;
bool lastDownButtonState = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool lastButtonState = HIGH;

// Mode and configuration variables
bool configMode = false;
bool displayIntensity = false;
int currentBand = -1;
float bandGains[NUM_BINS];        // Store gain values for each frequency band
#define EEPROM_START_ADDR 0
#define DEFAULT_GAIN 1.0
#define MAX_GAIN 3.0

// FFT variables
double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, NUM_SAMPLES, SAMPLING_RATE);

// Timing
unsigned long lastReading = 0;
const long readingInterval = 10;

// Noise floor - adjust based on your observations
const float NOISE_FLOOR = 20.0;

void loadConfiguration() {
  EEPROM.begin(512);              // Initialize EEPROM with size

  // Read gains from EEPROM
  for (int i = 0; i < NUM_BINS; i++) {
    float gain;
    EEPROM.get(EEPROM_START_ADDR + (i * sizeof(float)), gain);

    // Validate and set gain
    if (isnan(gain) || gain < 0.1 || gain > MAX_GAIN) {
      bandGains[i] = DEFAULT_GAIN;
    } else {
      bandGains[i] = gain;
    }
  }
  EEPROM.end();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(AUDIO_INPUT_PIN, INPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);

  // Initialize pins
  pinMode(CONFIG_SWITCH_PIN, INPUT_PULLDOWN);
  pinMode(BAND_DOWN_PIN, INPUT_PULLDOWN);
  pinMode(BAND_UP_PIN, INPUT_PULLDOWN);

  for (int i = 0; i < LED_ARR_LEN; i++) {
    pinMode(INDICATOR_LED_PINS[i], OUTPUT);
    digitalWrite(INDICATOR_LED_PINS[i], LOW);
  }

  for (int i = 0; i < OUTPUT_PINS_LEN; i++) {
    pinMode(OUTPUT_PINS[i], OUTPUT);
  }

  // Load saved configuration
  loadConfiguration();
}

void saveConfiguration() {
  EEPROM.begin(512);
  for (int i = 0; i < NUM_BINS; i++) {
    EEPROM.put(EEPROM_START_ADDR + (i * sizeof(float)), bandGains[i]);
  }
  EEPROM.commit();
  EEPROM.end();

  Serial.println("Configuration saved");
}

void updateLEDs() {
  // In config mode, only light up the current band's LED
  for (int i = 0; i < LED_ARR_LEN; i++) {
    digitalWrite(INDICATOR_LED_PINS[i], i == currentBand ? HIGH : LOW);
  }
}

void handleConfigMode() {
  // Read band button with debounce
  bool upButton = digitalRead(BAND_UP_PIN);
  bool downButton = digitalRead(BAND_DOWN_PIN);
  bool debounceReady = (millis() - lastDebounceTime) > debounceDelay;
  
  if (upButton != lastUpButtonState || downButton != lastDownButtonState) {
    lastDebounceTime = millis();
  }

  if (debounceReady) {
    if (upButton != lastUpButtonState && upButton) {
      currentBand = (currentBand + 1) % NUM_BINS;
      updateLEDs();
    }
    if (downButton != lastDownButtonState && downButton) {
      currentBand = (currentBand > 0) ? currentBand - 1 : NUM_BINS - 1;
      updateLEDs();
    }

    lastUpButtonState = upButton;
    lastDownButtonState = downButton;
  }

  // Read potentiometer and update gain for current band
  int potValue = analogRead(POTENTIOMETER_PIN);
  float newGain = map(potValue, 0, 4096, 1, 50) / 10.0;

  if (abs(newGain - bandGains[currentBand]) > 0.1) { // Only update if change is significant
    bandGains[currentBand] = newGain;

    #if PRINT_GAIN
    // Debug output
    Serial.print("Updated band ");
    Serial.print(currentBand);
    Serial.print(" gain to ");
    Serial.println(newGain);
    #endif
  }
}

void collectSamples() {
  unsigned long nextSampleTime = micros();
  float sum = 0;
  
  // First pass - just calculate average for DC offset
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int sample = analogRead(AUDIO_INPUT_PIN);
    sum += sample;
    
    #if PRINT_RAW_AUDIO
    if (i % 64 == 0) { // Print every 64th sample to avoid overwhelming serial
      Serial.print("Sample ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(sample);
    }
    #endif
    
    nextSampleTime += samplingPeriodUs;
    delayMicroseconds(samplingPeriodUs);
  }
  float dcOffset = sum / NUM_SAMPLES;
  
  #if PRINT_RAW_AUDIO
  Serial.print("DC Offset: ");
  Serial.println(dcOffset);
  #endif
  
  // Second pass - collect samples with DC offset removed
  nextSampleTime = micros();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    vReal[i] = analogRead(AUDIO_INPUT_PIN) - dcOffset;
    vImag[i] = 0;
    nextSampleTime += samplingPeriodUs;
    while (micros() < nextSampleTime) {
      // Wait
    }
  }
}

void performFFT() {
  // Apply windowing function to reduce spectral leakage
  FFT.windowing(vReal, NUM_SAMPLES, FFT_WIN_TYP_BLACKMAN_HARRIS, FFT_FORWARD);
  // Perform FFT
  FFT.compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);
  // Convert complex results to magnitude
  FFT.complexToMagnitude(vReal, vImag, NUM_SAMPLES);

  // Debug: Print FFT peaks
  #if PRINT_FFT_PEAKS
  Serial.println("\n=== FFT Peaks ===");
  // Find top 5 peaks
  for (int peak = 0; peak < 5; peak++) {
    double maxVal = 0;
    int maxIndex = 0;
    
    // Find the maximum value and its index
    for (int i = 1; i < NUM_SAMPLES/2; i++) {
      if (vReal[i] > maxVal) {
        maxVal = vReal[i];
        maxIndex = i;
      }
    }
    
    // Calculate the corresponding frequency
    double peakFreq = (double)maxIndex * SAMPLING_RATE / NUM_SAMPLES;
    
    // Print the peak information
    Serial.print("Peak ");
    Serial.print(peak+1);
    Serial.print(": ");
    Serial.print(peakFreq);
    Serial.print(" Hz (Magnitude: ");
    Serial.print(maxVal);
    Serial.println(")");
    
    // Set this peak's value to 0 to find the next highest
    vReal[maxIndex] = 0;
  }
  Serial.println("=================\n");
  #endif
}

void outputFFT() {
  static float smoothedMagnitudes[NUM_BINS] = {0};
  const float smoothingFactor = 0.95; // Lower value = more smoothing (0.2 instead of 0.85)

  #if PRINT_BANDS
  Serial.print(">");
  #endif

  // Process each frequency band
  for (int i = 0; i < NUM_BINS; i++) {
    float lowFreq = FREQ_BANDS[i];
    float highFreq = FREQ_BANDS[i + 1];

    int startBin = round(lowFreq * NUM_SAMPLES / SAMPLING_RATE);
    int endBin = round(highFreq * NUM_SAMPLES / SAMPLING_RATE);

    float magnitude = 0;
    for (int j = startBin; j <= endBin; j++) {
        if (vReal[j] > magnitude) {
            magnitude = vReal[j];  // Pick the strongest bin
        }
    }

    // Apply smoothing for visual stability
    smoothedMagnitudes[i] = (magnitude * smoothingFactor) + (smoothedMagnitudes[i] * (1 - smoothingFactor));
    
    // Apply gain adjustment
    float gainAdjustedMagnitude = smoothedMagnitudes[i] * bandGains[i];
    
    #if PRINT_DETAILED_BANDS
    Serial.print("  Smoothed: ");
    Serial.print(smoothedMagnitudes[i]);
    Serial.print(", With Gain (");
    Serial.print(bandGains[i]);
    Serial.print("): ");
    Serial.println(gainAdjustedMagnitude);
    #endif

    // Apply noise floor threshold
    if (gainAdjustedMagnitude < NOISE_FLOOR) {
      gainAdjustedMagnitude = 0;
    }

    Serial.print("Magnitude: ");
    Serial.println(gainAdjustedMagnitude);

    // Scale to LED range (0-255)
    int scaledMagnitude = constrain(map(gainAdjustedMagnitude, 0, 50, 0, 1023), 0, 1023);
    
    // Output to LEDs
    analogWrite(OUTPUT_PINS[i], scaledMagnitude);
    if (displayIntensity) {
      analogWrite(INDICATOR_LED_PINS[i], scaledMagnitude);
    }

    // Debug output for serial plotter
    #if PRINT_BANDS
    Serial.print("freq_bin_");
    Serial.print((int)highFreq);
    Serial.print(":");
    Serial.print(scaledMagnitude);
    if (i < NUM_BINS - 1) {
      Serial.print(",");
    }
    #endif
  }

  #if PRINT_BANDS
  Serial.println();
  #endif
}

void loop() {
  bool configSwitchState = digitalRead(CONFIG_SWITCH_PIN);
  if (!configMode && configSwitchState) {
    // Enter config mode
    configMode = true;
    currentBand = 0;
    handleConfigMode();
  } else if (configSwitchState) {
    handleConfigMode();
  } else if (configMode && !configSwitchState) {
    configMode = false;
    currentBand = -1;
    saveConfiguration();
  }

  if (!configMode) {
    // Update LEDs with audio band magnitudes
    displayIntensity = true;
  } else {
    displayIntensity = false;
    // Update LEDs to show current band in config mode
    updateLEDs();
  }

  collectSamples();
  performFFT();
  outputFFT();
}