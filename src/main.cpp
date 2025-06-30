#include <Arduino.h>
#include <arduinoFFT.h>

#define MIN_POT_DIFF_READING 25

// PIN configuration
#define THRESHOLD_POTENTIOMETER_PIN 33
#define SMOOTHING_POTENTIOMETER_PIN 34
#define GAIN_POTENTIOMETER_PIN 35
#define OTHER_POTENTIOMETER_PIN 39
#define GENERAL_USE_BUTTON 16
#define AUDIO_INPUT_PIN 32
#define LED_ARR_LEN 8
constexpr int INDICATOR_LED_PINS[LED_ARR_LEN] = {21, 22, 23, 25, 26, 27, 5, 17};
constexpr int OUTPUT_PINS[LED_ARR_LEN] = {2, 4, 12, 13, 14, 15, 18, 19};

// FFT configuration
#define NUM_SAMPLES 512     // Number of samples used for FFT analysis
#define SAMPLING_RATE 22020 // Sampling rate of the audio input in Hz
#define NUM_BINS 8
constexpr int FREQ_BANDS[NUM_BINS + 1] = {0, 200, 400, 800, 1200, 2000, 3000, 5000, 8000};
int startBins[NUM_BINS];
int endBins[NUM_BINS];
unsigned long samplingPeriodUs = 1000000 / SAMPLING_RATE;

// State variables
int globalPeak = 1;
const int globalPeakDecayFactor = 90;
static float smoothedMagnitudes[NUM_BINS] = {0};
float vReal[NUM_SAMPLES], vImag[NUM_SAMPLES];
unsigned long lastLEDUpdate = 0;
const unsigned long ledUpdateInterval = 2;

// Potentiometer Variables
int gainFactor = 100;
int prevGainReading = 0;
int magnitudeThresholdMultiplier = 1; // Multiplier for the magnitude threshold
int prevThresholdMultiplierReading = 0;
int smoothingFactor = 95; // Lower value = more smoothing
int prevSmoothingReading = 0;


// Constructs
ArduinoFFT<float> FFT(vReal, vImag, NUM_SAMPLES, SAMPLING_RATE);
SemaphoreHandle_t fftSemaphore; // Semaphore for FFT and LED synchronization

void calculateBins()
{
  int freqResolution = SAMPLING_RATE / NUM_SAMPLES; // Frequency resolution per bin

  for (int i = 0; i < NUM_BINS; i++)
  {
    startBins[i] = FREQ_BANDS[i] / freqResolution;
    endBins[i] = FREQ_BANDS[i + 1] / freqResolution - 1;

    // Ensure no overlap
    if (i > 0 && startBins[i] <= endBins[i - 1])
    {
      startBins[i] = endBins[i - 1] + 1;
    }

    // Clamp bins to valid range
    if (endBins[i] >= NUM_SAMPLES / 2)
    {
      endBins[i] = NUM_SAMPLES / 2 - 1;
    }
  }
}

void collectSamples()
{
  unsigned long startSampleTime = micros(); // Record the start time
  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    vReal[i] = analogRead(AUDIO_INPUT_PIN);
    vImag[i] = 0;

    // Calculate the next sample time
    unsigned long nextSampleTime = startSampleTime + (i + 1) * samplingPeriodUs;

    // Wait until the next sample time
    while (micros() < nextSampleTime)
    {
      // Busy-wait for precise timing
    }
  }
}


void performFFT()
{
  FFT.windowing(vReal, NUM_SAMPLES, FFT_WIN_TYP_BLACKMAN_HARRIS, FFT_FORWARD);
  // FFT.windowing(vReal, NUM_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, NUM_SAMPLES);
}

int lastDebugPrint = 0;
int highestVReal = 0;

void outputFFT()
{
  int magnitudeThreshold = (globalPeak * magnitudeThresholdMultiplier) / 100;

  // Apply decay to globalPeak
  globalPeak = (globalPeak * globalPeakDecayFactor) / 100;

  for (int i = 0; i < NUM_BINS; i++)
  {
    int startBin = startBins[i];
    int endBin = endBins[i];

    float peakMagnitude = 0;
    for (int j = startBin; j <= endBin; j++)
    {
      if (vReal[j] > peakMagnitude)
      {
        peakMagnitude = vReal[j];
      }
    }

    peakMagnitude = (peakMagnitude * gainFactor) / 100;

    if (peakMagnitude > globalPeak)
    {
      globalPeak = peakMagnitude;
    }

    if (peakMagnitude < magnitudeThreshold)
    {
      peakMagnitude = 0;
    }

    // Apply smoothing
    smoothedMagnitudes[i] = (peakMagnitude * smoothingFactor + smoothedMagnitudes[i] * (100 - smoothingFactor)) / 100;
    smoothedMagnitudes[i] = min(smoothedMagnitudes[i], (float)globalPeak);

    // Normalize by global peak
    if (globalPeak < 1) globalPeak = 1; 
    int normalizedMagnitude = (smoothedMagnitudes[i] * 255) / globalPeak;
    
    if (millis() - lastDebugPrint > 50) {
      lastDebugPrint = millis();
      // Serial.println(normalizedMagnitude);
      // Serial.print("Global Peak: ");
      // Serial.println(globalPeak);
      // Serial.print("Smoothed Magnitude: ");
      // Serial.println(smoothedMagnitudes[i]);
      // Serial.print("Normalized Magnitude: ");
      // Serial.println(normalizedMagnitude);
    }

    // if (millis() - lastDebugPrint > 20) {
    //   Serial.print("Band ");
    //   Serial.print(i);
    //   Serial.print(", Magnitude: ");
    //   Serial.println(normalizedMagnitude);
    //   for (int a = 100; a < 110; a++) {
    //     Serial.print("vreal:");
    //     Serial.println(vReal[a]);
    //     if (vReal[a] > highestVReal) {
    //       highestVReal = vReal[a];
    //       Serial.print("Highest vReal: ");
    //       Serial.println(highestVReal);
    //     }
    //   }
    //   lastDebugPrint = millis();
    // }

    // Light up the LEDs
    analogWrite(OUTPUT_PINS[i], normalizedMagnitude);
    analogWrite(INDICATOR_LED_PINS[i], normalizedMagnitude);
  }
}

void fftTask(void *parameter)
{
  while (true)
  {
    collectSamples();
    performFFT();

    xSemaphoreGive(fftSemaphore);       // Signal the LED task to update
    vTaskDelay(1 / portTICK_PERIOD_MS); // Important to keep scheduler happy
  }
}

void ledTask(void *parameter)
{
  while (true)
  {
    // Wait for FFT task to finish
    if (xSemaphoreTake(fftSemaphore, portMAX_DELAY) == pdTRUE)
    {
      if (millis() - lastLEDUpdate > ledUpdateInterval)
      {
        lastLEDUpdate = millis();
        outputFFT();
      }
    }

    vTaskDelay(1 / portTICK_PERIOD_MS); // Important to keep scheduler happy
  }
}

void configTask(void *parameter)
{
  while (true)
  {
    int thresholdPotValue = analogRead(THRESHOLD_POTENTIOMETER_PIN);
    int gainPotValue = analogRead(GAIN_POTENTIOMETER_PIN);
    int smoothingPotValue = analogRead(SMOOTHING_POTENTIOMETER_PIN);
    int otherPotValue = analogRead(OTHER_POTENTIOMETER_PIN);

    if (abs(thresholdPotValue - prevThresholdMultiplierReading) > MIN_POT_DIFF_READING) {
      prevThresholdMultiplierReading = thresholdPotValue;
      magnitudeThresholdMultiplier = map(thresholdPotValue, 0, 1023, 1, 99);
      // Serial.print("Threshold Multiplier: ");
      // Serial.println(magnitudeThresholdMultiplier);
    }

    if (abs(smoothingPotValue - prevSmoothingReading) > MIN_POT_DIFF_READING) {
      prevSmoothingReading = smoothingPotValue;
      smoothingFactor = map(smoothingPotValue, 0, 1023, 1, 99);
      // Serial.print("Smoothing: ");
      // Serial.println(smoothingFactor);
    }

    if (abs(gainPotValue - prevGainReading) > MIN_POT_DIFF_READING) {
      prevGainReading = gainPotValue;
      gainFactor = map(gainPotValue, 0, 1023, 100, 500);
      // Serial.print("Gain Factor: ");
      // Serial.println(gainFactor);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS); // Important to keep scheduler happy
  }
}

void setup()
{
  Serial.begin(115200);

  // Set ADC resolution to 10 bits (values will range from 0 to 1023)
  analogReadResolution(10);

  pinMode(AUDIO_INPUT_PIN, INPUT_PULLDOWN);
  pinMode(GAIN_POTENTIOMETER_PIN, INPUT_PULLDOWN);
  pinMode(SMOOTHING_POTENTIOMETER_PIN, INPUT_PULLDOWN);
  pinMode(THRESHOLD_POTENTIOMETER_PIN, INPUT_PULLDOWN);
  pinMode(OTHER_POTENTIOMETER_PIN, INPUT_PULLDOWN);
  pinMode(GENERAL_USE_BUTTON, INPUT_PULLDOWN);
  for (int i = 0; i < NUM_BINS; i++)
  {
    pinMode(INDICATOR_LED_PINS[i], OUTPUT);
    pinMode(OUTPUT_PINS[i], OUTPUT);
  }
  fftSemaphore = xSemaphoreCreateBinary();
  calculateBins();
  xTaskCreatePinnedToCore(fftTask, "FFT Task", 4096, NULL, 2, NULL, 0);       // Core 0
  xTaskCreatePinnedToCore(ledTask, "LED Task", 2048, NULL, 1, NULL, 1);       // Core 1
  xTaskCreatePinnedToCore(configTask, "Config Task", 2048, NULL, 1, NULL, 1); // Core 1
}

void loop()
{
  // Empty loop
}
