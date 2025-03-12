#include <Arduino.h>
#include <arduinoFFT.h>

// Enable printing of frequency bands for debugging
// Format is compatible with VSCode extension "serial-plotter"
// Extension ID: badlogicgames.serial-plotter
const bool PRINT_BANDS = false;

const int numSamples = 256; // number of samples to collect before performing FFT
const int samplingRate = 40000; // sampling rate in Hz (e.g. 1000 Hz = 1 kHz)
const int fftSize = 256; // size of the FFT (must be a power of 2)
const int FREQ_BANDS[] = {200, 500, 1000, 1500, 2000, 4000, 6000, 8000, 10000};
const int OUTPUT_PINS[] = {4, 5, 12, 13, 14, 16, 0, 2};  // Adjust pins as needed
const int numBins = 8;
const int binSize = fftSize / 2 / numBins;

const float OUTPUT_GAIN = 1;  // Adjust as needed

// Smoothing variables
float lastMagnitudes[numBins] = {0};
const float SMOOTHING_FACTOR = 0.1;  // Adjust between 0-1 (higher = smoother)

double vReal[numSamples];
double vImag[numSamples];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, numSamples, samplingRate);

unsigned long lastReading = 0;
const long readingInterval = 10;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
  // Configure output pins
  for(int i = 0; i < numBins; i++) {
    pinMode(OUTPUT_PINS[i], OUTPUT);
  }
}

void collectSamples() {
  for (int i = 0; i < numSamples; i++) {
    vReal[i] = analogRead(A0);
    vImag[i] = 0; // imaginary component is 0 for real-valued input
    delayMicroseconds(1000 / samplingRate); // wait for next sample
  }
}

void performFFT() {
  FFT.windowing(vReal, numSamples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, numSamples, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, numSamples);
}

void outputFFT() {
  if (PRINT_BANDS) {
    Serial.print(">");
  }

  // For each frequency band
  for (int i = 0; i < numBins; i++) {
    float lowFreq = FREQ_BANDS[i];
    float highFreq = FREQ_BANDS[i + 1];
    
    // Convert frequencies to FFT bins
    int startBin = lowFreq * fftSize / samplingRate;
    int endBin = highFreq * fftSize / samplingRate;
    
    // Ensure we don't exceed array bounds
    startBin = max(1, min(startBin, fftSize/2 - 1));
    endBin = max(1, min(endBin, fftSize/2));
    
    // Calculate magnitude for this frequency band
    float magnitude = 0;
    for (int j = startBin; j < endBin; j++) {
      magnitude += vReal[j];
    }
    // Average the magnitude
    magnitude /= (endBin - startBin);
    
    // Optional: Apply logarithmic scaling to magnitude
    // magnitude = magnitude > 0 ? log10(magnitude) * 20 : 0;  // Convert to dB scale

    // Apply smoothing
    magnitude = (magnitude * (1 - SMOOTHING_FACTOR)) + (lastMagnitudes[i] * SMOOTHING_FACTOR);
    lastMagnitudes[i] = magnitude;

    // Scale magnitude to 0-255 range
    // You might need to adjust these values based on your observed magnitude ranges
    float scaledMagnitude = constrain(map(magnitude * OUTPUT_GAIN, 0, 100, 0, 255), 0, 255);
    
    // Output to pin
    analogWrite(OUTPUT_PINS[i], scaledMagnitude);

    if (PRINT_BANDS) {
      Serial.print("freq_bin_");
      Serial.print((int)highFreq);  // Label bin by its upper frequency
      Serial.print(":");
      Serial.print(scaledMagnitude);
      if (i < numBins - 1) {
        Serial.print(",");
      }
    }
  }
  if (PRINT_BANDS) {
    Serial.println();
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastReading >= readingInterval) {
    lastReading = currentMillis;
  }

  collectSamples();
  performFFT();
  outputFFT();
}