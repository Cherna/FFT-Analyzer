# Audio Frequency Analyzer with ESP8266

## Description
This project uses an ESP8266 microcontroller to perform real-time audio frequency analysis using Fast Fourier Transform (FFT). It takes audio input through an analog pin (A0) and splits the signal into 8 frequency bands, which can be used to control LED strips or other visualization devices.

## Hardware Requirements
- ESP8266 board
- Analog audio input source connected to A0
- 8 output pins for frequency band visualization

## Software Dependencies
- PlatformIO
- Arduino framework
- arduinoFFT library (v2.0.4)

## Installation
1. Clone this repository
2. Open the project in PlatformIO
3. Let PlatformIO install the required dependencies as specified in the `platform.ini` file.
4. Upload to your ESP8266 board

## How It Works
The system:
1. Samples analog audio input at a defined sampling rate
2. Performs FFT analysis on the samples
3. Splits the frequency spectrum into 8 bands
4. Outputs the frequency band intensities through GPIO pins
5. Can also output debug information via Serial at 115200 baud

## Contributing
Feel free to submit issues and pull requests.