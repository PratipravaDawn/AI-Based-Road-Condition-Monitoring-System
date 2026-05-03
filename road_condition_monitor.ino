#include <BluetoothSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include "pothole_model.h" // Your generated model from Colab

BluetoothSerial SerialBT;
MPU6050 mpu;

// Offsets
float x_offset = 0.1088;
float y_offset = -0.179;
float z_offset = -0.741;

// Speed from mobile
float speed = 0;

// ML Windowing Variables
const int WINDOW_SIZE = 10;
float bufferX[WINDOW_SIZE], bufferY[WINDOW_SIZE], bufferZ[WINDOW_SIZE];
int buffer_count = 0;

// Timing
unsigned long lastTime = 0;
unsigned long interval = 200;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Pothole_AI");

  Wire.begin(21, 22);
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 failed!");
    while (1);
  }
}

// MATH: Standard Deviation
float get_std(float data[]) {
  float sum = 0, mean = 0, sq_sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) sum += data[i];
  mean = sum / WINDOW_SIZE;
  for (int i = 0; i < WINDOW_SIZE; i++) sq_sum += pow(data[i] - mean, 2);
  return sqrt(sq_sum / WINDOW_SIZE);
}

// MATH: Peak-to-Peak
float get_p2p(float data[]) {
  float mn = data[0], mx = data[0];
  for (int i = 1; i < WINDOW_SIZE; i++) {
    if (data[i] < mn) mn = data[i];
    if (data[i] > mx) mx = data[i];
  }
  return mx - mn;
}

void loop() {
  unsigned long currentTime = millis();

  // 1. Receive Speed from App
  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');
    if (input.startsWith("S,")) {
      speed = input.substring(2).toFloat();
      // Adjust sampling based on speed
      if (speed < 10) interval = 500;
      else if (speed < 40) interval = 100;
      else interval = 50; 
    }
  }

  // 2. Sample Data
  if (currentTime - lastTime >= interval) {
    // Record exactly when we started this window's last sample
    unsigned long samplingFinishedTime = millis(); 
    lastTime = currentTime;

    // ... (MPU6050 reading and buffer shifting logic same as before)

    buffer_count++;

    if (buffer_count >= WINDOW_SIZE && speed > 5.0) {
      
      double features[4] = {
        (double)get_std(bufferZ),
        (double)get_p2p(bufferZ),
        (double)get_std(bufferX),
        (double)get_std(bufferY)
      };

      double result[3];
      score(features, result);

      int prediction = 0;
      if (result[1] > result[0]) prediction = 1;
      if (result[2] > result[1] && result[2] > result[0]) prediction = 2;

      // Calculate how "old" the pothole is
      // It roughly happened in the middle of our window
      unsigned long timeOffset = (millis() - samplingFinishedTime) + (interval * (WINDOW_SIZE / 2));

      // 4. Send Packet: [Type],[TimeOffset]
      if (prediction == 2) {
        // Example output: "P,250" (Pothole happened 250ms ago)
        SerialBT.print("P,");
        SerialBT.println(timeOffset);
      } 
      // For Bad/Good roads, we don't usually need micro-accuracy, 
      // but you can send "B,0" or "G,0" if you want.
      else if (prediction == 1) {
        SerialBT.println("B,0");
      } else {
        SerialBT.println("G,0");
      }
    }
  }
}