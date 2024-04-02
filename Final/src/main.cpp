#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Encoder.h>

// IMU (rotation rate and acceleration)
Adafruit_MPU6050 mpu;

#include "move.h"

void setup() {
  const unsigned int ADC_1_CS = 2;
  const unsigned int ADC_2_CS = 17;

  // fix the pin that is set high during boot
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  // Disalbe the lightbar ADC chips so they don't hold the SPI bus used by the IMU
  pinMode(ADC_1_CS, OUTPUT);
  pinMode(ADC_2_CS, OUTPUT);
  digitalWrite(ADC_1_CS, HIGH);
  digitalWrite(ADC_2_CS, HIGH);

  // baud rate needed to be adjusted from 115200 used previously to 9600 for some reason
  Serial.begin(9600);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      ledcWriteNote(0, NOTE_C, 4);
      delay(500);
      ledcWriteNote(0, NOTE_G, 4);
      delay(500);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(".");
  delay(1000);
}