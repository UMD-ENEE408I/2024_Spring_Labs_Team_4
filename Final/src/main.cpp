#include "lineFollow.h"

#include <Arduino.h>
#include <Encoder.h>

#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

//const unsigned int PWM_VALUE = 350; // Max PWM given 8 bit resolution

bool status = false;

float error;
float last_error;
float total_error;

int base_pid = 420;

float Kp = 2;
float Ki = 0;
float Kd = 1;

const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 10;

const int M_PWM_FREQ = 2500;
const int M_PWM_BITS = 8;
//const unsigned int MAX_PWM_VALUE = 512; // Max PWM given 8 bit resolution

float METERS_PER_TICK = (3.14159 * 0.032) / 360.0; // The diameter of the wheel is 0.032 not 0.031
float TURNING_RADIUS_METERS = 4.3 / 100.0; // Wheels are about 4.3 cm from pivot point

void setup() {
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  Serial.begin(9600);

  // configure motor pins
  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);

}

void loop() {
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  
  delay(3000);

  /*
  // first line follow part
  straight(5, 0, 30, 50, 420, enc1, enc2); // move up to get out of box
  followLine(40, 0, 300, 420, 0);
  straight(5, 0, 30, 0, 420, enc1, enc2); // move up to get into box
  */
  
  /*
  // dotted line follow
  straight(5, 0, 30, 50, 420, enc1, enc2); // move up to get out of box
  followLine(40, 0, 300, 420, 0);
  straight(5, 0, 30, 0, 420, enc1, enc2); // move up to get into box
  */
  
  /*
  // endor dash
  straight(5, 0, 3, 50, 420, enc1, enc2); // move up to get out of box
  followLine(40, 0, 300, 420, 1); // follow until black detected
  straight(5, 0, 3, 0, 420, enc1, enc2); // got straight until box
  */
  

  brake();
}