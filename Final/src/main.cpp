#include "move.h"

#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Encoder.h>

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;
Adafruit_MPU6050 mpu;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int PWM_VALUE = 512; // Max PWM given 8 bit resolution

bool status = false;

float error;
float last_error;
float total_error;

int base_pid = 450;

float Kp = 0.2;
float Kd = 10;
float Ki = 0;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 10;

const int M_PWM_FREQ = 5000;
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

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);

}

void loop() {
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  
  delay(3000);

  arc(0.001, 90, 512, 0, enc1, enc2);
  delay(500);
  arc(0.1, 180, 512, 1, enc1, enc2);
  delay(500);
  arc(0.001, 90, 512, 0, enc1, enc2);

  /*
  delay(3000);
  spin(90, 480, 1, enc1, enc2);
  delay(500);
  spin(90, -480, 1, enc1, enc2);
  delay(500);
  spin(180, 480, 1, enc1, enc2);
  delay(500);
  spin(180, -480, 1, enc1, enc2);
  delay(500);
  spin(360, 480, 1, enc1, enc2);
  delay(500);
  */


  /* draft straight move function from 04/01/2024 session:

  // Create the encoder objects after the motor has
  // stopped, else some sort exception is triggered
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  enc1.write(0);
  enc2.write(0);

  delay(1000);

  while(true) {
    long enc1_value = enc1.read();
    long enc2_value = enc2.read();

    int t_start = micros();
    int t_end = micros();

    error = enc1_value + enc2_value;
    total_error += error;

    int pid_value = Kp*error + Kd*(error-last_error) + Ki*total_error;
    int right_motor = base_pid + pid_value;
    int left_motor = base_pid - pid_value;

    last_error = error;

    M1_forward(min(left_motor,512));
    M2_forward(min(right_motor,512));

      Serial.print(enc1_value);
      Serial.print("\t");
      Serial.print(enc2_value);
      Serial.print("\t");
      Serial.print(error);
      Serial.println();

  }
  */
}