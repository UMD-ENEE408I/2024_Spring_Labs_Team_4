/*
#include "move.h"

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

void M1_backward(int pwm) {
  ledcWrite(M1_IN_1_CHANNEL, pwm);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}

void M1_forward(int pwm) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, pwm);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, PWM_VALUE);
  ledcWrite(M1_IN_2_CHANNEL, PWM_VALUE);
}

void M2_backward(int pwm) {
  ledcWrite(M2_IN_1_CHANNEL, pwm);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M2_forward(int pwm) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, pwm);
}

void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, PWM_VALUE);
  ledcWrite(M2_IN_2_CHANNEL, PWM_VALUE);
}

void setup() {
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  Serial.begin(9600);
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

  pathPlanS(400, 1000);
  brake();
  delay(500);
  Serial.print("\nDone!");
  delay(1000);

  /* draft move function from 04/01/2024 session

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
}
*/