/*
Purpose: Contains all of the movement-related functions for the final project
Status: UNTESTED
Notes:
*/
#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Encoder.h>

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;
Adafruit_MPU6050 mpu;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

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

// Buzzer pin which we will use for indicating IMU initialization failure
const unsigned int BUZZ = 26;
const unsigned int BUZZ_CHANNEL = 0;

void degTurnR(float deg, int PWM) {

}
void degTurnL(float deg, int PWM) {

}

// precise movement in a straight line
// mm: millimeters to move forward (+) or backwards (-) by. Set to 0 if you want to use the white box as an end trigger instead
// PWM: speed from 0-255
void straight(float mm, int PWM) {

    // turn on the motors
    ledcWrite(M1_IN_1_CHANNEL, PWM);
    ledcWrite(M1_IN_2_CHANNEL, 0);
    ledcWrite(M2_IN_1_CHANNEL, PWM);
    ledcWrite(M2_IN_2_CHANNEL, 0);

    if (mm != 0) {
        // continue only for set length
        // use IMU to determine distance traveled
    }
    else {
        // continue until it hits a white box
        // use line sensors to determine this
        // if all on, then use brake
    }

}

// follow a path that contains curves.
void curveFollow(int PWM) {
    int kp = 0;
    int ki = 0;
    int kd = 0;
}

// hard stop
void brake() {
    ledcWrite(M1_IN_1_CHANNEL, 512);
    ledcWrite(M1_IN_2_CHANNEL, 512);
    ledcWrite(M2_IN_1_CHANNEL, 512);
    ledcWrite(M2_IN_2_CHANNEL, 512);
}

// section below is copied from provided Lab7 base file:
void configure_imu() {
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      ledcWriteNote(BUZZ_CHANNEL, NOTE_C, 4);
      delay(500);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_G, 4);
      delay(500);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
}

void read_imu(float& w_z) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  w_z = g.gyro.z;
}

void est_imu_bias(float& E_w_z, int N_samples) {
  float E_w_z_acc = 0.0;
  for (unsigned int i = 0; i < N_samples; i++) {
    float w_z;
    read_imu(w_z);
    E_w_z_acc += w_z;
    delay(5);
  }
  E_w_z = E_w_z_acc / N_samples;
}

void configure_motor_pins() {
  ledcSetup(M1_IN_1_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
  ledcSetup(M1_IN_2_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
  ledcSetup(M2_IN_1_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
  ledcSetup(M2_IN_2_CHANNEL, M_PWM_FREQ, M_PWM_BITS);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);
}

// Positive means forward, negative means backwards
void set_motors_pwm(float left_pwm, float right_pwm, int MAX) {
  if (isnan(left_pwm)) left_pwm = 0.0;
  if (left_pwm  >  (float)MAX) left_pwm  =  (float)MAX;
  if (left_pwm  < -1 * (float)MAX) left_pwm  = -1 * (float)MAX;
  if (isnan(right_pwm)) right_pwm = 0.0;
  if (right_pwm >  (float)MAX) right_pwm =  (float)MAX;
  if (right_pwm < -1 * (float)MAX) right_pwm = -1 * (float)MAX;

  if (left_pwm > 0) {
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, (uint32_t)(left_pwm));
  } else {
    ledcWrite(M1_IN_1_CHANNEL, (uint32_t)-left_pwm);
    ledcWrite(M1_IN_2_CHANNEL, 0);
  }

  if (right_pwm > 0) {
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, (uint32_t)(right_pwm));
  } else {
    ledcWrite(M2_IN_1_CHANNEL, (uint32_t)-right_pwm);
    ledcWrite(M2_IN_2_CHANNEL, 0);
  }
}

float update_pid(float dt, float kp, float ki, float kd,
                 float x_d, float x,
                 float& int_e, float abs_int_e_max, // last_x and int_e are updated by this function
                 float& last_x) {
  // Calculate or update intermediates
  float e = x_d - x; // Error

  // Integrate error with anti-windup
  int_e = int_e + e * dt;
  if (int_e >  abs_int_e_max) int_e =  abs_int_e_max;
  if (int_e < -abs_int_e_max) int_e = -abs_int_e_max;

  // Take the "Derivative of the process variable" to avoid derivative spikes if setpoint makes step change
  // with abuse of notation, call this de

  float de = -(x - last_x) / dt;
  last_x = x;

  float u = kp * e + ki * int_e + kd * de;
  return u;
}

void line(float t, float& x, float& y) {
  /*
  y = cos(t)/(sin(t)+0.00001);
  x = 0.00001;
  */
 float sin_t = sin(t);
 float den = 1 + sin_t * sin_t;
  x = 0.2 * cos(t) / den;
  y = 0.2 * sin(t) * cos(t) / den;
}

// Signed angle from (x0, y0) to (x1, y1)
// assumes norms of these quantities are precomputed
float signed_angle(float x0, float y0, float n0, float x1, float y1, float n1) {
  float normed_dot = (x1 * x0 + y1 * y0) / (n1 * n0);
  if (normed_dot > 1.0) normed_dot = 1.0; // Possible because of numerical error
  float angle = acosf(normed_dot);

  // use cross product to find direction of rotation
  // https://en.wikipedia.org/wiki/Cross_product#Coordinate_notation
  float s3 = x0 * y1 - x1 * y0;
  if (s3 < 0) angle = -angle;

  return angle;
}

// follow a straight line using path-planning
// based on the provided Lab7 base path-planning code 
// PWM: max motor speed to follow path
// len: length in mm to travel for
void pathPlanS(int PWM, float mm) {
    Encoder enc1(M1_ENC_A, M1_ENC_B);
    Encoder enc2(M2_ENC_A, M2_ENC_B);
    enc1.write(0);
    enc2.write(0);
    long enc1_value = enc1.read();
    long enc2_value = enc2.read();

    int encLim = mm * METERS_PER_TICK * 1000; // input mm converted to an encoder limit

    // Loop period
    int target_period_ms = 2; // Loop takes about 3 ms so a delay of 2 gives 200 Hz or 5ms

    // States used to calculate target velocity and heading
    float scale = 1.0; // speedup factor
    float x0, y0;
    line(0.0, x0, y0);
    float last_x, last_y;
    line(mm, last_x, last_y);
    float last_dx = (x0 - last_x) / ((float)target_period_ms / 1000.0);
    float last_dy = (y0 - last_y) / ((float)target_period_ms / 1000.0);
    float last_target_v = sqrtf(last_dx * last_dx + last_dy * last_dy);
    float target_theta = 0.0; // This is an integrated quantity

    // Motors are controlled by a position PID
    // with inputs interpreted in meters and outputs interpreted in volts
    // integral term has "anti-windup"
    // derivative term uses to derivative of process variable (wheel position)
    // instead of derivative of error in order to avoid "derivative kick"

    float kp_left = 400.0;
    float ki_left = 20.0;
    float kd_left = 20.0;
    float kf_left = 10.0;
    float target_pos_left  = 0.0;
    float last_pos_left = 0.0;
    float last_error_left = 0.0;
    float integral_error_pos_left = 0.0;
    float max_integral_error_pos_left = 1.0 * 8.0 / ki_left; // Max effect is the nominal battery voltage

    float kp_right = 400.0;
    float ki_right = 20.0;
    float kd_right = 20.0;
    float kf_right = 10.0;
    float last_pos_right = 0.0;
    float target_pos_right = 0.0;
    float integral_error_pos_right = 0.0;
    float max_integral_error_pos_right = 1.0 * 8.0 / ki_right; // Max effect is the nominal battery voltage

    // IMU Orientation variables
    float theta = 0.0;
    float bias_omega;
    // Gain applied to heading error when offseting target motor velocities
    // currently set to 360 deg/s compensation for 90 degrees of error
    float ktheta = (2 * 3.14159) / (90.0 * 3.14159 / 180.0);
    est_imu_bias(bias_omega, 500);// Could be expanded for more quantities

    // The real "loop()"
    // time starts from 0
    float start_t = (float)micros() / 1000000.0;
    float last_t = -target_period_ms / 1000.0; // Offset by expected looptime to avoid divide by zero

    int i = 0;

    float t;
    float dt;
    float pos_left;
    float pos_right;
    float omega;
    float x, y;
    float dx, dy;
    float target_v;
    float target_omega;

    float error_theta_z;
    float requested_v;
    float requested_w;

    float target_v_left;
    float target_v_right;

    float left_voltage;
    float left_pwm;

    float right_voltage;
    float right_pwm;

    while ( (pos_left*1000 < mm) && (pos_right*1000 < mm) ) {
        // Get the time elapsed
        t = ((float)micros()) / 1000000.0 - start_t;
        dt = ((float)(t - last_t)); // Calculate time since last update
        // Serial.print("t "); Serial.print(t);
        //Serial.print(" dt "); Serial.print(dt * 1000.0);
        last_t = t;

        // Get the distances the wheels have traveled in meters
        // positive is forward
        pos_left  =  (float)enc1.read() * METERS_PER_TICK;
        pos_right = -(float)enc2.read() * METERS_PER_TICK; // Take negative because right counts upwards when rotating backwards

        // TODO Battery voltage compensation, the voltage sense on my mouse is broken for some reason
        // int counts = analogRead(VCC_SENSE);
        // float battery_voltage = counts * ADC_COUNTS_TO_VOLTS;
        // if (battery_voltage <= 0) Serial.println("BATTERY INVALID");

        // Read IMU and update estimate of heading
        // positive is counter clockwise
        omega;
        read_imu(omega); // Could be expanded to read more things
        omega -= bias_omega; // Remove the constant bias measured in the beginning
        theta = theta + omega * dt;
        // Serial.print(" omega "); Serial.print(omega);
        // Serial.print(" theta "); Serial.print(theta);

        // Serial.print(" last_x "); Serial.print(last_x);
        // Serial.print(" last_y "); Serial.print(last_y);
        // Serial.print(" last_dx "); Serial.print(last_dx);
        // Serial.print(" last_dy "); Serial.print(last_dy);
        // Serial.print(" last tv "); Serial.print(last_target_v);

        x, y;
        line(t, x, y);

        // Serial.print(" x "); Serial.print(x);
        // Serial.print(" y "); Serial.print(y);

        dx = (x - last_x) / dt;
        dy = (y - last_y) / dt;
        target_v = sqrtf(dx * dx + dy * dy); // forward velocity

        // Serial.print(" dx "); Serial.print(dx);
        // Serial.print(" dy "); Serial.print(dy);
        // Serial.print(" tv "); Serial.print(target_v);

        // Compute the change in heading using the normalized dot product between the current and last velocity vector
        // using this method instead of atan2 allows easy smooth handling of angles outsides of -pi / pi at the cost of
        // a slow drift defined by numerical precision
        target_omega = signed_angle(last_dx, last_dy, last_target_v, dx, dy, target_v) / dt;
        target_theta = target_theta + target_omega * dt;

        // Serial.print(" target_omega "); Serial.print(target_omega);
        // Serial.print(" t theta "); Serial.print(target_theta);

        last_x = x;
        last_y = y;
        last_dx = dx;
        last_dy = dy;
        last_target_v = target_v;

        // Calculate target motor speeds from target forward speed and target heading
        // Could also include target path length traveled and target angular velocity
        error_theta_z = target_theta - theta;
        requested_v = target_v;
        requested_w = ktheta * error_theta_z; 

        target_v_left  = requested_v - TURNING_RADIUS_METERS * requested_w;
        target_v_right = requested_v + TURNING_RADIUS_METERS * requested_w;
        target_pos_left  = target_pos_left  + dt * target_v_left;
        target_pos_right = target_pos_right + dt * target_v_right;

        // Serial.print(" tpl "); Serial.print(target_pos_left);
        // Serial.print(" pl "); Serial.print(pos_left);
        // Serial.print(" tpr "); Serial.print(target_pos_right);
        // Serial.print(" pr "); Serial.print(pos_right);

        // Adding Feedforward control 
        // Left motor position PID
        left_voltage = update_pid(dt, kp_left, ki_left, kd_left,
                                        target_pos_left, pos_left,
                                        integral_error_pos_left, max_integral_error_pos_left,
                                        last_pos_left);

        left_voltage = left_voltage + kf_left * target_v_left;
        left_pwm = (float)PWM * (left_voltage / 8.0); // TODO use actual battery voltage

        // Right motor position PID
        right_voltage = update_pid(dt, kp_right, ki_right, kd_right,
                                            target_pos_right, pos_right,
                                            integral_error_pos_right, max_integral_error_pos_right,
                                            last_pos_right);
        left_voltage = right_voltage + kf_right * target_v_right;
        right_pwm = (float)PWM * (right_voltage / 8.0); // TODO use actual battery voltage

        // Serial.print(" l voltage " ); Serial.print(left_voltage);
        // Serial.print(" r voltage " ); Serial.print(right_voltage);

        set_motors_pwm(left_pwm, right_pwm, PWM);

        //delay(target_period_ms);

        Serial.print(" Pos Left " ); Serial.print(pos_left*1000);
        Serial.print("| Pos Right " ); Serial.print(pos_right*1000);
        Serial.print("| mm " ); Serial.print(mm);
        Serial.print("\n");

    }
    Serial.print("Attempting brake...\n");
    Serial.print(i);
    Serial.print("\n");
    Serial.print((uint32_t)(left_pwm));
    Serial.print("\n");
    Serial.print((uint32_t)(right_pwm));
    Serial.print("\n");
    brake();
}
// ---------------------------------------------------------------------