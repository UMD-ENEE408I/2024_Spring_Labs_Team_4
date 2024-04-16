#include <Arduino.h>
#include <Encoder.h>

#define PI 3.1415926535897932384626433832795

const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

void M1_backward(const int PWM) {
  ledcWrite(M1_IN_1_CHANNEL, PWM);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}

void M1_forward(const int PWM) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, PWM);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, 512);
  ledcWrite(M1_IN_2_CHANNEL, 512);
}

void M2_backward(const int PWM) {
  ledcWrite(M2_IN_1_CHANNEL, PWM);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M2_forward(const int PWM) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, PWM);
}

void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, 512);
  ledcWrite(M2_IN_2_CHANNEL, 512);
}

void brake() {
    M1_stop();
    M2_stop();
}

// uses differential drive modeling
// https://www.youtube.com/watch?v=LrsTBWf6Wsc
// R: radius of arc in m from the edge of the robot (not center)
// DEG: degrees to rotate in arc
// MAX_PWM: maximum speed motor can go at
// LR: left (1) or right (0) arc
void arc(const float R, const float DEG, const int MAX_PWM, const bool LR, Encoder& enc1, Encoder& enc2) {
    const float M_PER_TICK = (PI * 0.032) / 360.0; // meters per tick
    const float REF_R = 4.3 / 100.0; // referance point in m: distance of wheel edge from center
    float sl, sr, s, slTick, srTick;
    int lPWM, rPWM;
    const float RAD = DEG * (PI/180);
    
    // NOTE: there are aproximately 360 ticks per rotation

    if (LR) {
        // the following is for a + turn (ie, left)
        sl = R*RAD; // distance left motor needs to travel in m
        sr = (R + 2*REF_R)*RAD; // distance right motor needs to travel in m
        float rat = sl/sr; // motors will need to move in ratio to eachother

        // for a left turn, the right motor will need to be going faster
        rPWM = MAX_PWM;
        lPWM = MAX_PWM * rat;
    }
    else {
        // the following is for a - turn (ie, right)
        sl = (R + 2*REF_R)*RAD;
        sr = R*RAD;
        float rat = sr/sl;

        rPWM = MAX_PWM * rat;
        lPWM = MAX_PWM;
    }

    srTick = sr / M_PER_TICK;
    slTick = sl / M_PER_TICK;
    s = (sr + sl) / 2; // total distance the robot will travel
    float sNow = 0;

    // use the ecoders to check positioning
    enc1.write(0);
    enc2.write(0);
    if (MAX_PWM > 0) {
        M1_forward(abs(lPWM));
        M2_forward(abs(rPWM));
    }
    else {
        M1_backward(abs(lPWM));
        M2_backward(abs(rPWM));
    }
    while (sNow < s) {
        sNow = (abs(enc1.read()) + abs(enc2.read())) * M_PER_TICK;
    }
    brake();
}

// rotate the robot on its center
// R: radius of arc in m from the edge of the robot (not center)
// DEG: degrees to rotate in arc
// MAX_PWM: maximum speed motor can go at
// LR: left (1) or right (0) arc
void spin(const float DEG, const int MAX_PWM, const bool LR, Encoder& enc1, Encoder& enc2) {
    const float M_PER_TICK = (PI * 0.032) / 360.0; // meters per tick
    const float REF_R = 4.3 / 100.0; // referance point in m: distance of wheel edge from center
    float s, sTick;
    int lPWM, rPWM;
    const float RAD = DEG * (PI/180);
    
    // NOTE: there are aproximately 360 ticks per rotation

    s = REF_R * RAD; // distance each motor needs to travel in m
    sTick = s / M_PER_TICK;

    float sNow = 0;
    bool lStat, rStat;
    lStat = 0;
    rStat = 0;

    // use the ecoders to check positioning
    enc1.write(0);
    enc2.write(0);
    if (LR) {
        M1_forward(abs(MAX_PWM));
        M2_backward(abs(MAX_PWM));
    }
    else {
        M2_forward(abs(MAX_PWM));
        M1_backward(abs(MAX_PWM));
    }

    // TODO: compensate for the motors going at different speeds (may require controller)

    while (true) {
        if (!(lStat) && (abs(enc1.read()) > sTick)) {
            M1_stop();
            lStat = 1;
        }
        if (!(rStat) && (abs(enc2.read()) > sTick)) {
            M2_stop();
            rStat = 1;
        }
        if (lStat && rStat) {
            break;
        }
    }
}

// avoid an obstacle by performing an arc move of specified size
// R: radius of arc in m from the edge of the robot (not center)
// MAX_PWM: maximum speed motor can go at
// LR: left (1) or right (0) arc
void dodge(const float R, const int MAX_PWM, const bool LR, Encoder& enc1, Encoder& enc2) {
    spin(90, MAX_PWM, LR, enc1, enc2); // spin out, away from the line
    brake();
    delay(100);
    arc(0.5, 180, MAX_PWM, !(LR), enc1, enc2); // arc in the opposite direction
    brake();
    delay(100);
    spin(90, MAX_PWM, LR, enc1, enc2); // return back to the straight
    brake();
}

// move in a straight line without the aid of line following (to go backward, use negative PWM)
// PID controller https://microcontrollerslab.com/pid-controller-implementation-using-arduino/
// kp: proportional gain
// ki: integral gain
// kd: derivative gain
// mm: distance to travel in mm
// MAX_PWM: target speed for the bot (is also treated as the maximum allowable speed) 
void straight(int kp, int ki, int kd, int mm, int MAX_PWM, Encoder& enc1, Encoder& enc2) {
    const float M_PER_TICK = (PI * 0.032) / 360.0; // meters per tick
    const float REF_R = 4.3 / 100.0; // referance point in m: distance of wheel edge from center
    float sTick = (mm/1000.0) / M_PER_TICK;

    float error, errorTotal, control, t, tStart, tPrev, tNow;
    const float tInt = 100 * (1/1000000.0); // integration time in s
    int lPWM, rPWM;
    lPWM = MAX_PWM;
    rPWM = MAX_PWM;
    tPrev = 0.0;
    error = 0.0;
    errorTotal = 0.0;

    enc1.write(0);
    enc2.write(0);
    while (true) {
        // update PID
        tNow = micros() / 1000000.0;
        error = enc1.read() + enc2.read(); // one is always negative if the motors are going in the same direction
        errorTotal += error;
        control = kp * error + ki * errorTotal * tInt + kd * (tNow - tPrev)/tInt;

        lPWM = MAX_PWM - control;
        rPWM = MAX_PWM + control;

        if (MAX_PWM > 0) {
            M1_forward((int)min(lPWM, MAX_PWM));
            M2_forward((int)min(rPWM, MAX_PWM));
        }
        else {
            M1_backward((int)min(lPWM, MAX_PWM));
            M2_backward((int)min(rPWM, MAX_PWM));
        }

        tPrev = tNow;

        // check the distance
        if ((abs(enc1.read()) > sTick) || (abs(enc2.read()) > sTick)) {
            brake();
            break;
        }
        Serial.print(sTick);
        Serial.print("\t");
        Serial.print(enc1.read());
        Serial.print("\t");
        Serial.print(enc2.read());
        Serial.print("\t");
        Serial.print(error);
        Serial.print("\t");
        Serial.print(lPWM);
        Serial.print("\t");
        Serial.print(rPWM);
        Serial.print("\t");
        Serial.print(control);
        Serial.println();
    }
}