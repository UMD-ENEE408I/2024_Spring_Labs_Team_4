/*
Purpose: Contains all of the movement-related functions for the final project
Status: UNTESTED
Notes:
*/

// turn with the rotation centered between the two wheels of the robot 
// deg: degrees to turn by
// PWM: speed from 0-255
void degTurnR(float deg, float PWM) {

}
void degTurnL(float deg, float PWM) {

}

// precise movement in a straight line
// mm: millimeters to move forward (+) or backwards (-) by. Set to 0 if you want to use the white box as an end trigger instead
// PWM: speed from 0-255
void straight(float mm, float PWM) {
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
void curveFollow(float PWM) {
    int kp = 0;
    int ki = 0;
    int kd = 0;
}

// hard stop
void brake() {

}