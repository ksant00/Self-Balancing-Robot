#ifndef ROBOT_FUNCTIONS_H
#define ROBOT_FUNCTIONS_H

#include "Arduino_BMI270_BMM150.h"
#include "Arduino.h"
#include <ArduinoBLE.h>
// #include "DFRobot_DF2301Q.h"

// Define Arduino pins (corresponding to nRF52840 GPIOs)
#define PWM_PIN_D2  11  // P1.11 (Arduino D2) | Left Motor Black
#define PWM_PIN_D3  12  // P1.12 (Arduino D3) | Left Motor Red
#define PWM_PIN_D4  15  // P1.15 (Arduino D4) | Right Motor Black
#define PWM_PIN_D5  13  // P1.13 (Arduino D5) | Right Motor Red

#define BUFFER_SIZE 20 // Buffer size for incoming bluetooth message

#define k_comp_fil 0.95 // complementary k proportionality 

// Define UUIDs for the service and characteristic
#define SERVICE_UUID       "00000000-5EA4-4183-85CD-B1EB8D5CFBAD"
#define CHARACTERISTIC_UUID "00000001-5EA4-4183-85CD-B1EB8D5CFBAD"

// Sonar pins
#define sonar1_echo_pin 6
#define sonar1_trig_pin 7
#define sonar2_echo_pin 8
#define sonar2_trig_pin 9


//////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// Global Variables //////////////////////////////////

// PID controller class
class pid_controller{
    public: 
        float K_mast;   // master gain
        float Kp;   // porportional gain
        float Ki;   // integral gain
        float Kd;   // derivative gain

        float setpoint; // desired tilt angle
        float error;        // current error value
        float prev_error;   // previous error value
        float calibration_offset;

        // for derivative low-pass filt time constant if we use
        float tau;

        float proportional; // value of proportional error with gain
        float integral; // accumulated error of integral
        float derivative;   // value of derivative error w ith gain
        float control_sig;  // value of control sigal | sum of all errors

        unsigned long ct; // variable to store current/previous time in microseconds
        float dt; // variable for delta time in seconds

        pid_controller(void);
        void update(void);
};

// IMU class
class complementary_filter {
    public:
        float x;    // temporary value for x-axis
        float y;    // temporary value for y-axis
        float z;    // temporary value for z-axis
        float theta_a;  // value of accelerometer angle
        float theta;    // value of complementary filter angle
        float theta_prev; // value of previous angle

        unsigned long ct; // variable to store current/previous time in microseconds
        float dt; // variable for delta time in seconds

        complementary_filter(void);
        void update(void);
};

// Sonar class
class sonar {
    public:
        volatile unsigned long sonar1_start_time; // Stores start time for echo
        volatile unsigned long sonar2_start_time; // Stores start time for echo
        volatile unsigned long sonar1_end_time; // Stores end time for echo
        volatile unsigned long sonar2_end_time; // Stores end time for echo
        float sonar1_distance; // Distance of object
        float sonar2_distance; // Distance of object

        sonar(void);
        void setup(void);
        void get_distance(void);
};

class app_controls {
    public: 
        short int movement;
        short int stop;
        short int stop_count;
        bool backwards;
        
        app_controls(void);
        void update_movement(void);
        void update_stop(void);
};

extern uint16_t pwm_values[]; // PWM values array
extern pid_controller pid; // PID class
extern complementary_filter comp_fil; // IMU class
extern int task; // Task to keyboard input switch
extern String input; // Input from serial port
extern sonar sonars; // Sonar class
extern app_controls app; // App class
// extern DFRobot_DF2301Q_I2C asr; // Voice module

// Define a custom BLE service and characteristic
extern BLEService customService;
extern BLECharacteristic customCharacteristic;

//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////// Timer ISR(s)  ////////////////////////////////////////

extern "C" {
    void TIMER4_IRQHandler_v(void);
}

extern void sonar1_ISR(void);
extern void sonar2_ISR(void);

/////////////////////////////// Funciton Prototypes  /////////////////////////////////////
void setup_Serial(void);
void setup_IMU(void);
void setup_Timer4(void);
void setup_PWM0(void);
void setup_BLE(void);
void keyboard_test(void);
void update_pwm(void);
void setup_voice(void);

#endif
