#include "Arduino_BMI270_BMM150.h"

#define Left_Motor_Red 5
#define Left_Motor_Black 4
#define Right_Motor_Red 3
#define Right_Motor_Black 2


/////////////////////////////// PWM Config ///////////////////////////////

// Define Arduino pins (corresponding to nRF52840 GPIOs)
#define PWM_PIN_D2  11  // P1.11 (Arduino D2)
#define PWM_PIN_D3  12  // P1.12 (Arduino D3)
#define PWM_PIN_D4  15  // P1.15 (Arduino D4)
#define PWM_PIN_D5  13  // P1.13 (Arduino D5)

// PWM values (initialized to 25%, 50%, 75%, and 10%)
uint16_t pwm_values[] = {0 | (1 << 15), 0 | (1 << 15), 0 | (1 << 15), 0 | (1 << 15)}; 

//////////////////////////////////////////////////////////////////////////


/////////////////////////////// Sensor Stuff ///////////////////////////////
#define k 0.50
float theta = 0;
unsigned long ct = 0;

int task = 0;
String input;

class coords {
    public:
        float x;
        float y;
        float z;
        float theta_s;
};

// function to read gyroscope and accelerometer value
inline void gyroscope(coords & object) {
    float dt;
    // gets time for ∆t
    dt = (float) (micros() - ct) / 1000000;
    // sets new current time
    ct = micros();

    if (IMU.gyroscopeAvailable()) {
        // reads gyroscope value
        IMU.readGyroscope(object.x, object.y, object.z);
        
        // computes theta using integration
        theta = (theta + object.x * dt);
    }

    if (IMU.accelerationAvailable()) {
        // reads acceleration
        IMU.readAcceleration(object.x, object.y, object.z);
        
        // computes theta values
        object.theta_s = -degrees(atan(object.y / object.z));
    }

    theta = k * theta + (1 - k) * object.theta_s;
}

/////////////////////////////////////////////////////////////////////////////

///////////////////// function for arduino initialization ///////////////////
void setup() {
    // initialize serial
    Serial.begin(9600);
    while(!Serial);
    Serial.setTimeout(10);

    // case if serial does not initialize
    if(!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while(1);
    }

     // Enable the PWM hardware module
     NRF_PWM0->ENABLE = 1;

     // Set PWM mode to UP mode (counts up and resets)
     NRF_PWM0->MODE = PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos;
 
     // Set the prescaler (PWM frequency base) - 16MHz / 16 = 1MHz timer
     NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16;
 
     // Set the maximum counter top value (for a frequency of 1kHz)
     NRF_PWM0->COUNTERTOP = 1000;  // 1 MHz / 1000 = 1 kHz PWM Frequency
 
     // Configure the sequence (4 independent PWM duty cycles)
     NRF_PWM0->SEQ[0].PTR = (uint32_t)pwm_values;  // Point to duty cycle array
     NRF_PWM0->SEQ[0].CNT = 4;  // 4 PWM outputs
     NRF_PWM0->SEQ[0].REFRESH = 0;
     NRF_PWM0->SEQ[0].ENDDELAY = 0;
 
     // Configure PWM output pins
     NRF_PWM0->PSEL.OUT[0] = (1 << 5) | PWM_PIN_D2;  // Select P1.11 (D2)
     NRF_PWM0->PSEL.OUT[1] = (1 << 5) | PWM_PIN_D3;  // Select P1.12 (D3)
     NRF_PWM0->PSEL.OUT[2] = (1 << 5) | PWM_PIN_D4;  // Select P1.15 (D4)
     NRF_PWM0->PSEL.OUT[3] = (1 << 5) | PWM_PIN_D5;  // Select P1.13 (D5)
 
     // Configure PWM decoder (individual duty cycles)
     NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                         (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
 
     // Enable the loop and start the sequence
     NRF_PWM0->LOOP = PWM_LOOP_CNT_Disabled;
     NRF_PWM0->TASKS_SEQSTART[0] = 1;
}


void loop() {
    float Lred, Lblack, Rred, Rblack;
    // creates object for reading values
    coords gyro;
    float pwm, rpml, rpmr;

    // calls functions
    gyroscope(gyro);

    if (Serial.available() > 0) {
        input = Serial.readString();
        input.trim();
        input.toLowerCase();
    }


    if (input == "forward") {
        task = 1;
    }
    else if (input == "reverse") {
        task = 2;
    }
    else if (input == "opposite1") {
        task = 3;
    }
    else if (input == "opposite2") {
        task = 4;
    }
    else if (input == "tilt") {
        task = 5;   
    }


    switch (task) {
        case 1:
            if (input.toInt() <= 100){
                Lred = (1 << 15) | map(input.toInt(), 0, 100, 1000, 0);
                Lblack = (1 << 15) | 1000;
                Rred = (1 << 15) | map(input.toInt(), 0, 100, 1000, 0);
                Rblack = (1 << 15) | 1000;
            }
        break;
        case 2:
            if (input.toInt() <= 100){
                pwm = map(input.toInt(), 0, 100, 0 , 400);
                rpml = 9.89*exp(0.00817*pwm);
                rpmr = 12.1*exp(0.00767*pwm);

                Lred = 0;
                Lblack = rpml;
                Rred = 0;
                Rblack = rpmr;
            }
        break;
        case 3:
            if (input.toInt() <= 100){
                pwm = map(input.toInt(), 0, 100, 0 , 400);
                rpml = 9.89*exp(0.00817*pwm);
                rpmr = 12.1*exp(0.00767*pwm);

                Lred = rpml;
                Lblack = 0;
                Rred = 0;
                Rblack = rpmr;
            }
        break;
        case 4:
            if (input.toInt() <= 100){
                pwm = map(input.toInt(), 0, 100, 0 , 400);
                rpml = 9.89*exp(0.00817*pwm);
                rpmr = 12.1*exp(0.00767*pwm);

                Lred = 0;
                Lblack = rpml;
                Rred = rpmr;
                Rblack = 0;
            }
        break;
        case 5:
            if (theta < 0) {
                Lred = abs(theta/90.0 * 255);
                Lblack = 0;

                Rred = abs(theta/90.0 * 255);
                Rblack = 0;
            } 
            else {
                Lred = 0;
                Lblack = abs(theta/90.0 * 255);

                Rred = 0;
                Rblack = abs(theta/90.0 * 255);
            }
        break;
        default:
            Lred = 0;
            Lblack = 0;
            Rred = 0;
            Rblack = 0;
        break;
    }

    // Modify PWM values dynamically (example: cycling duty cycles)
    pwm_values[0] = Rblack;  // D2
    pwm_values[1] = Rred;  // D3
    pwm_values[2] = Lblack;  // D4
    pwm_values[3] = Lred;  // D5

    // Wait for the PWM sequence to finish before updating
    while (NRF_PWM0->EVENTS_SEQEND[0] == 0);
    NRF_PWM0->EVENTS_SEQEND[0] = 0;  // Clear event flag

    // Restart the PWM sequence with updated values
    NRF_PWM0->TASKS_SEQSTART[0] = 1;


    Serial.print(task);
    Serial.print("\t");

    Serial.print(Lred);
    Serial.print("\t");
    Serial.print(Lblack);
    Serial.print("\t");
    Serial.print(Rred);
    Serial.print("\t");
    Serial.println(Rblack);

}
