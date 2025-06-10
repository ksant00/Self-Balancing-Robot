#include "Arduino_BMI270_BMM150.h"

float thetax = 0;
float thetay = 0;

class coords {
    public:
        float x;
        float y;
        float z;
        float theta_s;
        float theta_l;
};

// function to read gyroscope value
inline void gyroscope(coords & object) {
    // gets sampling rate for ∆t
    float dt = float(IMU.gyroscopeSampleRate());

    if (IMU.gyroscopeAvailable()) {
        // reads gyroscope value
        IMU.readGyroscope(object.x, object.y, object.z);
        
        // computes theta using integration
        thetax += object.x/dt;
        thetay += object.y/dt;
    }
}

// function to send to serial
inline void serial_monitor() {
    Serial.print(thetax);
    Serial.print("\t");
    Serial.print(",");
    Serial.print(thetay);
    Serial.print("\n");
}

// function for arduino setup
void setup() {
    // initialize serial
    Serial.begin(9600);
    while(!Serial);

    // case if serial does not initialize
    if(!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while(1);
    }
}

// loop function
void loop() {
    // creates object for reading values
    coords gyro;

    // calls functions
    gyroscope(gyro);
    serial_monitor();
}
