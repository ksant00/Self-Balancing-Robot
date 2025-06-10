#include "Arduino_BMI270_BMM150.h"

class coords {
    public:
        float x;
        float y;
        float z;
        float theta_s;
        float theta_l;
};

// function to read acceleration value
inline void acceleration(coords & object) {
    if (IMU.accelerationAvailable()) {
        // reads acceleration
        IMU.readAcceleration(object.x, object.y, object.z);
        
        // computes theta values
        object.theta_l = degrees(atan(object.x / object.z));
        object.theta_s = degrees(atan(object.y / object.z));
    }
}

// function to send to serial
inline void serial_monitor(coords & object) {
    Serial.print(object.theta_s);
    Serial.print(",");
    Serial.print(object.theta_l);
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
    coords accel;

    // calls functions
    acceleration(accel);
    serial_monitor(accel);
}
