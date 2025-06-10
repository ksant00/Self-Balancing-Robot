#include "Arduino_BMI270_BMM150.h"

float theta_ca_l = 0;
float theta_ca_s = 0;
float theta_ca_l_g = 0;
float theta_ca_s_g = 0;
unsigned long ct = 0;

#define k .67

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
        object.theta_s = -degrees(atan(object.y / object.z));
    }
}

// function to read gyroscope value
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
        theta_ca_s = (theta_ca_s + object.x * dt);
        theta_ca_l = (theta_ca_l + object.y * dt);
        theta_ca_s_g += (object.x * dt);
        theta_ca_l_g += (object.y * dt);
    }
}

// function to send to serial
inline void serial_monitor(coords & object) {
    Serial.print(object.theta_s);
    Serial.print(",");
    Serial.print(object.theta_l);
    Serial.print(",");
    Serial.print(theta_ca_s_g);
    Serial.print(",");
    Serial.print(theta_ca_l_g);
    Serial.print(",");
    Serial.print(theta_ca_s);
    Serial.print(",");
    Serial.print(theta_ca_l);
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
    coords accel, gyro;

    // calls functions
    acceleration(accel);

    gyroscope(gyro);
    

    // computes final theta values
    theta_ca_l = k * theta_ca_l + (1 - k) * accel.theta_l;
    theta_ca_s = k * theta_ca_s + (1 - k) * accel.theta_s;

    serial_monitor(accel);
}
