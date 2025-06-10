#include "Arduino_HS300x.h"

// function for arduino initialization
void setup() {
    // initialize serial
    Serial.begin(9600);
    while(!Serial);

    // case if serial does not initialize
    if(!HS300x.begin()) {
        Serial.println("Failed to initialize HS300x!");
        while(1);
    }
}

void loop() {
    float temp;
    
    // reads temp value
    temp = HS300x.readTemperature(CELSIUS);

    // sends value to serial
    Serial.println(temp);

}
