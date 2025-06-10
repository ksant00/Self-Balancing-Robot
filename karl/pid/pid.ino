#include "robot_functions.h"
#include "DFRobotDFPlayerMini.h"

DFRobotDFPlayerMini mp3_player;

void setup() {
    // setup_Serial(); // Initialize serial communincation
    setup_IMU(); // Initialize IMU
    // setup_Timer4(); // Initialize timer4
    setup_PWM0();   // Initialize PWM0
    pinMode(LED_BUILTIN, OUTPUT); // Initialize the built-in LED to indicate connection status
    setup_BLE(); // Initialize BLE connection
    sonars.setup(); // Initialize tigger pins of sonars to output

    float theta = 0;
    for (int i = 0; i < 10; i ++) {
        while (!IMU.gyroscopeAvailable() && !IMU.accelerationAvailable());
        comp_fil.update();
        theta += comp_fil.theta_a;
    }
    comp_fil.theta = theta / 10.0;
}

void loop() {
    comp_fil.update();  // update IMU reading/angle
    pid.update(); // update PID outout
    update_pwm(); // updates motor pwm values
    keyboard_test(); // gets new K gain values from keyboard
    sonars.get_distance(); // gets distance from sonars

    // Wait for a BLE central to connect
    BLEDevice central = BLE.central();

    if (central) {
        // Serial.print("Connected to central: ");
        // Serial.println(central.address());
        digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

        // Keep running while connected
        while (central.connected()) {
            // Check if the characteristic was written
            if (customCharacteristic.written()) {
            // Get the length of the received data
                int length = customCharacteristic.valueLength();

                // Read the received data
                const unsigned char* receivedData = customCharacteristic.value();

                // Create a properly terminated string
                char receivedString[length + 1]; // +1 for null terminator
                memcpy(receivedString, receivedData, length);
                receivedString[length] = '\0'; // Null-terminate the string

                int temp = atoi(receivedString);
                if (temp > 4) {
                    app.update_extra(temp);
                } else {
                    app.movement = temp;
                    app.update_movement();
                    app.autopilot = 0;
                }

                // Print the received data to the Serial Monitor
                // Serial.print("Received data: ");
                // Serial.println(app.movement);
                
                
                // customCharacteristic.writeValue("Data received"); // Optionally, respond by updating the characteristic's value

            }
            
            comp_fil.update();  // update IMU reading/angle
            pid.update(); // update PID outout
            update_pwm(); // updates motor pwm values
            sonars.get_distance(); // gets distance from sonars

            if (app.autopilot) {
                app.update_autopilot();
                app.update_movement();
            } else {
                if (sonars.sonar1_distance < 20 || sonars.sonar2_distance < 20) {
                    app.movement = 0;
                    app.update_movement();
                }
            }

            Serial.print(comp_fil.theta);
            Serial.print("\t");
            Serial.print(app.backwards);
            Serial.print("\t");
            Serial.print(app.stop);
            Serial.print("\t");
            Serial.print(app.stop_count);
            Serial.print("\t");
            Serial.print(app.movement);
            Serial.print("\t");
            Serial.print(app.autopilot);
            Serial.print("\t");
            Serial.println(pid.setpoint);

            // String sendString = String(pid.setpoint);
            // sendString.concat(" ");
            // sendString.concat(app.stop_count);
            // sendString.concat(" ");
            // sendString.concat(app.stop);
            // customCharacteristic.writeValue(sendString.c_str());
      }

        digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
        // Serial.println("Disconnected from central.");
    }


    // Serial.print(pwm_values[1]);
    // Serial.print("\t");
    // Serial.print(pwm_values[0]);
    // Serial.print("\t");
    // Serial.print(pwm_values[3]);
    // Serial.print("\t");
    // Serial.print(pwm_values[2]);

    // Serial.print("\t");
    // Serial.print(pid.Kp);
    // Serial.print("\t");
    // Serial.print(pid.Ki);
    // Serial.print("\t");
    // Serial.print(pid.Kd);
    // Serial.print("\t");
    // Serial.print(pid.calibration_offset);
    
    // Serial.print("\t");
    // Serial.print(pid.proportional);
    // Serial.print("\t");
    // Serial.print(pid.integral);
    // Serial.print("\t");
    // Serial.print(pid.derivative);

    // Serial.print("\t");
    // Serial.print(sonars.sonar1_distance);
    // Serial.print("\t");
    // Serial.print(sonars.sonar2_distance);
    
    // Serial.print("\t");
    // Serial.print(comp_fil.dt,5);
    // Serial.print("\t");
    // Serial.print(pid.dt,5);
    // Serial.print("\t");
    // Serial.println(comp_fil.theta);
    // Serial.print("\t");
    // Serial.print(actual_angle);
    // Serial.print("\t");
    // Serial.println(pid.error);
    // Serial.print("\t");
    // Serial.println(pid.control_sig);
    // Serial.println(flag);
}