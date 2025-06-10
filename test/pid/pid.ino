#include "robot_functions.h"
#include "DFRobot_DF2301Q.h"
#include <DFRobotDFPlayerMini.h>

// Audio Trigger Pins 
#define ADKEY_1 A0  // Track 1
#define ADKEY_2 A1  // Track 2
#define ADKEY_3 A2  // Track 3
#define ADKEY_4 A3  // Track 4
#define ADKEY_5 A6  // Track 5 
#define BUSY_PIN A7 // DFPlayer Mini BUSY pin

DFRobot_DF2301Q_I2C asr;
DFRobotDFPlayerMini myDFPlayer;

const int distTHRESH = 35; // in cm
bool autoMoveFlag = false;
unsigned long prevSonTime = 0;
short trackApp = 0;
short prevTrackApp = 0;
unsigned long timecheck = 0;

void setup() {
    // setup_Serial(); // Initialize serial communincation
    setup_IMU(); // Initialize IMU
    // setup_Timer4(); // Initialize timer4
    setup_PWM0();   // Initialize PWM0
    pinMode(LED_BUILTIN, OUTPUT); // Initialize the built-in LED to indicate connection status
    setup_BLE(); // Initialize BLE connection
    sonars.setup(); // Initialize tigger pins of sonars to output
    setupVoice();
    
    float theta = 0;
    for (int i = 0; i < 10; i ++) {
        while (!IMU.gyroscopeAvailable() && !IMU.accelerationAvailable());
        comp_fil.update();
        theta += comp_fil.theta_a;
    }
    comp_fil.theta = theta / 10.0;

    pinMode(ADKEY_1, INPUT);
    pinMode(ADKEY_2, INPUT);
    pinMode(ADKEY_3, INPUT);
    pinMode(ADKEY_4, INPUT);
    pinMode(ADKEY_5, INPUT);
    pinMode(BUSY_PIN, INPUT);
}

void loop() {
    checkVoiceModule();
    comp_fil.update();  // update IMU reading/angle
    pid.update(); // update PID outout
    update_pwm(); // updates motor pwm values
    //keyboard_test(); // gets new K gain values from keyboard
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

                app.movement = atoi(receivedString);
                app.update_movement();
                if(app.movement == 8){
                  autoMoveFlag = 0;
                } else if(app.movement == 9){
                  autoMoveFlag = 1;
                }
                // Print the received data to the Serial Monitor
                Serial.print("Received data: ");
                Serial.println(app.movement);

                // Release the ADKEY pin after TRIGGER_TIME
            }
                trackApp = app.movement;
                  if(app.movement <= 9 && trackApp != prevTrackApp && digitalRead(BUSY_PIN) != LOW){
                  switch(app.movement){
                    case 5: {
                      pinMode(ADKEY_1, OUTPUT);
                      timecheck +=1;
                      break;
                    }
                    case 6: {
                      pinMode(ADKEY_2, OUTPUT);
                      timecheck +=1;      
                      break;      
                    }
                    case 7: {
                      pinMode(ADKEY_3, OUTPUT);
                      timecheck+=1;
                      break;                    
                    }
                    case 8: {
                      pinMode(ADKEY_4, OUTPUT);
                      timecheck+=1;
                      break;                    
                    }
                    case 9: {
                      pinMode(ADKEY_5, OUTPUT);
                      timecheck+=1;
                      break;
                    }
                    default: {
                      timecheck+=1;
                      break;
                    }
                  }
                  if(timecheck > 4){
                  pinMode(ADKEY_1, INPUT);
                  pinMode(ADKEY_2, INPUT);
                  pinMode(ADKEY_3, INPUT);       
                  pinMode(ADKEY_4, INPUT);  
                  pinMode(ADKEY_5, INPUT);    
                  timecheck = 0;           
                  prevTrackApp = trackApp;
                          
                  }

                }
                customCharacteristic.writeValue("Data received"); // Optionally, respond by updating the characteristic's value

            
            checkVoiceModule();
            comp_fil.update();
            pid.update();
            update_pwm();   // updates motor pwm values
            app.update_stop();

            sonars.get_distance(); // gets distance from sonars

            // start of logic code...
            if(autoMoveFlag){
            if (sonars.sonar1_distance < distTHRESH || sonars.sonar2_distance < distTHRESH) {
              Serial.println("distTHRESH!");
              if (millis() - prevSonTime >= 1800) { // the compare value must be greater than the sum of the autoMove durations.
                prevSonTime = millis();
                if (app.movement == 1 || app.movement == 2 || app.movement == 3) {
                  if (random(1, 3) == 1) {
                    autoMove(0, 500);
                    autoMove(4, 300);
                    autoMove(3, 750);
                    autoMove(1, 100);
                    Serial.println("DONE 1!");
                  } else {
                    autoMove(0, 500);
                    autoMove(4, 300);
                    autoMove(2, 750);
                    autoMove(1, 100);
                    Serial.println("DONE 2!");
                  }
                }
              } 
            } 
            }
            // Serial.print(comp_fil.theta);
            // Serial.print("\t");
            // Serial.print(app.stop_count);
            // Serial.print("\t");
            // Serial.print(app.movement);
            // Serial.print("\t");
            // Serial.println(pid.setpoint);

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
    Serial.print("\t");
    Serial.println(app.movement);

    // Serial.print("\t");
    // Serial.print(sonars.sonar1_distance);
    // Serial.print("\t");
    // Serial.println(sonars.sonar2_distance);
    
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

void autoMove(int direction, int duration) {
  // forward: 1, left: 2, right: 3, backward: 4
  // duration is in ms
  app.movement = direction;
  app.update_movement();
  app.update_stop();

  unsigned long start = millis();
  while (millis() - start < duration) {
    comp_fil.update();
    pid.update();
    update_pwm();
    Serial.println("automoving");
  }
}

void setupVoice() {
  while (!(asr.begin())) {
    Serial.println("Communcation with voice module failed.");
    delay(3000);
  }
  asr.setVolume(10);
  asr.setMuteMode(0); // 1 = mute, 0 = unmuted
  asr.setWakeTime(20);

  uint8_t wakeTime = 0;
  wakeTime = asr.getWakeTime();
  Serial.print("wakeTime = ");
  Serial.println(wakeTime);

  // asr.playByCMDID(1);   // Wake-up command

}

void checkVoiceModule(){
  uint8_t CMDID = asr.getCMDID();
  switch (CMDID) {
    case 22: {    // "Go Forward"
      Serial.println("going forwards...");
      app.movement = 1;
      app.update_movement();
      break;
    }

    case 5: {     // "Turn Left"
      Serial.println("turning left...");
      app.movement = 2;
      app.update_movement();
      break;
    }
    
    case 6: {     // "Turn Right"
      Serial.println("turning right...");
      app.movement = 3;
      app.update_movement();
      break;
    }

    case 7: {     // "Go backward"
      Serial.println("going backwards...");
      app.movement = 4;
      app.update_movement();
      break;
    }

    case 8: {     // "Stop moving"
      Serial.println("stopping...");
      app.movement = 0;
      app.update_movement();
      break;
    }
    
    default:
      if (CMDID != 0) {
        Serial.print("CMDID = "); Serial.println(CMDID);
      }
  }
}
