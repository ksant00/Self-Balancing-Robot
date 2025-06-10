#include "robot_functions.h"

/////////////////////////////// Global Variables //////////////////////////////////

// PWM values (initialized to 0%)
uint16_t pwm_values[] = {1000 | (1 << 15), 1000 | (1 << 15), 1000 | (1 << 15), 1000 | (1 << 15)}; 

pid_controller pid; // Creates instance of class pid_controller
complementary_filter comp_fil;   // Creates instance of class complementary_filter
app_controls app; // Creates instance of class app_control
int task = 0;   // for serial testing
String input;   // for serial testing
sonar sonars; // Creates instance of class sonar
// DFRobot_DF2301Q_I2C asr; // voice module

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EA4-4183-85CD-B1EB8D5CFBAD");
BLECharacteristic customCharacteristic("00000001-5EA4-4183-85CD-B1EB8D5CFBAD", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////// ISR(s) ///////////////////////////////////////////


extern "C" {
    /** 
     * Interrupt Service Routine (ISR) for Timer 4
     */ 
    void TIMER4_IRQHandler_v (void) {
        if (NRF_TIMER4->EVENTS_COMPARE[2]) { // Checks if event occured
            NRF_TIMER4->EVENTS_COMPARE[2] = 0;  // Clear the event
        }
    }
}


/**
 * Interrupt Handler for sonar 1 echo pin
 * 
 * Interrupt event happens every digital state change of echo pin
 * Takes the start and end time for echo signal
 */
void sonar1_ISR (void) {
    if (digitalRead(sonar1_echo_pin) == HIGH) {
        sonars.sonar1_start_time = micros();
    } else {
        sonars.sonar1_end_time = micros();
    }
}

/**
 * Interrupt Handler for sonar 2 echo pin
 * 
 * Interrupt event happens every digital state change of echo pin
 * Takes the start and end time for echo signal
 */
void sonar2_ISR (void) {
    if (digitalRead(sonar2_echo_pin) == HIGH) {
        sonars.sonar2_start_time = micros();
    } else {
        sonars.sonar2_end_time = micros();
    }
}


///////////////////////////////////////////////////////////////////////////////////

////////////////////////////////// Functions //////////////////////////////////////

/**
 * Timer4 setup function
 * 
 * Sets to Timer mode (Counter) at 1 MHz frequency
 * Compare event(s) at: 1 second
 */
void setup_Timer4 (void) {
    // Enables TIMER4
    NRF_TIMER4->TASKS_STOP = 1;  // Stop timer (just in case)
    NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;  // Set to Timer mode
    NRF_TIMER4->PRESCALER = 4;  // 16 MHz / 2^4 = 1 MHz timer frequency
    NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit;  // 32-bit mode
    NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos; // Shortcut to clear compare register
    
    // Sets Compare Register(s) (Up to 6 compare evenets)
    // Compare event for 1 second
    NRF_TIMER4->CC[0] = 100;
    
    // Enable Interrupt on Compare Event(s)
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE1_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE2_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE3_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE4_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE5_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NVIC_EnableIRQ(TIMER4_IRQn);  // Enable interrupt in NVIC
    
    // Start the Timer
    NRF_TIMER4->TASKS_CLEAR = 1;  // Reset counter
    NRF_TIMER4->TASKS_START = 1;  // Start timer
}


/**
 * PWM0 setup function
 * 
 * Sets to Timer mode (Counter) at 1 MHz frequncy
 * Countertop set at 1000 clock cycles for 1kHz
 */
void setup_PWM0 (void) {
    // Configure PWM output pins
    NRF_PWM0->PSEL.OUT[0] = (1 << 5) | PWM_PIN_D2;  // Select P1.11 (D2)
    NRF_PWM0->PSEL.OUT[1] = (1 << 5) | PWM_PIN_D3;  // Select P1.12 (D3)
    NRF_PWM0->PSEL.OUT[2] = (1 << 5) | PWM_PIN_D4;  // Select P1.15 (D4)
    NRF_PWM0->PSEL.OUT[3] = (1 << 5) | PWM_PIN_D5;  // Select P1.13 (D5)
    
    // Enable the PWM hardware module
    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    
    // Set PWM mode to UP mode (counts up and resets)
    NRF_PWM0->MODE = PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos;
    
    // Set the prescaler (PWM frequency base) - 16MHz / 16 = 1MHz timer
    NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16;
    
    // Set the maximum counter top value (for a frequency of 1kHz)
    NRF_PWM0->COUNTERTOP = 1000;  // 1 MHz / 1000 = 1 kHz PWM Frequency
    
    // Disable the loop 
    NRF_PWM0->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
    
    // Configure PWM decoder (individual duty cycles)
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);

    // Configure the sequence (4 independent PWM duty cycles)
    NRF_PWM0->SEQ[0].PTR = (uint32_t)pwm_values;  // Point to duty cycle array
    NRF_PWM0->SEQ[0].CNT = 4;  // 4 PWM outputs
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
    
    // Start the sequence
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

/**
 * Initializes serial
 * 
 * Configures to 9600 baud rate and timeout of 10 ms
 */
void setup_Serial (void) {
    // initialize serial
    Serial.begin(9600);
    while(!Serial);
    Serial.setTimeout(10);
}

/**
 * Initializes IMU
 */
void setup_IMU (void) {
    // case if serial does not initialize
    if(!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while(1);
    }
}


/**
 * Initiales BLE connection
 * 
 * Device named to BLE-DEVICEA4
 */
void setup_BLE (void) {
    if (!BLE.begin()) {
        Serial.println("Starting BLE failed!");
        while (1);
    }

    // Set the device name and local name
    BLE.setLocalName("A4");
    BLE.setDeviceName("A4");

    // Add the characteristic to the service
    customService.addCharacteristic(customCharacteristic);

    // Add the service
    BLE.addService(customService);

    // Set an initial value for the characteristic
    customCharacteristic.writeValue("Waiting for data");

    // Start advertising the service
    BLE.advertise();

    Serial.println("Bluetooth® device active, waiting for connections...");
}

// Constructor definition
pid_controller::pid_controller (void) {
    K_mast = 1.0f;   // master gain
    Kp = 33.33f;  // porportional gain
    Ki = 367.0f;   // integral gain
    Kd = 1.52;   // derivative gain

    setpoint = 0.0f; // desired tilt angle
    error = 0.0f;        // current error value
    prev_error = 0.0f;   // previous error value
    calibration_offset = 0.0f;

    // for derivative low-pass filt time constant if we use
    tau = 0.0f;

    proportional = 0.0f; // value of proportional error with gain
    integral = 0.0f; // accumulated error of integral
    derivative = 0.0f;   // value of derivative error w ith gain
    control_sig = 0.0f;  // value of control sigal | sum of all errors

    ct = 0; // variable to store current/previous time in microseconds
    dt = 0.0f; // variable for delta time in seconds
}

/**
 * Function to update PID controller
 * 
 * Takes the sum of all error multiplied with respective gain. Integral is constrained to (-10, and 10) to limit windup.
 */
void pid_controller::update (void) {
    float actual_angle = comp_fil.theta + calibration_offset;   // measured angle from the sensor with filter
    // if (actual_angle < .2 && actual_angle > -.2) actual_angle = 0;    // Filters out noise for verticle balance
    error = setpoint - actual_angle;    // sets new error

    dt = (float) (micros() - ct) / 1000000;  // gets time for ∆t
    ct = micros();  // sets new current time
    proportional = Kp * error; // computes the proportional error

    integral += Ki * dt * (error + prev_error) / 2.0; // computes the integral error
    if ((app.stop != 0) && app.movement == 0) {
        integral = constrain(integral, -800, 800); // contrains the error to prevent integral windup
    } else if (app.movement == 1 || app.movement == 4) {
        integral = constrain(integral, -1000, 1000); // contrains the error to prevent integral windup (higher constrain to allow the bot to go forward and backward)
    } else {
        integral = constrain(integral, -800, 800); // contrains the error to prevent integral windup
    }

    if (comp_fil.theta - comp_fil.theta_prev > 0.05 || comp_fil.theta - comp_fil.theta_prev < -0.05) 
        derivative = -Kd * (comp_fil.theta - comp_fil.theta_prev)/dt; // computes the derivative error
    else 
        derivative = 0; // filters out noise

    prev_error = error; // update the previous error

    control_sig = K_mast * ( proportional + integral + derivative );    // computes sum of error
    control_sig = constrain(control_sig, -1000, 1000);  // limits output of PID to limits of PWM
}


// Constructor definition
complementary_filter::complementary_filter (void) {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    theta_a = 0.0f;
    theta = 0.0f;
    theta_prev = 0.0f;
    ct = 0;
    dt = 0.0f;
}

/**
 * Function to use update IMU using complementary filter
 */
void complementary_filter::update (void) {
    dt = (float) (micros() - ct) / 1000000;  // gets time for ∆t
    ct = micros();  // sets new current time
    theta_prev = theta;

    if (IMU.gyroscopeAvailable()) {
        // reads gyroscope value
        IMU.readGyroscope(x, y, z);
        
        // computes theta using integration
        theta = (theta + x * dt);
    }

    if (IMU.accelerationAvailable()) {
        // reads acceleration
        IMU.readAcceleration(x, y, z);
        
        // computes theta values
        theta_a = -degrees(atan(y / z));
    }

    theta = k_comp_fil * theta + (1 - k_comp_fil) * theta_a;
}

/**
 * Function to update K gain values through serial
 * 
 * Enter kp/ki/kd
 * Enter gain value
 */
void keyboard_test (void) {
    if (Serial.available() > 0) {
        input = Serial.readString();
        input.trim();
        input.toLowerCase();
    }

    if (input == "kp") {
        task = 1;
    }
    else if (input == "ki") {
        task = 2;
    }
    else if (input == "kd") {
        task = 3;
    }
    else if (input == "reset") {
      pid.integral = 0;
    }
    else if (input == "c") {
      task = 4;
    }


    switch (task) {
        case 1:
            if(input == "0" || input.toFloat() > 0) pid.Kp = input.toFloat();
        break;
        case 2:
            if(input == "0" || input.toFloat() > 0) pid.Ki = input.toFloat();
        break;
        case 3:
            if(input == "0" || input.toFloat() > 0) pid.Kd = input.toFloat();
        break;
        case 4:
            if(input == "0" || input.toFloat() != 0) pid.calibration_offset = input.toFloat();
    }
}

/**
 * Function to update PWM for motors
 * 
 * Slow decay mode to increase torque
 * 100% dutcy cycle is stop 0% duty cycle is full speed
 */
void update_pwm (void) {
    if (pid.control_sig < 0) {
         if(pid.control_sig != 0) {
           if (app.movement == 2) {
             pwm_values[1] = (1 << 15) | map(abs(pid.control_sig * 1.4), 0, 1000, 965, 0);
             pwm_values[3] = (1 << 15) | map(abs(pid.control_sig * 0.6), 0, 1000, 965, 0);
            
             pwm_values[0] = (1 << 15) | 1000;
             pwm_values[2] = (1 << 15) | 1000;
           } 
           else if (app.movement == 3) {
             pwm_values[1] = (1 << 15) | map(abs(pid.control_sig * 0.6), 0, 1000, 965, 0);
             pwm_values[3] = (1 << 15) | map(abs(pid.control_sig * 1.4), 0, 1000, 965, 0);
 
             pwm_values[0] = (1 << 15) | 1000;
             pwm_values[2] = (1 << 15) | 1000;
           }
           else {
             pwm_values[1] = (1 << 15) | map(abs(pid.control_sig), 0, 1000, 965, 0);
             pwm_values[3] = (1 << 15) | map(abs(pid.control_sig), 0, 1000, 965, 0);
           }
         } 
         else {
           pwm_values[1] = (1 << 15) | 1000;
           pwm_values[3] = (1 << 15) | 1000;
         }
 
         if (app.movement != 3 || app.movement != 2) {
             pwm_values[0] = (1 << 15) | 1000;
             pwm_values[2] = (1 << 15) | 1000;
         }
     } 
   
    else {
        if(pid.control_sig != 0) {
            if (app.movement == 2) { // turns left with motors spining at opposite direction at moving angle
                pwm_values[0] = (1 << 15) | map(abs(pid.control_sig * 1.4), 0, 1000, 965, 0);
                pwm_values[2] = (1 << 15) | map(abs(pid.control_sig * 0.6), 0, 1000, 965, 0);

                
                pwm_values[1] = (1 << 15) | 1000;
                pwm_values[3] = (1 << 15) | 1000;
            } else if (app.movement == 3) { // turns right with motors spinning at scaled output at 0 degrees
                pwm_values[0] = (1 << 15) | map(abs(pid.control_sig * 0.6), 0, 1000, 965, 0);
                pwm_values[2] = (1 << 15) | map(abs(pid.control_sig * 1.4), 0, 1000, 965, 0);

                pwm_values[1] = (1 << 15) | 1000;
                pwm_values[3] = (1 << 15) | 1000;
            } else {
                pwm_values[0] = (1 << 15) | map(abs(pid.control_sig), 0, 1000, 962, 0);
                pwm_values[2] = (1 << 15) | map(abs(pid.control_sig), 0, 1000, 962, 0);
            }
        } else {
            pwm_values[0] = (1 << 15) | 1000;
            pwm_values[2] = (1 << 15) | 1000;
        }

        if (app.movement != 3 || app.movement != 2) {
            pwm_values[1] = (1 << 15) | 1000;
            pwm_values[3] = (1 << 15) | 1000;
        }
    }

    if (comp_fil.theta > 38 || comp_fil.theta < -38) {
      pwm_values[1] = (1 << 15) | 1000;
      pwm_values[3] = (1 << 15) | 1000;
      pwm_values[0] = (1 << 15) | 1000;
      pwm_values[2] = (1 << 15) | 1000;
    }

    // Wait for the PWM sequence to finish before updating
    while (NRF_PWM0->EVENTS_SEQEND[0] == 0);
    NRF_PWM0->EVENTS_SEQEND[0] = 0;  // Clear event flag

    // Restart the PWM sequence with updated values
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

sonar::sonar (void){
    sonar1_start_time = 0; 
    sonar2_start_time = 0; 
    sonar1_end_time = 0;
    sonar2_end_time = 0;
    sonar1_distance = 0; 
    sonar2_distance = 0; 
}

void sonar::setup (void) {
    pinMode(sonar1_echo_pin, INPUT);
    pinMode(sonar1_trig_pin, OUTPUT);
    pinMode(sonar2_echo_pin, INPUT);
    pinMode(sonar2_trig_pin, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(sonar1_echo_pin), sonar1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sonar2_echo_pin), sonar2_ISR, CHANGE);
}

void sonar::get_distance (void) {
    digitalWrite(sonar1_trig_pin, HIGH);
    digitalWrite(sonar2_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sonar1_trig_pin, LOW);
    digitalWrite(sonar2_trig_pin, LOW);
    delay(2);
    sonar1_distance = constrain((sonar1_end_time - sonar1_start_time)*0.0343/2.0, 0, 1000);
    sonar2_distance = constrain((sonar2_end_time - sonar2_start_time)*0.0343/2.0, 0, 1000);
}


app_controls::app_controls (void) {
    movement = 0;
    stop = 0;
    stop_count = 0;
}

void app_controls::update_movement (void) {
    switch (movement) {
        case 0:
            switch (stop) {
                case 0:
                    pid.setpoint = 0;
                break;
                case 1:
                    pid.setpoint = stop_count * 0.06;
                break;
                case 2:
                    pid.setpoint = stop_count * 0.05;
                break;
                case 3:
                    pid.setpoint = stop_count * 0.05;
                break;
                case 4:
                    pid.setpoint = stop_count * 0.06;
                break;
            }
        break;
        case 1: // forward
            pid.setpoint = -0.41;
            stop = 1;
        break;
        case 2: // left
            pid.setpoint = -0.2;

            stop = 2;
        break;
        case 3: // right 
            pid.setpoint = -0.2;

            stop = 3;
        break;
        case 4: // backward
            pid.setpoint = 0.41;
            stop = 4;
        break;
    }
}

void app_controls::update_stop (void) {
    if (movement != 0 && stop == 4) {
        stop_count--;
        stop_count = constrain(stop_count, -22, 22);
    } else if (movement != 0 && stop != 4) {
        stop_count++;
        stop_count = constrain(stop_count, -22, 22);
    } else if (movement == 0 && stop == 4) { // backward stop
        if (comp_fil.theta <= 0) {
            stop_count++;
            if (stop_count >= 0) {
                pid.setpoint = 0;
                stop = 0;
            }
        }
    } else if (movement == 0 && stop != 4) { // forward stop 
        if (comp_fil.theta >= 0) {
            stop_count--;
            if (stop_count <= 0) {
                pid.setpoint = 0;
                stop = 0;
            }
        }
    }
}

// void setup_voice (void) {
//     while (!(asr.begin())) {
//       Serial.println("Communcation with voice module failed.");
//       delay(3000);
//     }
//     asr.setVolume(9);
//     asr.setMuteMode(0); // 1 = mute, 0 = unmuted
//     asr.setWakeTime(20);
  
//     uint8_t wakeTime = 0;
//     wakeTime = asr.getWakeTime();
//     Serial.print("wakeTime = ");
//     Serial.println(wakeTime);
  
//     // asr.playByCMDID(1);   // Wake-up command
//   }
