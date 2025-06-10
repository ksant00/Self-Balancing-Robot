#include "ArduPID.h"
#include "Arduino_BMI270_BMM150.h"




ArduPID myController;




double input;
double output;

// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 512;
double p = 10;
double i = 1;
double d = 0.5;




void setup()
{
   // initialize serial
   Serial.begin(9600);
   while(!Serial);
   Serial.setTimeout(10);

   // case if serial does not initialize
   if(!IMU.begin()) {
       Serial.println("Failed to initialize IMU!");
       while(1);
   }

   pinMode(5, OUTPUT);
   pinMode(4, OUTPUT);
   pinMode(3, OUTPUT);
   pinMode(2, OUTPUT);

    myController.begin(&input, &output, &setpoint, p, i, d);

    // myController.reverse()               // Uncomment if controller output is "reversed"
    // myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
    myController.setOutputLimits(0, 255);
    myController.setBias(255.0 / 2.0);
    myController.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
    
    myController.start();
    // myController.reset();               // Used for resetting the I and D terms - only use this if you know what you're doing
    // myController.stop();                // Turn off the PID controller (compute() will not do anything until start() is called)
}




void loop()
{
    input = analogRead(A0); // Replace with sensor feedback

    myController.compute();
    // myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                                // PRINT_OUTPUT   | // in the Serial plotter
                                                // PRINT_SETPOINT |
                                                // PRINT_BIAS     |
                                                // PRINT_P        |
                                                // PRINT_I        |
                                                // PRINT_D);

    analogWrite(3, output); // Replace with plant control signal
    Serial.println(output);
}