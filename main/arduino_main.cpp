#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"
#include <QTRSensors.h>
#include <Wire.h>
#include <Arduino_APDS9960.h>
#include <bits/stdc++.h>
#include <ESP32SharpIR.h>

#define IN1  16  // Control pin 1
#define IN2  17  // Control pin 2
#define IN3  18
#define IN4  19
#define APDS9960_INT 2
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

TwoWire I2C_0 = TwoWire(0);
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT);

QTRSensors qtr;
uint16_t sensors[4];

ESP32SharpIR front(ESP32SharpIR::GP2Y0A21YK0F, 15);
ESP32SharpIR left(ESP32SharpIR::GP2Y0A21YK0F, 2);
ESP32SharpIR right(ESP32SharpIR::GP2Y0A21YK0F, 0);

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS]; // BP32 library allows for up to 4 concurrent controller connections, but we only need 1


void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "DPAD: %d A: %d B: %d X: %d Y: %d LX: %d LY: %d RX: %d RY: %d L1: %d R1: %d L2: %d R2: %d\n",
        ctl->dpad(),        // D-pad
        ctl->a(),           // Letter buttons
        ctl->b(),
        ctl->x(),
        ctl->y(),
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->l1(),           // Bumpers
        ctl->r1(),
        ctl->l2(),
        ctl->r2()
    );
}
void foo(ControllerPtr myController) {
    while(1) {
        BP32.update();
        if(myController->a()) {
            Console.printf("hi");

            // Spin motor
            analogWrite(IN1, 255);  // PWM signal
            digitalWrite(IN2, LOW); // Direction control

            analogWrite(IN3, 255);  // PWM signal
            digitalWrite(IN4, LOW); // Direction control

            delay(1000);  // Run for 1 second

            
            return;
        }
        else {
            Console.printf("Press button A!"); // Replace with whatever you want
            // Stop motor
            analogWrite(IN1, 0);
            digitalWrite(IN2, LOW);

            analogWrite(IN3, 0);
            digitalWrite(IN4, LOW);

            delay(1000); // Stop for 1 second

            vTaskDelay(1); // Yield CPU to not starve other ESP32 processes and cause WDT reset
        }
    }
}

void setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
    esp_log_level_set("gpio", ESP_LOG_ERROR); // Suppress info log spam from gpio_isr_service
    uni_bt_allowlist_set_enabled(true);

    //Serial.begin(115200);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    qtr.setTypeAnalog(); // or setTypeAnalog()
    qtr.setSensorPins((const uint8_t[]) {26, 25, 33, 35}, 4); // pin numbers go in the curly brackets {}, and number of sensors in use goes after

    // calibration sequence
    /*
    for (uint8_t i = 0; i < 250; i++) { 
        Console.printf("calibrating %d/250\n", i); // 250 is the number of calibrations recommended by manufacturer
        qtr.calibrate(); 
        delay(20);
    }
    */
    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    //sets up color sensor
    apds.setInterruptPin(APDS9960_INT);
    apds.begin();
    Serial.begin(115200);
    
    front.setFilterRate(1.0f);
    right.setFilterRate(1.0f);
    left.setFilterRate(1.0f);
    Console.printf("setup done\n");

}

void loop() {
    vTaskDelay(1);
    BP32.update(); 
    /*vTaskDelay(1); // Ensures WDT does not get triggered when no controller is connected
    BP32.update(); 

    qtr.readLineBlack(sensors); // Get calibrated sensor values returned into sensors[]
    Console.printf("S1: %d S2: %d S3: %d S4: %d\n", sensors[0], sensors[1], sensors[2], sensors[3]); //S1 is 1, S2 is 8
    delay(250);
    */
    //white is 1000, black is 0
    
    // Console.printf("RED: %d GREEN: %d BLUE: %d AMBIENT: %d\n", r, g, b, a);
    // delay(100);
    // BLUE: red is 120-130, green is about 220-230, blue is about 220 to 230
    // RED: red is about 360-370, green is about 40 to 50, blue is about 50 to 60
    // GREEN: red is about 270 to 290, green is about 340 to 360, blue is about 140 to 150
    //Serial.printf("reaching auto\n");
    for (auto myController : myControllers) { // Only execute code when controller is connected
        //Console.printf("inside for\n");

        if (myController && myController->isConnected() && myController->hasData()) {    
            Console.printf("inside if\n");    
            if(myController->a()){
                Console.printf("COLOR TEST");
                //color picker
                static int rf, gf, bf, af;
                int color=-1;
                while (!apds.colorAvailable()) { delay(5);} // Wait until color is read from the sensor 
                apds.readColor(rf, gf, bf, af);
                Console.printf("RED: %d GREEN: %d BLUE: %d AMBIENT: %d\n", rf, gf, bf, af);
                if (bf>rf&&bf>gf){ //for blue
                    color = 0;
                    Console.printf("collect blue");
                    delay(500);
                }else if(rf>bf&&rf>gf){ //for red
                    color = 1;
                    Console.printf("collect red");
                    delay(500);
                }else if(gf>rf&&gf>bf){ //for green
                    color = 2;
                    Console.printf("collect green");
                    delay(500);
                }
                
                    int r, g, b, a;
                    while (!apds.colorAvailable()) { delay(5);} // Wait until color is read from the sensor 
                    apds.readColor(r, g, b, a);
                    if(color==0){
                        
                        while(!(b>r&&b>g)){
                            //move forward
                            apds.readColor(r, g, b, a);
                            Console.printf("blue2");
                        }
                        Console.printf("blue3\n");
                    }
                    if(color == 1){
                        while(!(r>b&&r>g)){
                            //move forward
                            apds.readColor(r, g, b, a);
                            Console.printf("red");
                        }
                    }
                    if(color == 2){
                        while(!(g>r&&g>b)){
                            //move forward
                            apds.readColor(r, g, b, a);
                            Console.printf("green");
                        }
                    }
                    if(myController->y()){
                        break;
                    }
                
            }
            
            if(myController->b()){
                //use the distance sensor to go through the maze
                Console.printf("DISTANCE TEST");
                
                    if(front.getDistanceFloat()<30){
                        if(left.getDistanceFloat()>25){
                            //turn left
                            //turn off thr right wheel
                            Console.printf("turn left");
                        }else if(right.getDistanceFloat()>25){
                            //turn right
                            //turn off the left wheel
                            Console.printf("turn right");
                        }
                    }
                    //keep straight
                    Console.printf("keep straight");
                    if(myController->y()){
                        break;
                    }
                
                
            }
            if(myController->x()){
                //write the part for the launching
            }
            //foo(myController);
            dumpGamepad(myController); 

        }

    }

    // if(sensors[1]>800 && sensors[2]>800) {
        

    //     // Spin motor
    //     analogWrite(IN1, 255);  // PWM signal
    //     digitalWrite(IN2, LOW); // Direction control

    //     analogWrite(IN3, 255);  // PWM signal
    //     digitalWrite(IN4, LOW); // Direction control

    //     delay(1000);  // Run for 1 second

            
        
    // }
    // else {
    //     Console.printf("Press button A!"); // Replace with whatever you want
    //     // Stop motor
    //     analogWrite(IN1, 0);
    //     digitalWrite(IN2, LOW);

    //     analogWrite(IN3, 0);
    //     digitalWrite(IN4, LOW);

    //     delay(1000); // Stop for 1 second

    //     vTaskDelay(1); // Yield CPU to not starve other ESP32 processes and cause WDT reset
    // }

    //Console.println(front.getDistanceFloat()); 
    delay(500);

}



