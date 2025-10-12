#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include "controller_callbacks.h"


#define IN1  16  // Control pin 1
#define IN2  17  // Control pin 2

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

            delay(1000);  // Run for 1 second

            
            return;
        }
        else {
            Console.printf("Press button A!"); // Replace with whatever you want
            // Stop motor
            analogWrite(IN1, 0);
            digitalWrite(IN2, LOW);

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
}

void loop() {

    vTaskDelay(1); // Ensures WDT does not get triggered when no controller is connected
    BP32.update(); 

    for (auto myController : myControllers) { // Only execute code when controller is connected
        if (myController && myController->isConnected() && myController->hasData()) {        
            
            foo(myController);
            dumpGamepad(myController); 

        }
    }
}