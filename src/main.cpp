#include <Arduino.h>

#if ENABLE_EEPROM
    #include <EEPROM.h>
    #include "nvs_flash.h"
#endif

#include <config.h>
#include <presenceSensor.h>
#include <servoControl.h>
#include <serialMenu.h>
#include <lightingEffects.h>

// Define your hardware pins and settings here
//const uint8_t PCA9685_ADDRESS = 0x40; // Example: PCA9685 I2C address
//const uint8_t DOOR1_CHANNEL = 0;
//const uint8_t DOOR2_CHANNEL = 1;
//const uint8_t PUPPET_CHANNEL = 2;
const int NEOPIXEL_PIN = 19;
const int NEOPIXEL_COUNT = 30; // Example: Number of NeoPixels
// I2C Pin Definitions
#define I2C_SDA 21
#define I2C_SCL 22

#define EEPROM_DOOR1_MIN_ADDR 0
#define EEPROM_DOOR1_MAX_ADDR 4
#define EEPROM_DOOR2_MIN_ADDR 8
#define EEPROM_DOOR2_MAX_ADDR 12
#define EEPROM_PUPPET_MIN_ADDR 16
#define EEPROM_PUPPET_MAX_ADDR 20
#define EEPROM_SERVO_SPEED_ADDR 24 // Choose an unused EEPROM address

#define SERIAL_TIMEOUT 5000 // 5 seconds timeout

unsigned long lastTriggerTime = 0;
const unsigned long triggerDelay = 5000;
bool activeMode = true;

// Create instances of your classes
PresenceSensor presenceSensor;
ServoControl servoControl(PCA9685_ADDRESS, DOOR1_CHANNEL, DOOR2_CHANNEL, PUPPET_CHANNEL);
LightingEffects lightingEffects; // Corrected declaration
SerialMenu serialMenu(presenceSensor, servoControl, lightingEffects);

void setup() {
    Serial.begin(115200); // Initialize serial communication

    // Short while loop delay to allow the serial port to initialize as
    // otherwise the first print statement will not be seen.
    unsigned long startTime = millis();
    while (!Serial && millis() - startTime < SERIAL_TIMEOUT) {
        delay(100); // Wait for 100 milliseconds before checking again
    }

    if (!Serial) {
        Serial.println("Serial port not available after timeout.");
        // Optionally, add code here to handle the timeout (e.g., stop the program)
        // while(true); //Stops the program.
    } else {
        Serial.println("Serial port initialized.");
    }    // Initialize NVS
        
#if ENABLE_EEPROM
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
#endif

    Serial.println("Initialising i2c");
    Wire.begin(I2C_SDA, I2C_SCL); // Initialize Wire library with I2C pins
    //  TODO: Initialize PCA9685 a i2c is producing errors


#if ENABLE_EEPROM
    EEPROM.begin(32); // Initialize EEPROM with 32 bytes (adjust as needed)
#endif
    Serial.println("Halloween Bug Puppet Initializing...");
    lightingEffects.clear(); // clear any left over lights.

    // Call the functions to center servos and print variables
    servoControl.centerServos();
    servoControl.printServoVariables();
}

void loop() {
    presenceSensor.update();
    servoControl.update();
    serialMenu.update();
}

