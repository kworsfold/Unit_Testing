#ifndef SerialMenu_h
#define SerialMenu_h

#include <config.h>
#include "PresenceSensor.h"
#include "ServoControl.h"
#include "LightingEffects.h"

class SerialMenu {
public:
    SerialMenu(PresenceSensor& presenceSensor, ServoControl& servoControl, LightingEffects& lightingEffects);
    void update();
    int _selectedServo; // Add this line
    void testServos();  //  TODO: public or private?

private:
    LightingEffects _lightingEffects; // Add the member variable
    PresenceSensor& _presenceSensor;
    ServoControl& _servoControl;
    bool _servoConfigMode;
    void printHelp();
    void printServoConfigHelp();
    void checkPresenceSensor();
    void openDoors();
    void closeDoors();
    void animatePuppet();
    void stopPuppet();
    void lightningEffect();
    void redLights();
    void clearLights();
    void servoConfig();
    void enableActiveMode();
    void disableActiveMode();
    void incrementServo(int servoNumber);
    void decrementServo(int servoNumber);
    void processCommand(char command);
    void processServoConfigCommand(char command);
    void verifyHardware();
    void i2cScan(); // Added I2C scan function
    void testLEDs(); // Added LED test function
    void displayEEPROMSettings();
    void displayCurrentServoSettings(int servo);

//  TODO: add more functions for other commands
// Add new functions
    void setDoor1Min();
    void setDoor1Max();
    void setDoor2Min();
    void setDoor2Max();
    void setPuppetMin();
    void setPuppetMax();
    void setServoSpeed();

    void setServoMinFromSerial(int servo);
    void setServoMaxFromSerial(int servo);
    void setServoSpeedFromSerial(int servo) ;


    void centerServos();
    void printServoVariables();

// Add functions for the other min/max settings


};

#endif