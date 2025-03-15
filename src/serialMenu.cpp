#include <Arduino.h>
//#include <Wire.h> // Include Wire library for I2C
#include "config.h"
#include "SerialMenu.h"
#include "PresenceSensor.h"
#include "ServoControl.h"
#include "LightingEffects.h"
#if ENABLE_EEPROM
    #include <EEPROM.h>
#endif

SerialMenu::SerialMenu(PresenceSensor& presenceSensor, ServoControl& servoControl, LightingEffects& lightingEffects)
    : _presenceSensor(presenceSensor), _servoControl(servoControl), _lightingEffects(lightingEffects), _selectedServo(0), _servoConfigMode(false) {}

//SerialMenu::SerialMenu(PresenceSensor& presenceSensor, ServoControl& servoControl, LightingEffects& lightingEffects)
//    : _presenceSensor(presenceSensor), _servoControl(servoControl), _lightingEffects(lightingEffects), _servoConfigMode(false) {}

void SerialMenu::setServoMinFromSerial(int servo) {
    Serial.print("Enter new Servo "); Serial.print(servo + 1); Serial.println(" Min value:");
    while (!Serial.available()) {
        delay(10);
    }
    int newValue = Serial.parseInt();
    _servoControl.setServoMin(servo, newValue);
    Serial.print("Servo "); Serial.print(servo + 1); Serial.print(" Min set to: "); Serial.println(newValue);
}

void SerialMenu::setServoMaxFromSerial(int servo) {
    Serial.print("Enter new Servo "); Serial.print(servo + 1); Serial.println(" Max value:");
    while (!Serial.available()) {
        delay(10);
    }
    int newValue = Serial.parseInt();
    _servoControl.setServoMax(servo, newValue);
    Serial.print("Servo "); Serial.print(servo + 1); Serial.print(" Max set to: "); Serial.println(newValue);
}


void SerialMenu::setServoSpeedFromSerial(int servo) {
    Serial.print("Enter new Servo "); Serial.print(servo + 1); Serial.println(" Speed value:");
    while (!Serial.available()) {
        delay(10);
    }
    int newValue = Serial.parseInt();
    _servoControl.setServoSpeed(servo, newValue);
    Serial.print("Servo "); Serial.print(servo + 1); Serial.print(" Speed set to: "); Serial.println(newValue);
}





void SerialMenu::update() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        if (_servoConfigMode) {
            processServoConfigCommand(command);
        } else {
            processCommand(command);
        }
    }
}



void SerialMenu::processCommand(char command) {
    if (_servoConfigMode) {
        processServoConfigCommand(command);
    } else {
        switch (command) {
            case 'S': _servoConfigMode = true; printServoConfigHelp(); break;
            case 'h': printHelp(); break;
            case 'p': checkPresenceSensor(); break;

            case 'o': openDoors(); break;
            case 'c': closeDoors(); break;
            case 'a': animatePuppet(); break;
            case 's': stopPuppet(); break;

            case 'l': lightningEffect(); break;
            case 'r': redLights(); break;
            case 'n': clearLights(); break;
            
            case 'm': servoConfig(); break;
            case 'T': testServos(); break;
            
            case 'e': enableActiveMode(); break;
            case 'd': disableActiveMode(); break;
            case 'v': verifyHardware(); break;
            case 'i': i2cScan(); break; // Added I2C scan command
            case 't': testLEDs(); break; // Added LED test command
            
            case 'E': displayEEPROMSettings(); break; // Added EEPROM display command
            case 'C': centerServos(); break; // Center servos command
            case 'V': printServoVariables(); break; // Print servo variables command

            default: Serial.println("Invalid command."); break;
        }
    }
}


void SerialMenu::printServoConfigHelp() {
    Serial.println("Servo Config Mode Commands:");
    Serial.println("1: Select Door 1");
    Serial.println("2: Select Door 2");
    Serial.println("3: Select Puppet");
    Serial.println("'+': Increment selected servo by 1");
    Serial.println("'-': Decrement selected servo by 1");
    Serial.println("a: set Servo Min");
    Serial.println("b: set Servo Max");
    Serial.println("s: set Servo Speed");
    Serial.println("x: exit servo config mode");
    Serial.println("h: help");
}

void SerialMenu::processServoConfigCommand(char command) {
    switch (command) {
        case '1': _selectedServo = 0; displayCurrentServoSettings(_selectedServo); break;
        case '2': _selectedServo = 1; displayCurrentServoSettings(_selectedServo); break;
        case '3': _selectedServo = 2; displayCurrentServoSettings(_selectedServo); break;
        case '+': incrementServo(_selectedServo); break;
        case '-': decrementServo(_selectedServo); break;
        case 'a': setServoMinFromSerial(_selectedServo); break;
        case 'b': setServoMaxFromSerial(_selectedServo); break;
        case 's': setServoSpeedFromSerial(_selectedServo); break;
        case 'x': _servoConfigMode = false; Serial.println("Exiting servo config mode."); break;
        case 'h': printServoConfigHelp(); break;
        default: Serial.println("Invalid servo config command."); break;
    }
}

void SerialMenu::printHelp() {
    Serial.println("Available commands:");
    Serial.println("h: Help");
    Serial.println("p: Check Presence Sensor");
    Serial.println("o: Open Doors");
    Serial.println("c: Close Doors");
    Serial.println("a: Animate Puppet");
    Serial.println("s: Stop Puppet");
    Serial.println("l: Lightning Effect");
    Serial.println("r: Red Lights");
    Serial.println("n: Clear Lights");
    Serial.println("m: Servo Configuration");
    Serial.println("e: Enable Active Mode");
    Serial.println("d: Disable Active Mode");
    Serial.println("v: Verify Hardware");
    Serial.println("i: I2C Scan"); // Added I2C scan command
    Serial.println("t: Test LEDs"); // Added LED test command
    Serial.println("E: Display EEPROM Settings");
    Serial.println("C: Center Servos");
    Serial.println("V: Print Servo Variables");
}


void SerialMenu::displayCurrentServoSettings(int servo) {
    Serial.print("Servo "); Serial.print(servo + 1); Serial.println(" Settings:");
    Serial.print("  Min: "); Serial.println(_servoControl.getServoMin(servo));
    Serial.print("  Max: "); Serial.println(_servoControl.getServoMax(servo));
    Serial.print("  Current Pulse: "); Serial.println(_servoControl.getServoCurrentPulse(servo));
    Serial.print("  Speed: "); Serial.println(_servoControl.getServoSpeed());
}

void SerialMenu::centerServos() {
    _servoControl.centerServos();
}

void SerialMenu::printServoVariables() {
    _servoControl.printServoVariables();
}

void SerialMenu::checkPresenceSensor() {
    Serial.println(_presenceSensor.isDetected() ? "Detected" : "Not Detected");
}

void SerialMenu::openDoors() {
    _servoControl.openDoors();
    Serial.println("Opening doors.");
}

void SerialMenu::closeDoors() {
    _servoControl.closeDoors();
    Serial.println("Closing doors.");
}

void SerialMenu::animatePuppet() {
    _servoControl.animatePuppet();
    Serial.println("Animating puppet.");
}

void SerialMenu::stopPuppet() {
    _servoControl.stopPuppet();
    Serial.println("Stopping puppet.");
}

void SerialMenu::lightningEffect() {
    //_lightingEffects.lightningEffect();
    Serial.println("Lightning effect triggered.");
}

void SerialMenu::redLights() {
    //_lightingEffects.redLights();
    Serial.println("Red lights on.");
}

void SerialMenu::clearLights() {
    //_lightingEffects.clearLights();
    Serial.println("Lights cleared.");
}

void SerialMenu::servoConfig() {
    _servoConfigMode = true;
    Serial.println("Entering servo config mode. Use +, -, 1, 2, 3, 4, a, b, c, d, e, f, s, h to help, x to exit.");
}

void SerialMenu::enableActiveMode() {
    // Implement active mode logic
    Serial.println("Active mode enabled.");
}

void SerialMenu::disableActiveMode() {
    // Implement disable active mode logic
    Serial.println("Active mode disabled.");
}




void SerialMenu::incrementServo(int servo) {
    _servoControl.adjustServoPulse(servo, 1);
    displayCurrentServoSettings(servo);
}



void SerialMenu::decrementServo(int servo) {
    _servoControl.adjustServoPulse(servo, -1);
    displayCurrentServoSettings(servo);
}





void SerialMenu::i2cScan() {
    Serial.println("I2C Scanner:");

    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknow error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.print(nDevices);
        Serial.println(" device(s) found\n");
    }
}

void SerialMenu::verifyHardware() {
    Serial.println("Hardware Check:");

    // LD2410 Check
    Serial.print("LD2410 Presence: ");
    Serial.println(_presenceSensor.isDetected() ? "Detected" : "Not Detected");

    // PCA9685 Check (Basic: Check if I2C communication is working)
    Serial.print("PCA9685: ");
    // Here add code to test I2C communication with the PCA9685.
    // Example: try to read a register or send a command.
    // However, the current library does not have an easy way to check if the device is connected.
    Serial.println("Communication test not implemented");

    // NeoPixel Check (Basic: Turn on a pixel)
    Serial.print("NeoPixel: ");
    _lightingEffects.setPixelColor(0, 0x00FF00); // Green first pixel
    _lightingEffects.show();
    delay(100);
    _lightingEffects.clear();
    _lightingEffects.show();
    Serial.println("Basic pixel test complete.");
    Serial.println("Hardware check complete.");
}

void SerialMenu::testServos() {
    Serial.println("Servo Test Mode. Enter command: [d1/d2/p] [position]");
    Serial.println("Example: d1 90");
    while (true) {
        if (Serial.available() > 0) {
            char servo = Serial.read();
            if (servo == '\r'){
                continue;
            }
            int position = Serial.parseInt();
            if (Serial.peek() == '\n'){
                Serial.read();
            }

            switch (servo) {
                case 'd':
                    if (Serial.read() == '1') {
                        _servoControl.setServoPulse(_servoControl.getDoor1Channel(), position);
                        Serial.print("Door 1 moved to: ");
                        Serial.println(_servoControl.getDoor1Position());
                    } else if (Serial.read() == '2') {
                        _servoControl.setServoPulse(_servoControl.getDoor2Channel(), position);
                        Serial.print("Door 2 moved to: ");
                        Serial.println(_servoControl.getDoor2Position());
                    }
                    break;
                case 'p':
                    _servoControl.setServoPulse(_servoControl.getPuppetChannel(), position);
                    Serial.print("Puppet moved to: ");
                    Serial.println(_servoControl.getPuppetPosition());
                    break;
                case 'x':
                    Serial.println("Exiting servo test mode.");
                    return;
                default:
                    Serial.println("Invalid servo command.");
                    break;
            }
        }
    }
}

void SerialMenu::testLEDs() {
    Serial.println("Testing LEDs...");

    int ledPins[] = {
        _lightingEffects.getLED1Pin(),
        _lightingEffects.getLED2Pin(),
        _lightingEffects.getLED3Pin(),
        _lightingEffects.getLED4Pin(),
        _lightingEffects.getLED5Pin()
    };
    
    for (int i = 0; i < 5; i++) {
        int pin = ledPins[i];
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        Serial.print("LED "); Serial.print(i + 1); Serial.println(" ON");
        delay(500);
        digitalWrite(pin, LOW);
        Serial.print("LED "); Serial.print(i + 1); Serial.println(" OFF");
        delay(250);
    }

    Serial.println("LED test complete.");
}

//  TODO: Add more functions 
// Add new functions


void SerialMenu::setDoor1Min() {
    Serial.println("Enter Door 1 Min Pulse Width:");
    while (Serial.available() == 0);
    int min = Serial.parseInt();
    _servoControl.setDoor1Min(min);
    Serial.print("Door 1 Min set to: ");
    Serial.println(min);
}

void SerialMenu::setDoor1Max() {
    Serial.println("Enter Door 1 Max Pulse Width:");
    while (Serial.available() == 0);
    int max = Serial.parseInt();
    _servoControl.setDoor1Max(max);
    Serial.print("Door 1 Max set to: ");
    Serial.println(max);
}

void SerialMenu::setDoor2Min() {
    Serial.println("Enter Door 2 Min Pulse Width:");
    while (Serial.available() == 0);
    int min = Serial.parseInt();
    _servoControl.setDoor2Min(min);
    Serial.print("Door 2 Min set to: ");
    Serial.println(min);
}

void SerialMenu::setDoor2Max() {
    Serial.println("Enter Door 2 Max Pulse Width:");
    while (Serial.available() == 0);
    int max = Serial.parseInt();
    _servoControl.setDoor2Max(max);
    Serial.print("Door 2 Max set to: ");
    Serial.println(max);
}

void SerialMenu::setPuppetMin() {
    Serial.println("Enter Puppet Min Pulse Width:");
    while (Serial.available() == 0);
    int min = Serial.parseInt();
    _servoControl.setPuppetMin(min);
    Serial.print("Puppet Min set to: ");
    Serial.println(min);
}

void SerialMenu::setPuppetMax() {
    Serial.println("Enter Puppet Max Pulse Width:");
    while (Serial.available() == 0);
    int max = Serial.parseInt();
    _servoControl.setPuppetMax(max);
    Serial.print("Puppet Max set to: ");
    Serial.println(max);
}

// Add functions for the other min/max settings

void SerialMenu::setServoSpeed() {
    Serial.println("Enter Servo Speed (1-10):");
    while (Serial.available() == 0);
    int speed = Serial.parseInt();
    _servoControl.setServoSpeed(speed);
    Serial.print("Servo Speed set to: ");
    Serial.println(speed);
}

void SerialMenu::displayEEPROMSettings() {
    Serial.println("EEPROM Settings:");


#if ENABLE_EEPROM
    int door1Min = EEPROM.read(EEPROM_DOOR1_MIN_ADDR) | (EEPROM.read(EEPROM_DOOR1_MIN_ADDR + 1) << 8) | (EEPROM.read(EEPROM_DOOR1_MIN_ADDR + 2) << 16) | (EEPROM.read(EEPROM_DOOR1_MIN_ADDR + 3) << 24);
    int door1Max = EEPROM.read(EEPROM_DOOR1_MAX_ADDR) | (EEPROM.read(EEPROM_DOOR1_MAX_ADDR + 1) << 8) | (EEPROM.read(EEPROM_DOOR1_MAX_ADDR + 2) << 16) | (EEPROM.read(EEPROM_DOOR1_MAX_ADDR + 3) << 24);
    int door2Min = EEPROM.read(EEPROM_DOOR2_MIN_ADDR) | (EEPROM.read(EEPROM_DOOR2_MIN_ADDR + 1) << 8) | (EEPROM.read(EEPROM_DOOR2_MIN_ADDR + 2) << 16) | (EEPROM.read(EEPROM_DOOR2_MIN_ADDR + 3) << 24);
    int door2Max = EEPROM.read(EEPROM_DOOR2_MAX_ADDR) | (EEPROM.read(EEPROM_DOOR2_MAX_ADDR + 1) << 8) | (EEPROM.read(EEPROM_DOOR2_MAX_ADDR + 2) << 16) | (EEPROM.read(EEPROM_DOOR2_MAX_ADDR + 3) << 24);
    int puppetMin = EEPROM.read(EEPROM_PUPPET_MIN_ADDR) | (EEPROM.read(EEPROM_PUPPET_MIN_ADDR + 1) << 8) | (EEPROM.read(EEPROM_PUPPET_MIN_ADDR + 2) << 16) | (EEPROM.read(EEPROM_PUPPET_MIN_ADDR + 3) << 24);
    int puppetMax = EEPROM.read(EEPROM_PUPPET_MAX_ADDR) | (EEPROM.read(EEPROM_PUPPET_MAX_ADDR + 1) << 8) | (EEPROM.read(EEPROM_PUPPET_MAX_ADDR + 2) << 16) | (EEPROM.read(EEPROM_PUPPET_MAX_ADDR + 3) << 24);
    int servoSpeed = EEPROM.read(EEPROM_SERVO_SPEED_ADDR);

    Serial.print("Door 1 Min: "); Serial.println(door1Min);
    Serial.print("Door 1 Max: "); Serial.println(door1Max);
    Serial.print("Door 2 Min: "); Serial.println(door2Min);
    Serial.print("Door 2 Max: "); Serial.println(door2Max);
    Serial.print("Puppet Min: "); Serial.println(puppetMin);
    Serial.print("Puppet Max: "); Serial.println(puppetMax);
    Serial.print("Servo Speed: "); Serial.println(servoSpeed);

#else
    Serial.println("EEPROM functionality disabled.");
#endif

}
/**/