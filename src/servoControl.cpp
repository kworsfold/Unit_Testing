#include "ServoControl.h"
#include <EEPROM.h>
#include <Arduino.h>
#include <config.h>
//#include <Wire.h>

// Assuming you have these constants defined somewhere

#define PUPPETMINPOS -80
#define PUPPETMAXPOS 80
//#define OPENEDPOS    -70     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
//#define CLOSEDPOS    70    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600

// Example pwmForAngle() function (adjust as needed for your servos)


struct ServoSettings servoSettings[NUM_SERVOS] = {  
    {DOOR1_DEFAULT_MIN, DOOR1_DEFAULT_MAX, 0, 0},
    {DOOR2_DEFAULT_MIN, DOOR2_DEFAULT_MAX, 0, 0},
    {PUPPET_DEFAULT_MIN, PUPPET_DEFAULT_MAX, 0, 0}
};





int ServoControl::getServoMin(int servoIndex) {
    if (servoIndex >= 0 && servoIndex < NUM_SERVOS) {
        return _servoSettings[servoIndex].minPulse;
    }
    return 0; // Error: Invalid servo index
}

void ServoControl::setServoMin(int servoIndex, int value) {
    if (servoIndex >= 0 && servoIndex < NUM_SERVOS) {
        _servoSettings[servoIndex].minPulse = value;
    }
}

int ServoControl::getServoMax(int servoIndex) {
    if (servoIndex >= 0 && servoIndex < NUM_SERVOS) {
        return _servoSettings[servoIndex].maxPulse;
    }
    return 0; // Error: Invalid servo index
}

void ServoControl::setServoMax(int servoIndex, int value) {
    if (servoIndex >= 0 && servoIndex < NUM_SERVOS) {
        _servoSettings[servoIndex].maxPulse = value;
    }
}






int ServoControl::pwmForAngle(int angle) {
    return map(angle, 0, 180, 1000, 2000); // Example mapping
}

ServoControl::ServoControl(uint8_t address, uint8_t door1Channel, uint8_t door2Channel, uint8_t puppetChannel)
    : _pwm(address), _doorChannels{door1Channel, door2Channel, puppetChannel},
        _animationStartTime(0), _animationStep(0), _animating(false),
        _door1TargetPosition(0), _door2TargetPosition(180), _puppetTargetPosition(90),
        _door1CurrentPosition(0), _door2CurrentPosition(180), _puppetCurrentPosition(90),
        //_servoSpeed(1),
        _doorMoving(false), _puppetMoving(false),
        _door1Min(pwmForAngle(OPENEDPOS)), _door1Max(pwmForAngle(CLOSEDPOS)), _door2Min(pwmForAngle(OPENEDPOS)), _door2Max(pwmForAngle(CLOSEDPOS)), 
        _puppetMin(pwmForAngle(PUPPETMINPOS)), _puppetMax(pwmForAngle(PUPPETMAXPOS)) {
    _pwm.init(); // Corrected: Use pwm.init()
    _pwm.resetDevices(); // Corrected: use pwm.resetDevices()
    _pwm.setPWMFreqServo();
    _pwm.setChannelPWM(_door1Channel, _door1CurrentPosition);
    _pwm.setChannelPWM(_door2Channel, _door2CurrentPosition);
    _pwm.setChannelPWM(_puppetChannel, _puppetCurrentPosition);

#if ENABLE_EEPROM
    EEPROM.begin(32); // Initialize EEPROM with enough space
    _door1Min = EEPROM.read(EEPROM_DOOR1_MIN_ADDR) | (EEPROM.read(EEPROM_DOOR1_MIN_ADDR + 1) << 8) | (EEPROM.read(EEPROM_DOOR1_MIN_ADDR + 2) << 16) | (EEPROM.read(EEPROM_DOOR1_MIN_ADDR + 3) << 24);
    _door1Max = EEPROM.read(EEPROM_DOOR1_MAX_ADDR) | (EEPROM.read(EEPROM_DOOR1_MAX_ADDR + 1) << 8) | (EEPROM.read(EEPROM_DOOR1_MAX_ADDR + 2) << 16) | (EEPROM.read(EEPROM_DOOR1_MAX_ADDR + 3) << 24);
    _door2Min = EEPROM.read(EEPROM_DOOR2_MIN_ADDR) | (EEPROM.read(EEPROM_DOOR2_MIN_ADDR + 1) << 8) | (EEPROM.read(EEPROM_DOOR2_MIN_ADDR + 2) << 16) | (EEPROM.read(EEPROM_DOOR2_MIN_ADDR + 3) << 24);
    _door2Max = EEPROM.read(EEPROM_DOOR2_MAX_ADDR) | (EEPROM.read(EEPROM_DOOR2_MAX_ADDR + 1) << 8) | (EEPROM.read(EEPROM_DOOR2_MAX_ADDR + 2) << 16) | (EEPROM.read(EEPROM_DOOR2_MAX_ADDR + 3) << 24);
    _puppetMin = EEPROM.read(EEPROM_PUPPET_MIN_ADDR) | (EEPROM.read(EEPROM_PUPPET_MIN_ADDR + 1) << 8) | (EEPROM.read(EEPROM_PUPPET_MIN_ADDR + 2) << 16) | (EEPROM.read(EEPROM_PUPPET_MIN_ADDR + 3) << 24);
    _puppetMax = EEPROM.read(EEPROM_PUPPET_MAX_ADDR) | (EEPROM.read(EEPROM_PUPPET_MAX_ADDR + 1) << 8) | (EEPROM.read(EEPROM_PUPPET_MAX_ADDR + 2) << 16) | (EEPROM.read(EEPROM_PUPPET_MAX_ADDR + 3) << 24);
#endif

    // Initialize Servo Settings Array
    _servoSettings[0] = {DOOR1_DEFAULT_MIN, DOOR1_DEFAULT_MAX, pwmForAngle(OPENEDPOS), pwmForAngle(OPENEDPOS), pwmForAngle(OPENEDPOS), SERVO_DEFAULT_SPEED}; // Door 1
    _servoSettings[1] = {DOOR2_DEFAULT_MIN, DOOR2_DEFAULT_MAX, pwmForAngle(CLOSEDPOS), pwmForAngle(CLOSEDPOS), pwmForAngle(CLOSEDPOS), SERVO_DEFAULT_SPEED}; // Door 2
    _servoSettings[2] = {PUPPET_DEFAULT_MIN, PUPPET_DEFAULT_MAX, pwmForAngle(PUPPETPOS), pwmForAngle(PUPPETPOS), pwmForAngle(PUPPETPOS), SERVO_DEFAULT_SPEED}; // Puppet


    // Set initial positions
    for (int i = 0; i < NUM_SERVOS; ++i) {
        _pwm.setChannelPWM(_doorChannels[i], _servoSettings[i].currentPulse);
    }

    _servoSpeed = SERVO_DEFAULT_SPEED;
}

void ServoControl::centerServos() {
    int door1Center = (_door1Min + _door1Max) / 2;
    int door2Center = (_door2Min + _door2Max) / 2;
    int puppetCenter = (_puppetMin + _puppetMax) / 2;

    setServoPulse(_door1Channel, door1Center);
    setServoPulse(_door2Channel, door2Center);
    setServoPulse(_puppetChannel, puppetCenter);

    Serial.println("Servos centered.");
}

void ServoControl::setServoPulse(int channel, int pulse) {
    int minPulse = 0;
    int maxPulse = 0;

    if (channel == _door1Channel) {
        minPulse = _door1Min;
        maxPulse = _door1Max;
    } else if (channel == _door2Channel) {
        minPulse = _door2Min;
        maxPulse = _door2Max;
    } else if (channel == _puppetChannel) {
        minPulse = _puppetMin;
        maxPulse = _puppetMax;
    }

    pulse = constrain(pulse, minPulse, maxPulse);

    _pwm.setChannelPWM(channel, pulse); // Corrected: Use setChannelPWM()
}


//  TODO: setDoor1Min, setDoor1Max, setDoor2Min, setDoor2Max, setPuppetMin, setPuppetMax
void ServoControl::setDoor1Min(int min) {
    _door1Min = min;
    EEPROM.write(EEPROM_DOOR1_MIN_ADDR, (min >> 0) & 0xFF);
    EEPROM.write(EEPROM_DOOR1_MIN_ADDR + 1, (min >> 8) & 0xFF);
    EEPROM.write(EEPROM_DOOR1_MIN_ADDR + 2, (min >> 16) & 0xFF);
    EEPROM.write(EEPROM_DOOR1_MIN_ADDR + 3, (min >> 24) & 0xFF);
    EEPROM.commit();
}

void ServoControl::setDoor1Max(int max) {
    _door1Max = max;
    EEPROM.write(EEPROM_DOOR1_MAX_ADDR, (max >> 0) & 0xFF);
    EEPROM.write(EEPROM_DOOR1_MAX_ADDR + 1, (max >> 8) & 0xFF);
    EEPROM.write(EEPROM_DOOR1_MAX_ADDR + 2, (max >> 16) & 0xFF);
    EEPROM.write(EEPROM_DOOR1_MAX_ADDR + 3, (max >> 24) & 0xFF);
    EEPROM.commit();
}

void ServoControl::setDoor2Max(int max) {
    _door2Max = max;
    EEPROM.write(EEPROM_DOOR2_MAX_ADDR, (max >> 0) & 0xFF);
    EEPROM.write(EEPROM_DOOR2_MAX_ADDR + 1, (max >> 8) & 0xFF);
    EEPROM.write(EEPROM_DOOR2_MAX_ADDR + 2, (max >> 16) & 0xFF);
    EEPROM.write(EEPROM_DOOR2_MAX_ADDR + 3, (max >> 24) & 0xFF);
    EEPROM.commit();
}

void ServoControl::setPuppetMax(int max) {
    _puppetMax = max;
    EEPROM.write(EEPROM_PUPPET_MAX_ADDR, (max >> 0) & 0xFF);
    EEPROM.write(EEPROM_PUPPET_MAX_ADDR + 1, (max >> 8) & 0xFF);
    EEPROM.write(EEPROM_PUPPET_MAX_ADDR + 2, (max >> 16) & 0xFF);
    EEPROM.write(EEPROM_PUPPET_MAX_ADDR + 3, (max >> 24) & 0xFF);
    EEPROM.commit();
}

void ServoControl::setDoor2Min(int min) {
    _door2Min = min;
    EEPROM.write(EEPROM_DOOR2_MIN_ADDR, (min >> 0) & 0xFF);
    EEPROM.write(EEPROM_DOOR2_MIN_ADDR + 1, (min >> 8) & 0xFF);
    EEPROM.write(EEPROM_DOOR2_MIN_ADDR + 2, (min >> 16) & 0xFF);
    EEPROM.write(EEPROM_DOOR2_MIN_ADDR + 3, (min >> 24) & 0xFF);
    EEPROM.commit();
}

void ServoControl::setPuppetMin(int min) {
    _puppetMin = min;
    EEPROM.write(EEPROM_PUPPET_MIN_ADDR, (min >> 0) & 0xFF);
    EEPROM.write(EEPROM_PUPPET_MIN_ADDR + 1, (min >> 8) & 0xFF);
    EEPROM.write(EEPROM_PUPPET_MIN_ADDR + 2, (min >> 16) & 0xFF);
    EEPROM.write(EEPROM_PUPPET_MIN_ADDR + 3, (min >> 24) & 0xFF);
    EEPROM.commit();
}




void ServoControl::openDoors() {
    _door1TargetPosition = pwmForAngle(OPENEDPOS);
    _door2TargetPosition = pwmForAngle(OPENEDPOS);
    _doorMoving = true;
}

void ServoControl::closeDoors() {
    _door1TargetPosition = pwmForAngle(CLOSEDPOS);
    _door2TargetPosition = pwmForAngle(CLOSEDPOS);
    _doorMoving = true;
}

void ServoControl::animatePuppet() {
    _animationStartTime = millis();
    _animationStep = 0;
    _animating = true;
    }

void ServoControl::stopPuppet() {
    _puppetTargetPosition = 90;
    _puppetMoving = true;
}



void ServoControl::update() {
    for (int i = 0; i < NUM_SERVOS; ++i) {
        if (_servoSettings[i].currentPulse != _servoSettings[i].targetPulse) {
            if (_servoSettings[i].currentPulse < _servoSettings[i].targetPulse) {
                _servoSettings[i].currentPulse += _servoSpeed;
                if (_servoSettings[i].currentPulse > _servoSettings[i].targetPulse) {
                    _servoSettings[i].currentPulse = _servoSettings[i].targetPulse;
                }
            } else {
                _servoSettings[i].currentPulse -= _servoSpeed;
                if (_servoSettings[i].currentPulse < _servoSettings[i].targetPulse) {
                    _servoSettings[i].currentPulse = _servoSettings[i].targetPulse;
                }
            }
            _pwm.setChannelPWM(_doorChannels[i], _servoSettings[i].currentPulse);
        }
    }

    // Puppet Animation
    if (_animating) {
        unsigned long currentTime = millis();
        if (currentTime - _animationStartTime >= 250) {
            _animationStartTime = currentTime;
            if (_animationStep % 2 == 0) {
                _servoSettings[2].targetPulse = 1400; // Puppet up
            } else {
                _servoSettings[2].targetPulse = 1600; // Puppet down
            }
            _animationStep++;
            if (_animationStep >= 6) {
                _servoSettings[2].targetPulse = 1500; // Puppet center
                _animating = false;
            }
        }
    }
}

void ServoControl::incrementDoor1() {
    _door1TargetPosition = min(_door1TargetPosition + 1, 180);
    _doorMoving = true;
}

void ServoControl::decrementDoor1() {
    _door1TargetPosition = max(_door1TargetPosition - 1, 0);
    _doorMoving = true;
}

void ServoControl::incrementDoor2() {
    _door2TargetPosition = min(_door2TargetPosition + 1, 180);
    _doorMoving = true;
}

void ServoControl::decrementDoor2() {
    _door2TargetPosition = max(_door2TargetPosition - 1, 0);
    _doorMoving = true;
}

void ServoControl::incrementPuppet() {
    _puppetTargetPosition = min(_puppetTargetPosition + 1, 180);
    _puppetMoving = true;
}

void ServoControl::decrementPuppet() {
    _puppetTargetPosition = max(_puppetTargetPosition - 1, 0);
    _puppetMoving = true;
}

int ServoControl::getDoor1Position() {
    return _door1CurrentPosition;
}

int ServoControl::getDoor2Position() {
    return _door2CurrentPosition;
}

int ServoControl::getPuppetPosition() {
    return _puppetCurrentPosition;
}

void ServoControl::setServoSpeed(int servoIndex, int value) {
    if (servoIndex >= 0 && servoIndex < NUM_SERVOS) {
        _servoSettings[servoIndex].speed = value;
    }
}

void ServoControl::printServoVariables() {
    Serial.println("Servo Variables:");
    Serial.print("  Door 1 Channel: "); Serial.println(_door1Channel);
    Serial.print("  Door 2 Channel: "); Serial.println(_door2Channel);
    Serial.print("  Puppet Channel: "); Serial.println(_puppetChannel);
    Serial.print("  Servo Speed: "); Serial.println(_servoSpeed);
    Serial.print("  Door 1 Min: "); Serial.println(_door1Min);
    Serial.print("  Door 1 Max: "); Serial.println(_door1Max);
    Serial.print("  Door 2 Min: "); Serial.println(_door2Min);
    Serial.print("  Door 2 Max: "); Serial.println(_door2Max);
    Serial.print("  Puppet Min: "); Serial.println(_puppetMin);
    Serial.print("  Puppet Max: "); Serial.println(_puppetMax);
}

void ServoControl::adjustServoPulse(int servoIndex, int increment) {
    if (servoIndex >= 0 && servoIndex < NUM_SERVOS) {
        _servoSettings[servoIndex].currentPulse += increment;
        _pwm.setChannelPWM(_doorChannels[servoIndex], _servoSettings[servoIndex].currentPulse);
    }
}

int ServoControl::getServoCurrentPulse(int servoIndex) {
    if (servoIndex >= 0 && servoIndex < NUM_SERVOS) {
        return _servoSettings[servoIndex].currentPulse;
    }
    return 0; // Error: Invalid servo index
}

/**/