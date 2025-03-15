#ifndef ServoControl_h
#define ServoControl_h

#include <PCA9685.h>

/*
    int minPulse, 
    int maxPulse, 
    int currentPulse, 
    int speed 
    int lastPulse;
    int targetPulse;
        servoSettings[SERVO] = { int minPulse, int maxPulse, int currentPulse, int speed, int lastPulse, int targetPulse },
*/
struct ServoSettings {
    int minPulse;
    int maxPulse;
    int currentPulse;
    int lastPulse; // Add lastPulse
    int targetPulse; // Add targetPulse
    int speed;
};

#define NUM_SERVOS 3 // Define the number of servos

class ServoControl {

public:
    ServoControl(uint8_t address, uint8_t door1Channel, uint8_t door2Channel, uint8_t puppetChannel);
    void setServoPulse(int channel, int pulse);
    int pwmForAngle(int angle);
    void openDoors();
    void closeDoors();
    void animatePuppet();
    void stopPuppet();
    void update();
    void setDoor1Min(int min);
    void setDoor1Max(int max);
    void setDoor2Min(int min);
    void setDoor2Max(int max);
    void setPuppetMin(int min);
    void setPuppetMax(int max);

    uint8_t _doorChannels[NUM_SERVOS];

/*      TODO: Servo Settings now as array
    int getDoor1Min() const { return _door1Min; }
    int getDoor1Max() const { return _door1Max; }
    int getDoor2Min() const { return _door2Min; }
    int getDoor2Max() const { return _door2Max; }
    int getPuppetMin() const { return _puppetMin; }
    int getPuppetMax() const { return _puppetMax; }
*/
    int getServoMin(int servoIndex);
    void setServoMin(int servoIndex, int value);
    int getServoMax(int servoIndex) ;
    void setServoMax(int servoIndex, int value);



    uint8_t getDoor1Channel() const { return _door1Channel; }
    uint8_t getDoor2Channel() const { return _door2Channel; }
    uint8_t getPuppetChannel() const { return _puppetChannel; }

    int getDoor1Position();
    int getDoor2Position();
    int getPuppetPosition();

    void incrementDoor1();
    void decrementDoor1();
    void incrementDoor2();
    void decrementDoor2();
    void incrementPuppet();
    void decrementPuppet();
    void centerServos();
    void printServoVariables();

    int getServoSpeed() const { return _servoSpeed; }
    void setServoSpeed(int servoIndex, int value);
    void setServoSpeed(int speed);
    void adjustServoPulse(int servoIndex, int increment) ;
    int getServoCurrentPulse(int servoIndex);

private:
    PCA9685 _pwm;
    ServoSettings _servoSettings[NUM_SERVOS];

    uint8_t _door1Channel;
    uint8_t _door2Channel;
    uint8_t _puppetChannel;
    unsigned long _animationStartTime;
    int _animationStep;
    bool _animating;
    int _door1TargetPosition;
    int _door2TargetPosition;
    int _puppetTargetPosition;
    int _door1CurrentPosition;
    int _door2CurrentPosition;
    int _puppetCurrentPosition;
    int _servoSpeed;
    bool _doorMoving;
    bool _puppetMoving;
    int _door1Min;
    int _door1Max;
    int _door2Min;
    int _door2Max;
    int _puppetMin;
    int _puppetMax;
};

#endif