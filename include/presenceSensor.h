#ifndef PresenceSensor_h
#define PresenceSensor_h

#include <Arduino.h>

class PresenceSensor {
public:
    PresenceSensor();
    void update();
    bool isDetected();

private:
    bool _detected;
    unsigned long _lastDetectionTime;
    unsigned long _detectionStartTime;
    int _detectionCounter; // Counter for filtering
    const int _detectionThreshold = 3; // Threshold for detection
    const int _noDetectionThreshold = -3; // Threshold for no detection
};

#endif