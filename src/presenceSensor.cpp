#include "PresenceSensor.h"
#include <Arduino.h>
#include <ld2410.h>

const int RADAR_RX_PIN = 16; // RX pin for LD2410
const int RADAR_TX_PIN = 17; // TX pin for LD2410

ld2410 radar; // Create an LD2410 object

// Constructor
PresenceSensor::PresenceSensor() : _detected(false), _lastDetectionTime(0), _detectionStartTime(0), _detectionCounter(0) {
  // Use Serial2 for LD2410 communication
  Serial2.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN); // Initialize Serial2
  radar.begin(Serial2); // Initialize the LD2410 sensor
}



// Method to update the presence sensor
// This method should be called in the main loop
void PresenceSensor::update() {
    radar.read(); // Process LD2410 data

    if (radar.presenceDetected()) {
        _detectionCounter++;
        if (_detectionCounter >= _detectionThreshold) {
        _detected = true;
        _detectionCounter = _detectionThreshold; // Prevent overflow
        _lastDetectionTime = millis();
        }
    } else {
        _detectionCounter--;
        if (_detectionCounter <= _noDetectionThreshold) {
        _detected = false;
        _detectionCounter = _noDetectionThreshold; // Prevent overflow
        if (millis() - _lastDetectionTime > 30000) { // Using 30 seconds as default interval
            _lastDetectionTime = millis();
        }
        }
    } 
}

//  Getter for _detected presence
bool PresenceSensor::isDetected() {
    return _detected;
}