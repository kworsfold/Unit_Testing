#include "LightingEffects.h"
#include <Arduino.h>

void LightingEffects::lightningEffect() {
    for (int i = 0; i < 5; i++) {
        int randomPixel = random(_numPixels);
        int randomBrightness = random(100, 255);
        pixels.setPixelColor(randomPixel, pixels.Color(randomBrightness, randomBrightness, randomBrightness));
        pixels.show();
        delay(random(50, 200));
        clear();
        delay(random(100, 300));
    }
};

void LightingEffects::presenceDetectedEffect() {
    for (int i = 0; i < _numPixels; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 0, 0)); // Red
        pixels.show();
        delay(50);
    }
    delay(1000); // Keep red for 1 second
    clear();
}

void LightingEffects::clear() {
    for (int i = 0; i < _numPixels; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }
    pixels.show();
}

void LightingEffects::initLEDs() {
    int ledPins[]= {LED_1_Pin, LED_2_Pin, LED_3_Pin, LED_4_Pin, LED_5_Pin};
    int numLEDs = sizeof(ledPins) / sizeof(ledPins[0]);

    for (int i = 0; i < numLEDs; i++) {
        pinMode(ledPins[i], OUTPUT);
    }
}

void LightingEffects::setPixelColor(uint16_t n, uint32_t c) {
    pixels.setPixelColor(n, c);
}

void LightingEffects::show() {
    pixels.show();
}