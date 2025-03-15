#ifndef LightingEffects_h
#define LightingEffects_h

#include <Adafruit_NeoPixel.h>

class LightingEffects {
    public:
    void lightningEffect();
    void presenceDetectedEffect();
    void clear();
    void initLEDs();
    void setPixelColor(uint16_t n, uint32_t c);
    void show();
    int getLED1Pin() const { return LED_1_Pin; }
    int getLED2Pin() const { return LED_2_Pin; }
    int getLED3Pin() const { return LED_3_Pin; }
    int getLED4Pin() const { return LED_4_Pin; }
    int getLED5Pin() const { return LED_5_Pin; }

private:
    Adafruit_NeoPixel pixels;
    int _numPixels;
    const int LED_4_Pin = 5;
    const int LED_5_Pin = 18;
    const int LED_1_Pin = 4;
    const int LED_2_Pin = 14;
    const int LED_3_Pin = 26;
    const int NEOPIXEL_PIN = 19;
    const int NEOPIXEL_COUNT = 30;
};

#endif