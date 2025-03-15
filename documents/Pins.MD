1. ESP32 Connections:

LD2410 Presence Sensor:
RX (Receive) to ESP32 GPIO 16
TX (Transmit) to ESP32 GPIO 17
VCC (3.3V or 5V, depending on your LD2410)
GND (Ground)

PCA9685 Servo Driver:
SDA (Serial Data) to ESP32 SDA pin (typically GPIO 21)
SCL (Serial Clock) to ESP32 SCL pin (typically GPIO 22)
VCC (5V)
GND (Ground)

NeoPixel LED Strip:
Data In to ESP32 GPIO 5 (or your defined NEOPIXEL_PIN)
VCC (5V)
GND (Ground)

Servo Motors (Connected to PCA9685):
Servo 1 (Door 1) signal to PCA9685 channel 0 (or your defined DOOR1_CHANNEL)
Servo 2 (Door 2) signal to PCA9685 channel 1 (or your defined DOOR2_CHANNEL)
Servo 3 (Puppet) signal to PCA9685 channel 2 (or your defined PUPPET_CHANNEL)
Servo VCC (5V) to PCA9685 VCC (5V)
Servo GND (Ground) to PCA9685 GND (Ground)



3. Power Supply:

Provide a 5V power supply that can handle the current requirements of the servos and NeoPixel LED strip.
Connect the 5V supply to the PCA9685's VCC and the NeoPixel's VCC.
Connect the 5V supply's GND to the ESP32's GND, the PCA9685's GND, and the NeoPixel's GND.
Important Notes:

I2C Pull-up Resistors:
Ensure that you have pull-up resistors (typically 4.7k ohms) on the SDA and SCL lines between the ESP32 and the PCA9685.
Power Distribution:
Consider using a power distribution board or terminal block to distribute power to the various components.
Wire Lengths:
Keep wire lengths as short as possible to minimize signal interference.
Grounding:
Ensure that all ground connections are properly connected.
Pin Assignments:
Double check that the pin assignments in your code match the physical connections.
Serial 2:
Please double check that your ESP32 board does support the use of serial 2.
By carefully following these connection guidelines, you'll be able to ensure that your hardware is properly connected and functioning correctly.