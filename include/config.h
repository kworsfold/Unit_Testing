#ifndef CONFIG_H
#define CONFIG_H


#define ENABLE_EEPROM 0 // Set to 1 to enable EEPROM, 0 to disable




// I2C Pin Definitions
#define I2C_SDA 21
#define I2C_SCL 22

// PCA9685 Settings
const uint8_t PCA9685_ADDRESS = 0x40;
const uint8_t DOOR1_CHANNEL = 0;
const uint8_t DOOR2_CHANNEL = 1;
const uint8_t PUPPET_CHANNEL = 2;

#define SERVO_DEFAULT_SPEED 100 // Example default speed (adjust as needed)


// EEPROM Addresses
#define EEPROM_DOOR1_MIN_ADDR 0
#define EEPROM_DOOR1_MAX_ADDR 4
#define EEPROM_DOOR2_MIN_ADDR 8
#define EEPROM_DOOR2_MAX_ADDR 12
#define EEPROM_PUPPET_MIN_ADDR 16
#define EEPROM_PUPPET_MAX_ADDR 20
#define EEPROM_SERVO_SPEED_ADDR 24

// Servo Default Min/Max
#define DOOR1_DEFAULT_MIN 1000
#define DOOR1_DEFAULT_MAX 2000
#define DOOR2_DEFAULT_MIN 1000
#define DOOR2_DEFAULT_MAX 2000
#define PUPPET_DEFAULT_MIN 1000
#define PUPPET_DEFAULT_MAX 2000

// Servo Default Angle Positions
#define OPENEDPOS 0
#define CLOSEDPOS 180
#define PUPPETPOS 90

#endif // CONFIG_H