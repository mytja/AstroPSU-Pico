// Watchdog defines
//#define WATCHDOG_ENABLED
#define WATCHDOG_TIMER 1000 // 1 second

#define DEBUG_WAIT_FOR_USB

// I2C defines
#define I2C0_ENABLED
#define I2C1_ENABLED
#define I2C0_SDA 20
#define I2C0_SCL 21
#define I2C1_SDA 2
#define I2C1_SCL 3
#define I2C_DEBUG

// Dew heater PWMs
#define DEW3 6
#define DEW2 7
#define DEW1 8

// DC jacks
#define DC1 9
#define DC2 10
#define DC3 11
#define DC4 12
#define DC5 13

// UART defines

// Flash defines
//#define SAVE_ENABLED


#define EXTERNAL_ADC_ENABLED


// UART defines
#define UART_ENABLED
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart0
#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1
#define UART1_TX_PIN 4
#define UART1_RX_PIN 5

// GPS defines
#define GPS0_ENABLED
#define PPS_PIN_GPS0 17
#define GPS0_ENABLE 18


#define ADS1115_GND_ADDR 0x48
#define ADS1115_VCC_ADDR 0x49

#define FLASH_TARGET_OFFSET (8 * 256 * 1024) // choosing to start at 8*256K = 1.5MB (512KB remaining for user data)

// ADS1115 resolutions
#define ADS1115_BIT_TO_MV 0.125

// ADS
// Voltage on the ACS712 current sensor is 3.3V, however Pico's linear voltage regulator
// is not quite spot on, so we account for the usually higher voltage
// Base voltage on the ACS712 is half the VCC.
#define ACS712_REPEATED_READS 50
#define ACS712_OFFSET 1688
#define ACS712_30A_RESOLUTION 66.7
#define ACS712_5A_RESOLUTION 185.0
#define DEW1_CURRENT_RESOLUTION ACS712_5A_RESOLUTION
#define DEW2_CURRENT_RESOLUTION ACS712_5A_RESOLUTION
#define DEW3_CURRENT_RESOLUTION ACS712_5A_RESOLUTION
#define DC1_CURRENT_RESOLUTION ACS712_30A_RESOLUTION
#define DC2_CURRENT_RESOLUTION ACS712_30A_RESOLUTION
#define DC3_CURRENT_RESOLUTION ACS712_5A_RESOLUTION
#define DC4_CURRENT_RESOLUTION ACS712_30A_RESOLUTION
#define DC5_CURRENT_RESOLUTION ACS712_5A_RESOLUTION
#define INPUT_CURRENT_RESOLUTION ACS712_30A_RESOLUTION


// (R2+R1)/R1 =>
// (10+2)/2
#define INPUT_VOLTAGE_RESISTOR 6


// NTC
#define NTC_RESISTOR 1000
#define NTC_A 0.070187488
#define NTC_B 0.034789527
#define NTC_C 0.002285809
#define NTC1_ENABLED
#define NTC2_ENABLED
#define NTC3_ENABLED

// SHT3x temperature & humidity sensor
#define SHT3X_ENABLED
#define SHT3X1_I2C i2c0
#define SHT3X1_ADDRESS SHT3X_I2C_ADDRESS_A
#define SHT3X2_I2C i2c0
#define SHT3X2_ADDRESS SHT3X_I2C_ADDRESS_B
#define SHT3X3_I2C i2c1
#define SHT3X3_ADDRESS SHT3X_I2C_ADDRESS_A

struct State {
    uint16_t dew1 = 0;
    uint16_t dew2 = 0;
    uint16_t dew3 = 0;
    bool dc1 = false;
    bool dc2 = false;
    bool dc3 = false;
    bool dc4 = false;
    bool dc5 = false;
    bool autodew = false;
    bool gps_sleep = false;
    char dew1_name[64] = {'\0'};
    char dew2_name[64] = {'\0'};
    char dew3_name[64] = {'\0'};
    char dc1_name[64] = {'\0'};
    char dc2_name[64] = {'\0'};
    char dc3_name[64] = {'\0'};
    char dc4_name[64] = {'\0'};
    char dc5_name[64] = {'\0'};
};
