// Watchdog defines
//#define WATCHDOG_ENABLED
#define WATCHDOG_TIMER 1000 // 1 second

// I2C defines
#define I2C0_ENABLED
#define I2C1_ENABLED
#define I2C0_SDA 20
#define I2C0_SCL 21
#define I2C1_SDA 2
#define I2C1_SCL 3

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
//#define UART0_ENABLED

// Flash defines
//#define SAVE_ENABLED


#define EXTERNAL_ADC_ENABLED


// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5


#define ADS1115_GND_ADDR 0x48
#define ADS1115_VCC_ADDR 0x49

#define FLASH_TARGET_OFFSET (8 * 256 * 1024) // choosing to start at 8*256K = 1.5MB (512KB remaining for user data)

// ADS1115 resolutions
// 66.7 mV/A = 0.0667 V/A
// formula: I = (ADC * 3.3V) / (0.0667 V/A * 65535)
#define ADS1115_30A_RESOLUTION 3.3 / (0.0667 * 65535)
#define ADS1115_5A_RESOLUTION 3.3 / (0.185 * 65535)
#define DEW1_CURRENT_RESOLUTION ADS1115_5A_RESOLUTION
#define DEW2_CURRENT_RESOLUTION ADS1115_5A_RESOLUTION
#define DEW3_CURRENT_RESOLUTION ADS1115_5A_RESOLUTION
#define DC1_CURRENT_RESOLUTION ADS1115_30A_RESOLUTION
#define DC2_CURRENT_RESOLUTION ADS1115_5A_RESOLUTION
#define DC3_CURRENT_RESOLUTION ADS1115_5A_RESOLUTION
#define DC4_CURRENT_RESOLUTION ADS1115_5A_RESOLUTION
#define DC5_CURRENT_RESOLUTION ADS1115_5A_RESOLUTION
#define INPUT_CURRENT_RESOLUTION ADS1115_30A_RESOLUTION

#define INPUT_VOLTAGE_RESISTOR 2/(10+2)


// NTC
#define NTC_RESISTOR 1000
#define NTC_A 0.070187488
#define NTC_B 0.034789527
#define NTC_C 0.002285809
#define NTC1_ENABLED
#define NTC2_ENABLED
#define NTC3_ENABLED


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
    char16_t dew1_name[64] = {'\0'};
    char16_t dew2_name[64] = {'\0'};
    char16_t dew3_name[64] = {'\0'};
    char16_t dc1_name[64] = {'\0'};
    char16_t dc2_name[64] = {'\0'};
    char16_t dc3_name[64] = {'\0'};
    char16_t dc4_name[64] = {'\0'};
    char16_t dc5_name[64] = {'\0'};
};
