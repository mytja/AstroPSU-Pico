// General bits of advice for future self:
// Try to use AS FEW vectors and strings as possible. Use C-native types if possible.
// C++ types in general are dynamic, therefore allocating memory using malloc(). I've
// found that a lot of the times, Pi Pico doesn't deallocate this RAM, despite running
// out of scope. Especially don't do malloc() operations inside interrupts. This won't
// end well, oftentimes hanging the core. Then you'll need to do the entire debugging
// procedure with gdb:
// 1. C:\\Users\\Mitja\\.pico-sdk\\openocd\\0.12.0+dev\\openocd.exe -c "gdb_port 50000" -c "tcl_port 50001" -c "telnet_port 50002" -s "C:\\Users\\Mitja/.pico-sdk/openocd/0.12.0+dev/scripts" -f "c:/Users/Mitja/.vscode/extensions/marus25.cortex-debug-1.12.1/support/openocd-helpers.tcl" -f "interface/cmsis-dap.cfg" -f "target/rp2040.cfg" -c "adapter speed 5000"
// 2. C:\\Users\\Mitja\\.pico-sdk\\toolchain\\14_2_Rel1\\bin\\arm-none-eabi-gdb .\build\AstroPSU-Pico.elf
// Inside the gdb terminal:
// target remote localhost:50000
// thread 2 (to select core 1 instead of core 0)
// continue&
// And other commands, such as (backtrace/bt, breakpoint)

// Watchdog defines
#define WATCHDOG_ENABLED
//#define WATCHDOG_DEBUG
#define WATCHDOG_TIMER 5000 // 5 seconds

//#define DEBUG_WAIT_FOR_USB
const bool DEBUG_INIT_MESSAGES = false;
//#define DEBUG_GPS
//#define DEBUG_CRASH
//#define DEBUG_CORE1_ITERATIONS
//#define DEBUG_FLASH

// DO NOT REMOVE THIS TOGGLE - IF THIS SHIT ISN'T TURNED ON, IT FOR SOME DUMBFUCK REASON
// CRASHES main(). DO NOT TURN THIS OFF!!!!!!!!!!!!!!
#define DEBUG_I2C

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
//#define AUTODEW_FORCE_TEMPERATURE 7.2

#define FAN_ENABLED
#define FAN 22
#define FAN_MIN_TEMP 33.0f
#define FAN_MAX_TEMP_DELTA 5.0f
#define FAN_USE_CURRENT_ALONGSIDE_TEMP
#define FAN_MAX_CURRENT 7.0f

// DC jacks
#define DC1 9
#define DC2 10
#define DC3 11
#define DC4 12
#define DC5 13

// UART defines

// Flash defines
#define SAVE_ENABLED
#define READ_FLASH_ON_BOOT


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

const uint32_t FLASH_TARGET_OFFSET = (6 * 256 * 1024); // choosing to start at 6*256K = 1.5MB (512KB remaining for user data)

// ADS1115 resolutions
#define ADS1115_BIT_TO_MV 0.125

// ACS712
// Voltage on the ACS712 current sensor is 3.3V, however Pico's linear voltage regulator
// is not quite spot on, so we account for the usually higher voltage
// Base voltage on the ACS712 is half the VCC.
#define ACS712_CALIBRATE
#define ACS712_REPEATED_READS 20
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
#define ACS712_CURRENT_MULTIPLY 1.64 // Inherited from testing
#define INPUT_BASE_CURRENT 0.085

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
#define HUMIDITY_THRESHOLD 0

// BMI160 gyroscope and accelerometer
#define BMI160_ENABLED
#define BMI160_1_I2C i2c0
#define BMI160_1_ADDRESS BMI160_I2C_ADDRESS_B
#define BMI160_2_I2C i2c0
#define BMI160_2_ADDRESS BMI160_I2C_ADDRESS_A
#define BMI160_3_I2C i2c1
#define BMI160_3_ADDRESS BMI160_I2C_ADDRESS_B

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
    uint16_t dew1_zero = 0;
    uint16_t dew2_zero = 0;
    uint16_t dew3_zero = 0;
    uint16_t dc1_zero = 0;
    uint16_t dc2_zero = 0;
    uint16_t dc3_zero = 0;
    uint16_t dc4_zero = 0;
    uint16_t dc5_zero = 0;
    uint16_t input_zero = 0;
    bool disable_fan = false;
};

struct Data {
    State* state;
    float input_current = 0.0;
    float dew1_current = 0.0;
    float dew2_current = 0.0;
    float dew3_current = 0.0;
    float dc1_current = 0.0;
    float dc2_current = 0.0;
    float dc3_current = 0.0;
    float dc4_current = 0.0;
    float dc5_current = 0.0;
    float ext1_analog_temp = 0.0;
    float ext2_analog_temp = 0.0;
    float ext3_analog_temp = 0.0;
    float dew_point = 0.0;
    float sht1_temp = 0.0;
    float sht2_temp = 0.0;
    float sht3_temp = 0.0;
    float sht1_hum = 0.0;
    float sht2_hum = 0.0;
    float sht3_hum = 0.0;
    float input_voltage = 0.0;
    float gps1_lat = 0.0;
    float gps1_lng = 0.0;
    float gps1_elevation = 0.0;
    int gps1_satellite_count = 0;
    float gyro_x = 0.0;
    float gyro_y = 0.0;
    float cpu_temp = 0.0;
};
