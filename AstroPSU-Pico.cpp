#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/flash.h"
#include "tusb.h"

#include "AstroPSU-Pico.h"
#include "ads1115.h"
#include "sht3x.h"
#include "bmi160.h"

#include <nmea/sentence.hpp>
#include <nmea/message/gga.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <pico/multicore.h>

using namespace std;

State state;

struct ads1115_adc adc1;
struct ads1115_adc adc2;
struct ads1115_adc adc3;
struct sht3x sht3x1;
struct sht3x sht3x2;
struct sht3x sht3x3;
struct bmi160 bmi160_1;
struct bmi160 bmi160_2;
struct bmi160 bmi160_3;

string gps0_rx;
double lat = 0, lng = 0;
float elevation = 0;
int satelliteNum = 0;

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

uint32_t millis() {
    return to_ms_since_boot(get_absolute_time());
}

void save_data() {
    // Flush any pending output before starting
    //cout << "Writing data!" << endl;

    uint8_t *myDataAsBytes = (uint8_t *)&state;
    int myDataSize = sizeof(state);

    //cout << "Data size is " << myDataSize << endl;

    int writeSize = (myDataSize / FLASH_PAGE_SIZE) + 1;
    int sectorCount = ((writeSize * FLASH_PAGE_SIZE) / FLASH_SECTOR_SIZE) + 1;
    //cout << "Writing size " << writeSize << " with sector count " << sectorCount << endl;

    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE * sectorCount);
    flash_range_program(FLASH_TARGET_OFFSET, myDataAsBytes, FLASH_PAGE_SIZE * writeSize);
    restore_interrupts(interrupts);
}

void read_data() {
    const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(&state, flash_target_contents, sizeof(state));
}

std::vector<std::string> split(const std::string& str, const std::string& delimiter) {
    std::vector<std::string> tokens;
    if(str.empty()) {
        return tokens;
    }

    size_t start = 0;
    size_t end = 0;

    while((end = str.find(delimiter, start)) != std::string::npos) {
        // Only add non-empty segments
        if(end > start) {
            tokens.push_back(str.substr(start, end - start));
        }
        start = end + delimiter.length();
    }

    // Add the last part if it exists
    if(start < str.length()) {
        tokens.push_back(str.substr(start));
    }

    return tokens;
}

void pwm_setup(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    // Get PWM slice
    uint slice_num = pwm_gpio_to_slice_num(pin);
    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config, 4.f);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(pin, 0);
}

void gpio_setup(int pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
}

int translate_to_pin(const string &pin) {
    if(pin == "DEW1")
        return DEW1;
    if(pin == "DEW2")
        return DEW2;
    if(pin == "DEW3")
        return DEW3;
    if(pin == "DC1")
        return DC1;
    if(pin == "DC2")
        return DC2;
    if(pin == "DC3")
        return DC3;
    if(pin == "DC4")
        return DC4;
    if(pin == "DC5")
        return DC5;
    return -1;
}

float temperature_calc_ntc(uint16_t adc) {
    // https://arduinodiy.wordpress.com/2015/11/10/measuring-temperature-with-ntc-the-steinhart-hart-formula/
    float ntc_resistance = NTC_RESISTOR / ((65535.0F / (float)adc) - 1);
    // https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
    float temperature = pow(NTC_A + NTC_B * log(ntc_resistance) + NTC_C * pow(log(ntc_resistance), 3), -1);
    // To convert from Kelvins to degrees Celcius
    return temperature - 273.15;
}

uint16_t adc_read(ads1115_adc &adc, int pin, int repeat) {
    auto mux = ADS1115_MUX_SINGLE_0;
    if(pin == 1)
        mux = ADS1115_MUX_SINGLE_1;
    if(pin == 2)
        mux = ADS1115_MUX_SINGLE_2;
    if(pin == 3)
        mux = ADS1115_MUX_SINGLE_3;
    ads1115_set_input_mux(mux, &adc);
    ads1115_write_config(&adc);

    // Mediana
    vector<uint32_t> adc_values(repeat);
    uint16_t adc_value;
    for(int i = 0; i < repeat; i++) {
        ads1115_read_adc(&adc_value, &adc);
        adc_values[i] = adc_value;
    }
    sort(adc_values.begin(), adc_values.end());

    adc_value = adc_values[(repeat - 1) / 2];
    return adc_value;
}

// ADS1115 ima omejitev branja 860 branj/sekundo
// Konzervativno izberemo 50 ponavljanj branj
uint16_t adc_read_value(const string &device) {
    if(device == "DEW3_CURRENT")
        return adc_read(adc1, 0, ACS712_REPEATED_READS);
    if(device == "DEW2_CURRENT")
        return adc_read(adc1, 1, ACS712_REPEATED_READS);
    if(device == "DEW1_CURRENT")
        return adc_read(adc1, 2, ACS712_REPEATED_READS);
    if(device == "DC1_CURRENT")
        return adc_read(adc1, 3, ACS712_REPEATED_READS);
    if(device == "DC2_CURRENT")
        return adc_read(adc2, 0, ACS712_REPEATED_READS);
    if(device == "DC3_CURRENT")
        return adc_read(adc2, 1, ACS712_REPEATED_READS);
    if(device == "DC4_CURRENT")
        return adc_read(adc2, 2, ACS712_REPEATED_READS);
    if(device == "DC5_CURRENT")
        return adc_read(adc2, 3, ACS712_REPEATED_READS);
    if(device == "INPUT_CURRENT")
        return adc_read(adc3, 0, ACS712_REPEATED_READS);
    if(device == "EXT1_ANALOG_TEMP")
        return adc_read(adc3, 1, 20);
    if(device == "EXT2_ANALOG_TEMP")
        return adc_read(adc3, 2, 20);
    if(device == "EXT3_ANALOG_TEMP")
        return adc_read(adc3, 3, 20);
    if(device == "INPUT_VOLTAGE") {
        // TODO: repeated reads
        adc_select_input(2);
        return adc_read();
    }
    return 0;
}

void i2c_debug(bool isForce) {
    if(DEBUG_INIT_MESSAGES || isForce) {
        cout << "\nAstroPSU I2C Debug tool";
        cout << "\n\nI2C0\n";
        cout << "   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F";
    }
    for(int addr = 0; addr < (1 << 7); ++addr) {
        if(addr % 16 == 0) {
            if(DEBUG_INIT_MESSAGES || isForce)
                cout << endl << addr / 16 << "0 ";
        }

        int ret;
        uint8_t rxdata;
        if(reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c0, addr, &rxdata, 1, false);

        if(DEBUG_INIT_MESSAGES || isForce)
            cout << (ret < 0 ? "." : "@") << "  ";
    }
    if(DEBUG_INIT_MESSAGES || isForce) {
        cout << "\n\nI2C1\n";
        cout << "   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F";
    }

    for(int addr = 0; addr < (1 << 7); ++addr) {
        if(addr % 16 == 0) {
            if(DEBUG_INIT_MESSAGES || isForce)
                cout << endl << addr / 16 << "0 ";
        }

        int ret;
        uint8_t rxdata;
        if(reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);

        if(DEBUG_INIT_MESSAGES || isForce)
            cout << (ret < 0 ? "." : "@") << "  ";
    }
    if(DEBUG_INIT_MESSAGES || isForce)    cout << endl;
}

void gps0_callback() {
    char ch;
    //cout << "GPS0 Callback from core " << get_core_num() << endl;
    //cout << "Callback " << uart_is_readable(UART_ID) << endl;
    while(uart_is_readable(UART_ID)) {
        ch = uart_getc(UART_ID);
        //cout << ch;
        gps0_rx += ch;
        if(ch == '\n') {
            cout << gps0_rx << flush;
            //cout << flush;
            try {
                nmea::sentence nmea_sentence(gps0_rx);
                if(nmea_sentence.type() == "GGA") {
                    nmea::gga gga(nmea_sentence);
                    if(gga.latitude.exists()) lat = gga.latitude.get();
                    if(gga.longitude.exists()) lng = gga.longitude.get();
                    if(gga.altitude.exists()) elevation = gga.altitude.get();
                    if(gga.satellite_count.exists()) satelliteNum = gga.satellite_count.get();
                }
            } catch(exception& e) {
                cout << "Exception received in GPS with " << gps0_rx << " - exception: " << e.what() << endl;
            }
            gps0_rx = "";
        }
    }
}

double dew_point(double tempC, double humidity) {
    double a = 17.62;
    double b = 243.12;
    double gamma = (a * tempC) / (b + tempC) + log(humidity / 100.0);
    return (b * gamma) / (a - gamma);
}

pair<double, double> get_dew_point() {
    double temp = 0, hum = 0;
    int count = 0;
    if(sht3x1.humidity >= HUMIDITY_THRESHOLD) {
        temp += sht3x1.temperature;
        hum += sht3x1.humidity;
        count++;
    }
    if(sht3x2.humidity >= HUMIDITY_THRESHOLD) {
        temp += sht3x2.temperature;
        hum += sht3x2.humidity;
        count++;
    }
    if(sht3x3.humidity >= HUMIDITY_THRESHOLD) {
        temp += sht3x3.temperature;
        hum += sht3x3.humidity;
        count++;
    }
    if(count == 0) return {-1000.0, temp};

    temp /= (double)count;
    hum /= (double)count;

    return {dew_point(temp, hum), temp};
}

void autodew() {
    pair<double, double> d = {-1000.0, -1};
    for(int i = 0; i < 3; i++) {
        // Retries
        d = get_dew_point();
        if(d.first != -1000.0) break;
    }
    double dp = d.first;
    double temp = d.second;
    if(dp == -1000.0) return;

#ifdef AUTODEW_FORCE_TEMPERATURE
    temp = AUTODEW_FORCE_TEMPERATURE;
    //dp = 5.0;
#endif

    float surface_temp = temp - 0.2; // Adjust offset based on testing
    float margin = 0.8; // Prevents oscillations (adjust as needed)
    float error = (dp + margin) - surface_temp;

    // Simple proportional control (tune Kp for your system)
    float Kp = 2.15;
    float pwm_duty = max(0.0f, min(100.0f, Kp * error)) / 100.0f; // Clamp to 0-100%

    state.dew1 = pwm_duty * 65535.0f;
    state.dew2 = pwm_duty * 65535.0f;
    state.dew3 = pwm_duty * 65535.0f;

    pwm_set_gpio_level(DEW1, (uint16_t)state.dew1);
    pwm_set_gpio_level(DEW2, (uint16_t)state.dew2);
    pwm_set_gpio_level(DEW3, (uint16_t)state.dew3);
}

int refresh_gyro_data() {
    int err = bmi160_setup(&bmi160_1);
    if(err < 0) bmi160_trigger_error(&bmi160_1);
    else {
        bmi160_median(&bmi160_1);
        return 0;
    }
    //cout << err << endl;

    err = bmi160_setup(&bmi160_2);
    if(err < 0) bmi160_trigger_error(&bmi160_2);
    else {
        bmi160_median(&bmi160_2);
        return 1;
    }
    //cout << err << endl;

    err = bmi160_setup(&bmi160_3);
    if(err < 0) bmi160_trigger_error(&bmi160_3);
    else {
        bmi160_median(&bmi160_3);
        return 2;
    }
    //cout << err << endl;

    return -1;
}

uint32_t lastMeasurementTime = 0;
float angles[] = {0, 0};

bool autodew_timer_callback(__unused struct repeating_timer *t) {
    //cout << "Autodew from core " << get_core_num() << endl;

    int gyro = -1;
    for(int i = 0; i < 3; i++) {
        gyro = refresh_gyro_data();
        if(gyro != -1) break;
    }
    if(gyro != -1) {
        bmi160_calculate_absolute_angle(gyro == 0 ? &bmi160_1 : (gyro == 1 ? &bmi160_2 : &bmi160_3), angles);
    } else {
        angles[0] = 0;
        angles[1] = 0;
    }

    if(!state.autodew) return true;

    sht3x_read_data(&sht3x1);
    sht3x_read_data(&sht3x2);
    sht3x_read_data(&sht3x3);

    autodew();
    return true;
}

void calibrate() {
    state.dc1_zero = adc_read_value("DC1_CURRENT");
    state.dc2_zero = adc_read_value("DC2_CURRENT");
    state.dc3_zero = adc_read_value("DC3_CURRENT");
    state.dc4_zero = adc_read_value("DC4_CURRENT");
    state.dc5_zero = adc_read_value("DC5_CURRENT");
    state.dew1_zero = adc_read_value("DEW1_CURRENT");
    state.dew2_zero = adc_read_value("DEW2_CURRENT");
    state.dew3_zero = adc_read_value("DEW3_CURRENT");
    state.input_zero = adc_read_value("INPUT_CURRENT");
}

float adc_get(string c) {
    float adc = adc_read_value(c);
    if(c == "DEW1_CURRENT")
        adc = ((adc - (float)state.dew1_zero) * ADS1115_BIT_TO_MV * ACS712_CURRENT_MULTIPLY) / DEW1_CURRENT_RESOLUTION;
    else if(c == "DEW2_CURRENT")
        adc = ((adc - (float)state.dew2_zero) * ADS1115_BIT_TO_MV * ACS712_CURRENT_MULTIPLY) / DEW2_CURRENT_RESOLUTION;
    else if(c == "DEW3_CURRENT")
        adc = ((adc - (float)state.dew3_zero) * ADS1115_BIT_TO_MV * ACS712_CURRENT_MULTIPLY) / DEW3_CURRENT_RESOLUTION;
    else if(c == "DC1_CURRENT")
        adc = ((adc - (float)state.dc1_zero) * ADS1115_BIT_TO_MV * ACS712_CURRENT_MULTIPLY) / DC1_CURRENT_RESOLUTION;
    else if(c == "DC2_CURRENT")
        adc = ((adc - (float)state.dc2_zero) * ADS1115_BIT_TO_MV * ACS712_CURRENT_MULTIPLY) / DC2_CURRENT_RESOLUTION;
    else if(c == "DC3_CURRENT")
        adc = ((adc - (float)state.dc3_zero) * ADS1115_BIT_TO_MV * ACS712_CURRENT_MULTIPLY) / DC3_CURRENT_RESOLUTION;
    else if(c == "DC4_CURRENT")
        adc = ((adc - (float)state.dc4_zero) * ADS1115_BIT_TO_MV * ACS712_CURRENT_MULTIPLY) / DC4_CURRENT_RESOLUTION;
    else if(c == "DC5_CURRENT")
        adc = ((adc - (float)state.dc5_zero) * ADS1115_BIT_TO_MV * ACS712_CURRENT_MULTIPLY) / DC5_CURRENT_RESOLUTION;
    else if(c == "INPUT_CURRENT")
        adc = ((adc - (float)state.input_zero) * ADS1115_BIT_TO_MV * ACS712_CURRENT_MULTIPLY) / INPUT_CURRENT_RESOLUTION + INPUT_BASE_CURRENT;
    else if(c == "EXT1_ANALOG_TEMP")
        adc = temperature_calc_ntc(adc);
    else if(c == "EXT2_ANALOG_TEMP")
        adc = temperature_calc_ntc(adc);
    else if(c == "EXT3_ANALOG_TEMP")
        adc = temperature_calc_ntc(adc);
    else if(c == "DEW_POINT")
        adc = get_dew_point().first;
    else if(c == "SHT3X1_TEMP") {
        sht3x_read_data(&sht3x1);
        adc = sht3x1.temperature;
    } else if(c == "SHT3X2_TEMP") {
        sht3x_read_data(&sht3x2);
        adc = sht3x2.temperature;
    } else if(c == "SHT3X3_TEMP") {
        sht3x_read_data(&sht3x3);
        adc = sht3x3.temperature;
    } else if(c == "SHT3X1_HUM") {
        sht3x_read_data(&sht3x1);
        adc = sht3x1.humidity;
    } else if(c == "SHT3X2_HUM") {
        sht3x_read_data(&sht3x2);
        adc = sht3x2.humidity;
    } else if(c == "SHT3X3_HUM") {
        sht3x_read_data(&sht3x3);
        adc = sht3x3.humidity;
    } else if(c == "INPUT_VOLTAGE") {
        // const float conversion_factor = 3.3f / (1 << 12);
        adc = (3.3f * INPUT_VOLTAGE_RESISTOR) * (adc / (1 << 12));
    } else if(c == "GPS1_LATITUDE") {
        return lat;
    } else if(c == "GPS1_LONGITUDE") {
        return lng;
    } else if(c == "GPS1_ELEVATION") {
        return elevation;
    } else if(c == "GPS1_SATELLITE_COUNT") {
        return satelliteNum;
    } else if(c == "DEW1") {
        adc = (state.dew1 * 100.0f) / 65535.0f;
    } else if(c == "DEW2") {
        adc = (state.dew2 * 100.0f) / 65535.0f;
    } else if(c == "DEW3") {
        adc = (state.dew3 * 100.0f) / 65535.0f;
    } else if(c == "GYRO_X") {
        adc = angles[0];
    } else if(c == "GYRO_Y") {
        adc = angles[1];
    }
    adc = round(adc * 100.0) / 100.0;
    return adc;
}


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
};

Data mcd;

// CORE 1
//
// Reserved for I2C operations
void core1_entry() {
    Data* mcld = (Data*) multicore_fifo_pop_blocking();

    alarm_pool_t* autodew_timer = alarm_pool_create_with_unused_hardware_alarm(4);
    repeating_timer rpt;
    alarm_pool_add_repeating_timer_ms(autodew_timer, 1000, autodew_timer_callback, NULL, &rpt);

#ifdef GPS0_ENABLED
    gpio_init(GPS0_ENABLE);
    gpio_set_dir(GPS0_ENABLE, GPIO_OUT);
    gpio_put(GPS0_ENABLE, !state.gps_sleep);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, false);
    uart_set_irq_enables(UART_ID, true, false);
    irq_set_exclusive_handler(UART0_IRQ, gps0_callback);
    irq_set_enabled(UART0_IRQ, true);
    irq_set_priority(UART0_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
    //gpio_set_irq_enabled_with_callback(UART0_RX_PIN, GPIO_IRQ_EDGE_RISE, true, &gps0_callback);
#endif

    int ADCS_SIZE = 20;
    string adcs[ADCS_SIZE] = {
        "DEW1_CURRENT",
        "DEW2_CURRENT",
        "DEW3_CURRENT",
        "DC1_CURRENT",
        "DC2_CURRENT",
        "DC3_CURRENT",
        "DC4_CURRENT",
        "DC5_CURRENT",
        "INPUT_CURRENT",
        "EXT1_ANALOG_TEMP",
        "EXT2_ANALOG_TEMP",
        "EXT3_ANALOG_TEMP",
        "DEW_POINT",
        "SHT3X1_TEMP",
        "SHT3X2_TEMP",
        "SHT3X3_TEMP",
        "SHT3X1_HUM",
        "SHT3X2_HUM",
        "SHT3X3_HUM",
        "INPUT_VOLTAGE",
    };
    while(true) {
        for(int i = 0; i < ADCS_SIZE; i++) {
            mcld->gps1_lat = lat;
            mcld->gps1_lng = lng;
            mcld->gps1_elevation = elevation;
            mcld->gps1_satellite_count = satelliteNum;
            mcld->gyro_x = angles[0];
            mcld->gyro_y = angles[1];

#ifdef WATCHDOG_ENABLED
#ifdef WATCHDOG_DEBUG
            cout << "Watchdog updated!" << endl;
#endif
            watchdog_update();
#endif

            string c = adcs[i];
            float adc = adc_get(c);

            if(c == "DEW1_CURRENT")
                mcld->dew1_current = adc;
            if(c == "DEW2_CURRENT")
                mcld->dew2_current = adc;
            if(c == "DEW3_CURRENT")
                mcld->dew3_current = adc;
            if(c == "DC1_CURRENT")
                mcld->dc1_current = adc;
            if(c == "DC2_CURRENT")
                mcld->dc2_current = adc;
            if(c == "DC3_CURRENT")
                mcld->dc3_current = adc;
            if(c == "DC4_CURRENT")
                mcld->dc4_current = adc;
            if(c == "DC5_CURRENT")
                mcld->dc5_current = adc;
            if(c == "INPUT_CURRENT")
                mcld->input_current = adc;
            if(c == "EXT1_ANALOG_TEMP")
                mcld->ext1_analog_temp = adc;
            if(c == "EXT2_ANALOG_TEMP")
                mcld->ext2_analog_temp = adc;
            if(c == "EXT3_ANALOG_TEMP")
                mcld->ext3_analog_temp = adc;
            if(c == "DEW_POINT")
                mcld->dew_point = adc;
            if(c == "SHT3X1_TEMP")
                mcld->sht1_temp = adc;
            if(c == "SHT3X2_TEMP") mcld->sht2_temp = adc;
            if(c == "SHT3X3_TEMP") mcld->sht3_temp = adc;
            if(c == "SHT3X1_HUM") mcld->sht1_hum = adc;
            if(c == "SHT3X2_HUM") mcld->sht2_hum = adc;
            if(c == "SHT3X3_HUM") mcld->sht3_hum = adc;
            if(c == "INPUT_VOLTAGE") mcld->input_voltage = adc;
        }
        sleep_ms(50);
    }
}

// CORE 0
int main() {
    // TODO: IRQ for voltage cutoff

    stdio_init_all();
    adc_init();

#ifdef DEBUG_WAIT_FOR_USB
    while(!tud_cdc_connected()) {
        sleep_ms(100);
    }
#endif

    pwm_setup(DEW1);
    pwm_setup(DEW2);
    pwm_setup(DEW3);

    if(DEBUG_INIT_MESSAGES) {
        cout << "PWM setup completed!" << endl;
    }

    gpio_setup(DC1);
    gpio_setup(DC2);
    gpio_setup(DC3);
    gpio_setup(DC4);
    gpio_setup(DC5);

    if(DEBUG_INIT_MESSAGES) {
        cout << "GPIO setup completed!" << endl;
    }

#ifdef SAVE_ENABLED
#ifdef READ_FLASH_ON_BOOT
    read_data();
    gpio_put(DC1, state.dc1);
    gpio_put(DC2, state.dc2);
    gpio_put(DC3, state.dc3);
    gpio_put(DC4, state.dc4);
    gpio_put(DC5, state.dc5);
    gpio_put(GPS0_ENABLE, !state.gps_sleep);
    pwm_set_gpio_level(DEW1, state.dew1);
    pwm_set_gpio_level(DEW2, state.dew2);
    pwm_set_gpio_level(DEW3, state.dew3);
    mcd.state = &state;
#endif
#endif

    if(DEBUG_INIT_MESSAGES) {
        cout << "Read data completed!" << endl;
    }

#ifdef I2C0_ENABLED
    // I2C0 Initialisation. Using it at 100Khz.
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);
    if(DEBUG_INIT_MESSAGES) {
        cout << "I2C0 setup completed!" << endl;
    }
#endif
#ifdef I2C1_ENABLED
    // I2C1 Initialisation. Using it at 100Khz.
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);
    if(DEBUG_INIT_MESSAGES) {
        cout << "I2C1 setup completed!" << endl;
    }
#endif

#ifdef DEBUG_I2C
    i2c_debug(false);
    if(DEBUG_INIT_MESSAGES) {
        cout << "I2C debug completed!" << endl;
    }
#endif

#ifdef EXTERNAL_ADC_ENABLED
    // ADS1115, I2C0, GND
    ads1115_init(i2c0, ADS1115_GND_ADDR, &adc1);
    ads1115_set_pga(ADS1115_PGA_4_096, &adc1);
    ads1115_set_data_rate(ADS1115_RATE_475_SPS, &adc1);
    if(DEBUG_INIT_MESSAGES) {
        cout << "ADC1 setup completed!" << endl;
    }

    // ADS1115, I2C0, VCC
    ads1115_init(i2c0, ADS1115_VCC_ADDR, &adc2);
    ads1115_set_pga(ADS1115_PGA_4_096, &adc2);
    ads1115_set_data_rate(ADS1115_RATE_475_SPS, &adc2);
    if(DEBUG_INIT_MESSAGES) {
        cout << "ADC2 setup completed!" << endl;
    }

    // ADS1115, I2C1, GND
    ads1115_init(i2c1, ADS1115_GND_ADDR, &adc3);
    ads1115_set_pga(ADS1115_PGA_4_096, &adc3);
    ads1115_set_data_rate(ADS1115_RATE_475_SPS, &adc3);
    if(DEBUG_INIT_MESSAGES) {
        cout << "ADC3 setup completed!" << endl;
    }
#endif

#ifdef ACS712_CALIBRATE
    calibrate();
    if(DEBUG_INIT_MESSAGES) {
        cout << "ACS712 calibration completed: " << state.dc1_zero << " " << state.dc2_zero << endl;
    }
#endif

#ifdef SHT3X_ENABLED
    sht3x_init(SHT3X1_I2C, SHT3X1_ADDRESS, &sht3x1);
    sht3x_init(SHT3X2_I2C, SHT3X2_ADDRESS, &sht3x2);
    sht3x_init(SHT3X3_I2C, SHT3X3_ADDRESS, &sht3x3);
    if(DEBUG_INIT_MESSAGES) {
        cout << "SHT setup completed!" << endl;
    }
#endif

#ifdef BMI160_ENABLED
    bmi160_init(BMI160_1_I2C, BMI160_1_ADDRESS, &bmi160_1);
    bmi160_init(BMI160_2_I2C, BMI160_2_ADDRESS, &bmi160_2);
    bmi160_init(BMI160_3_I2C, BMI160_3_ADDRESS, &bmi160_3);
    if(DEBUG_INIT_MESSAGES) {
        cout << "BMI160 setup completed!" << endl;
    }
#endif

#ifdef WATCHDOG_ENABLED
    // Watchdog example code
    if(watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // Whatever action you may take if a watchdog caused a reboot
    }

    // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
    // second arg is pause on debug which means the watchdog will pause when stepping through code
    watchdog_enable(WATCHDOG_TIMER, 1);
    if(DEBUG_INIT_MESSAGES) {
        cout << "Watchdog setup completed!" << endl;
    }

    // You need to call this function at least more often than the 100ms in the enable call to prevent a reboot
    watchdog_update();
#endif

#ifdef UART_ENABLED
    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    if(DEBUG_INIT_MESSAGES) {
        cout << "UART0 setup completed!" << endl;
    }
#endif

    adc_gpio_init(28);
    adc_select_input(2);
    if(DEBUG_INIT_MESSAGES) {
        cout << "iADC setup completed!" << endl;
    }

    multicore_launch_core1(core1_entry);
    multicore_fifo_push_blocking((uint32_t)&mcd);

    while(true) {
        string c;
        getline(cin, c);
        string original = c;
        vector<string> commands = split(c, ";");
        if(commands.size() == 0) {
            cout << "INVALID_COMMAND" << endl;
            continue;
        }

        if(commands[0] == "FWINFO") {
            cout << "AstroPSU-Pico v0.0.1-beta " << __DATE__ << " " << __TIME__ << endl;
        } else if(commands[0] == "PWMSET") {
            if(commands.size() != 3) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            int num = stoi(commands[2]);
            if(num < 0 || num > 65535) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            int pin = translate_to_pin(commands[1]);
            if(pin == -1) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }
            if(pin == DEW1)
                state.dew1 = num;
            if(pin == DEW2)
                state.dew2 = num;
            if(pin == DEW3)
                state.dew3 = num;
            state.autodew = false;
            pwm_set_gpio_level(pin, num);
#ifdef SAVE_ENABLED
            save_data();
#endif
            cout << "OK" << endl;
        } else if(commands[0] == "OFF") {
            if(commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if(commands[1] != "AUTODEW" && commands[1] != "GPS1") {
                int pin = translate_to_pin(commands[1]);
                if(pin == -1) {
                    cout << "INVALID_COMMAND_ARGS" << endl;
                    continue;
                }
                if(pin == DC1)
                    state.dc1 = false;
                if(pin == DC2)
                    state.dc2 = false;
                if(pin == DC3)
                    state.dc3 = false;
                if(pin == DC4)
                    state.dc4 = false;
                if(pin == DC5)
                    state.dc5 = false;
                gpio_put(pin, false);
            } else {
                if(commands[1] == "AUTODEW")
                    state.autodew = false;
                if(commands[1] == "GPS1") {
                    state.gps_sleep = false;
                    gpio_put(GPS0_ENABLE, true);
                }
            }
#ifdef SAVE_ENABLED
            save_data();
#endif
            cout << "OK" << endl;
        } else if(commands[0] == "ON") {
            if(commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if(commands[1] != "AUTODEW" && commands[1] != "GPS1") {
                int pin = translate_to_pin(commands[1]);
                if(pin == -1) {
                    cout << "INVALID_COMMAND_ARGS" << endl;
                    continue;
                }
                if(pin == DC1)
                    state.dc1 = true;
                if(pin == DC2)
                    state.dc2 = true;
                if(pin == DC3)
                    state.dc3 = true;
                if(pin == DC4)
                    state.dc4 = true;
                if(pin == DC5)
                    state.dc5 = true;
                gpio_put(pin, true);
            } else {
                if(commands[1] == "AUTODEW")
                    state.autodew = true;
                if(commands[1] == "GPS1") {
                    gpio_put(GPS0_ENABLE, false);
                    state.gps_sleep = true;
                }
            }
#ifdef SAVE_ENABLED
            save_data();
#endif
            cout << "OK" << endl;
        } else if(commands[0] == "PWMGET") {
            if(commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if(commands[1] == "DEW1")
                cout << state.dew1 << endl;
            else if(commands[1] == "DEW2")
                cout << state.dew2 << endl;
            else if(commands[1] == "DEW3")
                cout << state.dew3 << endl;
            else
                cout << 0 << endl;
        } /*else if(commands[0] == "NAMEGET") {
            if(commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if(commands[1] == "DC1")
                cout << state.dc1_name << endl;
            else if(commands[1] == "DC2")
                cout << state.dc2_name << endl;
            else if(commands[1] == "DC3")
                cout << state.dc3_name << endl;
            else if(commands[1] == "DC4")
                cout << state.dc4_name << endl;
            else if(commands[1] == "DC5")
                cout << state.dc5_name << endl;
            else if(commands[1] == "DEW1")
                cout << state.dew1_name << endl;
            else if(commands[1] == "DEW2")
                cout << state.dew2_name << endl;
            else if(commands[1] == "DEW3")
                cout << state.dew3_name << endl;
            else
                cout << "Unknown Switch" << endl;
        } else if(commands[0] == "NAMESET") {
            if(commands.size() != 3) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            // TODO: boundary check
            string name = commands[2];

            if(commands[1] == "DC1")
                strncpy(state.dc1_name, name.c_str(), sizeof(state.dc1_name));
            else if(commands[1] == "DC2")
                strncpy(state.dc2_name, name.c_str(), sizeof(state.dc2_name));
            else if(commands[1] == "DC3")
                strncpy(state.dc3_name, name.c_str(), sizeof(state.dc3_name));
            else if(commands[1] == "DC4")
                strncpy(state.dc4_name, name.c_str(), sizeof(state.dc4_name));
            else if(commands[1] == "DC5")
                strncpy(state.dc5_name, name.c_str(), sizeof(state.dc5_name));
            else if(commands[1] == "DEW1")
                strncpy(state.dew1_name, name.c_str(), sizeof(state.dew1_name));
            else if(commands[1] == "DEW2")
                strncpy(state.dew2_name, name.c_str(), sizeof(state.dew2_name));
            else if(commands[1] == "DEW3")
                strncpy(state.dew3_name, name.c_str(), sizeof(state.dew3_name));
#ifdef SAVE_ENABLED
            save_data();
#endif
            cout << "OK" << endl;
        } */else if(commands[0] == "FLUSH") {
#ifdef SAVE_ENABLED
            save_data();
#endif
            cout << "OK" << endl;
        } else if(commands[0] == "STATEGET") {
            if(commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if(commands[1] == "DC1")
                cout << state.dc1 << endl;
            else if(commands[1] == "DC2")
                cout << state.dc2 << endl;
            else if(commands[1] == "DC3")
                cout << state.dc3 << endl;
            else if(commands[1] == "DC4")
                cout << state.dc4 << endl;
            else if(commands[1] == "DC5")
                cout << state.dc5 << endl;
            else if(commands[1] == "AUTODEW")
                cout << state.autodew << endl;
            else
                cout << false << endl;
        } else if(commands[0] == "ADCGET") {
            if(commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }
            float adc = 0.0;
            if(commands[1] == "DEW1_CURRENT")
                adc = mcd.dew1_current;
            if(commands[1] == "DEW2_CURRENT")
                adc = mcd.dew2_current;
            if(commands[1] == "DEW3_CURRENT")
                adc = mcd.dew3_current;
            if(commands[1] == "DC1_CURRENT")
                adc = mcd.dc1_current;
            if(commands[1] == "DC2_CURRENT")
                adc = mcd.dc2_current;
            if(commands[1] == "DC3_CURRENT")
                adc = mcd.dc3_current;
            if(commands[1] == "DC4_CURRENT")
                adc = mcd.dc4_current;
            if(commands[1] == "DC5_CURRENT")
                adc = mcd.dc5_current;
            if(commands[1] == "INPUT_CURRENT")
                adc = mcd.input_current;
            if(commands[1] == "EXT1_ANALOG_TEMP")
                adc = mcd.ext1_analog_temp;
            if(commands[1] == "EXT2_ANALOG_TEMP")
                adc = mcd.ext2_analog_temp;
            if(commands[1] == "EXT3_ANALOG_TEMP")
                adc = mcd.ext3_analog_temp;
            if(commands[1] == "DEW_POINT")
                adc = mcd.dew_point;
            if(commands[1] == "SHT3X1_TEMP") adc = mcd.sht1_temp;
            if(commands[1] == "SHT3X2_TEMP") adc = mcd.sht2_temp;
            if(commands[1] == "SHT3X3_TEMP") adc = mcd.sht3_temp;
            if(commands[1] == "SHT3X1_HUM") adc = mcd.sht1_hum;
            if(commands[1] == "SHT3X2_HUM") adc = mcd.sht2_hum;
            if(commands[1] == "SHT3X3_HUM") adc = mcd.sht3_hum;
            if(commands[1] == "INPUT_VOLTAGE") adc = mcd.input_voltage;
            if(commands[1] == "GPS1_LATITUDE") {
                cout << mcd.gps1_lat << endl;
                continue;
            }
            if(commands[1] == "GPS1_LONGITUDE") {
                cout << mcd.gps1_lng << endl;
                continue;
            }
            if(commands[1] == "GPS1_ELEVATION") {
                cout << mcd.gps1_elevation << endl;
                continue;
            }
            if(commands[1] == "GPS1_SATELLITE_COUNT") {
                cout << mcd.gps1_satellite_count << endl;
                continue;
            }
            if(commands[1] == "DEW1") adc = (state.dew1 * 100.0f) / 65535.0f;
            if(commands[1] == "DEW2") adc = (state.dew2 * 100.0f) / 65535.0f;
            if(commands[1] == "DEW3") adc = (state.dew3 * 100.0f) / 65535.0f;
            if(commands[1] == "GYRO_X") adc = mcd.gyro_x;
            if(commands[1] == "GYRO_Y") adc = mcd.gyro_y;
            adc = round(adc * 100.0) / 100.0;
            cout << adc << endl;
        } else if(commands[0] == "RAWADCGET") {
            if(commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }
            cout << adc_read_value(commands[1]) << endl;
        } else if(commands[0] == "RESET") {
            cout << "OK_BYE" << endl;
            watchdog_reboot(0, 0, 0);
        } else if(commands[0] == "I2CDEBUG") {
            i2c_debug(true);
            cout << "OK" << endl;
        } else if(commands[0] == "REFRESH_DATA") {
            refresh_gyro_data();
            cout << "OK" << endl;
        } else if(commands[0] == "CALIBRATE") {
            calibrate();
            cout << "OK" << endl;
        } else {
            cout << "UNDEFINED_COMMAND" << endl;
        }
    }
}
