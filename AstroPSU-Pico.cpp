#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/flash.h"

#include "AstroPSU-Pico.h"
#include "ads1115.h"

#include <string>
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>

using namespace std;

State state;

void saveMyData() {
    uint8_t* myDataAsBytes = (uint8_t*) &state;
    int myDataSize = sizeof(state);
    
    int writeSize = (myDataSize / FLASH_PAGE_SIZE) + 1; // how many flash pages we're gonna need to write
    int sectorCount = ((writeSize * FLASH_PAGE_SIZE) / FLASH_SECTOR_SIZE) + 1; // how many flash sectors we're gonna need to erase    
    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE * sectorCount);
    flash_range_program(FLASH_TARGET_OFFSET, myDataAsBytes, FLASH_PAGE_SIZE * writeSize);
    restore_interrupts(interrupts);
   
}

void readBackMyData() {
    const uint8_t* flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(&state, flash_target_contents, sizeof(state));
}


vector<string> split(string& s, const string& delimiter) {
    vector<string> tokens;
    size_t pos = 0;
    string token;
    while ((pos = s.find(delimiter)) != string::npos) {
        token = s.substr(0, pos);
        tokens.push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    tokens.push_back(s);
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


int translate_to_pin(const string& pin) {
    if (pin == "DEW1") return DEW1;
    if (pin == "DEW2") return DEW2;
    if (pin == "DEW3") return DEW3;
    if (pin == "DC1") return DC1;
    if (pin == "DC2") return DC2;
    if (pin == "DC3") return DC3;
    if (pin == "DC4") return DC4;
    if (pin == "DC5") return DC5;
    return -1;
}


struct ads1115_adc adc1;
struct ads1115_adc adc2;
struct ads1115_adc adc3;


float temperature_calc_ntc(uint16_t adc) {
    // https://arduinodiy.wordpress.com/2015/11/10/measuring-temperature-with-ntc-the-steinhart-hart-formula/
    float ntc_resistance = NTC_RESISTOR / ((65535.0F / (float)adc) - 1);
    // https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
    float temperature = pow(NTC_A + NTC_B * log(ntc_resistance) + NTC_C * pow(log(ntc_resistance), 3), -1);
    // To convert from Kelvins to degrees Celcius
    return temperature - 273.15;
}


uint16_t adc_read(ads1115_adc& adc, int pin) {
    auto mux = ADS1115_MUX_SINGLE_0;
    if (pin == 1) mux = ADS1115_MUX_SINGLE_1;
    if (pin == 2) mux = ADS1115_MUX_SINGLE_2;
    if (pin == 3) mux = ADS1115_MUX_SINGLE_3; 
    ads1115_set_input_mux(mux, &adc);
    ads1115_write_config(&adc);

    uint16_t adc_value;
    ads1115_read_adc(&adc_value, &adc);
    return adc_value;
}

uint16_t adc_read_value(const string& device) {
    if (device == "DEW3") return adc_read(adc1, 0);
    if (device == "DEW2") return adc_read(adc1, 1);
    if (device == "DEW1") return adc_read(adc1, 2);
    if (device == "DC1") return adc_read(adc1, 3);
    if (device == "DC2") return adc_read(adc2, 0);
    if (device == "DC3") return adc_read(adc2, 1);
    if (device == "DC4") return adc_read(adc2, 2);
    if (device == "DC5") return adc_read(adc2, 3);
    if (device == "INPUT") return adc_read(adc3, 0);
    if (device == "EXT1") return adc_read(adc3, 1);
    if (device == "EXT2") return adc_read(adc3, 2);
    if (device == "EXT3") return adc_read(adc3, 3);
    return 0;
}


int main() {
    // TODO: Timer for auto-dew functionality
    // https://github.com/raspberrypi/pico-examples/blob/master/timer/hello_timer/hello_timer.c
    
    stdio_init_all();
    adc_init();

#ifdef SAVE_ENABLED
    readBackMyData();
#endif

    pwm_setup(DEW1);
    pwm_setup(DEW2);
    pwm_setup(DEW3);

    gpio_setup(DC1);
    gpio_setup(DC2);
    gpio_setup(DC3);
    gpio_setup(DC4);
    gpio_setup(DC5);
    
#ifdef I2C0_ENABLED
    // I2C0 Initialisation. Using it at 400Khz.
    i2c_init(i2c0, 400*1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);
#endif
#ifdef I2C1_ENABLED
    // I2C1 Initialisation. Using it at 400Khz.
    i2c_init(i2c1, 400*1000);
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);
#endif

#ifdef EXTERNAL_ADC_ENABLED
    // ADS1115, I2C0, GND
    ads1115_init(i2c0, ADS1115_GND_ADDR, &adc1);
    ads1115_set_pga(ADS1115_PGA_4_096, &adc1);
    ads1115_set_data_rate(ADS1115_RATE_475_SPS, &adc1);

    // ADS1115, I2C0, VCC
    ads1115_init(i2c0, ADS1115_VCC_ADDR, &adc2);
    ads1115_set_pga(ADS1115_PGA_4_096, &adc2);
    ads1115_set_data_rate(ADS1115_RATE_475_SPS, &adc2);

    // ADS1115, I2C1, GND
    ads1115_init(i2c1, ADS1115_GND_ADDR, &adc3);
    ads1115_set_pga(ADS1115_PGA_4_096, &adc3);
    ads1115_set_data_rate(ADS1115_RATE_475_SPS, &adc3);
#endif

#ifdef WATCHDOG_ENABLED
    // Watchdog example code
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // Whatever action you may take if a watchdog caused a reboot
    }
    
    // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
    // second arg is pause on debug which means the watchdog will pause when stepping through code
    watchdog_enable(100, 1);
    
    // You need to call this function at least more often than the 100ms in the enable call to prevent a reboot
    watchdog_update();
#endif

#ifdef UART0_ENABLED
    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART
    
    // Send out a string, with CR/LF conversions
    uart_puts(UART_ID, " Hello, UART!\n");
#endif

    adc_gpio_init(28);
    adc_select_input(2);

    while (true) {
        string c;
        cin >> c;
        string original = c;
        vector<string> commands = split(c, ";");
        if (commands.size() == 0) {
            cout << "INVALID_COMMAND" << endl;
            continue;
        }
        
        if (commands[0] == "FWINFO") {
            cout << "AstroPSU-Pico v0.0.1-beta " << __DATE__ << " " << __TIME__ << endl;
        } else if (commands[0] == "PWMSET") {
            if (commands.size() != 3) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            int num = stoi(commands[2]);
            if (num < 0 || num > 65535) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            int pin = translate_to_pin(commands[1]);
            if (pin == -1) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }
            if (pin == DEW1) state.dew1 = num;
            if (pin == DEW2) state.dew2 = num;
            if (pin == DEW3) state.dew3 = num;
            pwm_set_gpio_level(pin, num);
            cout << "OK" << endl;
        } else if (commands[0] == "OFF") {
            if (commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if (commands[1] != "AUTODEW" && commands[1] != "GPS1") {
                int pin = translate_to_pin(commands[1]);
                if (pin == -1) {
                    cout << "INVALID_COMMAND_ARGS" << endl;
                    continue;
                }
                if (pin == DC1) state.dc1 = false;
                if (pin == DC2) state.dc2 = false;
                if (pin == DC3) state.dc3 = false;
                if (pin == DC4) state.dc4 = false;
                if (pin == DC5) state.dc5 = false;
                gpio_put(pin, false);
            } else {
                if (commands[1] == "AUTODEW") state.autodew = false;
                if (commands[1] == "GPS1") state.gps_sleep = false;
            }
            cout << "OK" << endl;
        } else if (commands[0] == "ON") {
            if (commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if (commands[1] != "AUTODEW" && commands[1] != "GPS1") {
                int pin = translate_to_pin(commands[1]);
                if (pin == -1) {
                    cout << "INVALID_COMMAND_ARGS" << endl;
                    continue;
                }
                if (pin == DC1) state.dc1 = true;
                if (pin == DC2) state.dc2 = true;
                if (pin == DC3) state.dc3 = true;
                if (pin == DC4) state.dc4 = true;
                if (pin == DC5) state.dc5 = true;
                gpio_put(pin, true);
            } else {
                if (commands[1] == "AUTODEW") state.autodew = true;
                if (commands[1] == "GPS1") state.gps_sleep = true;
            }
            cout << "OK" << endl;
        } else if (commands[0] == "PWMGET") {
            if (commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if (commands[1] == "DEW1") cout << state.dew1 << endl;
            else if (commands[1] == "DEW2") cout << state.dew2 << endl;
            else if (commands[1] == "DEW3") cout << state.dew3 << endl;
            else cout << 0 << endl;
        } else if (commands[0] == "NAMEGET") {
            if (commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if (commands[1] == "DC1") cout << state.dc1_name << endl;
            else if (commands[1] == "DC2") cout << state.dc2_name << endl;
            else if (commands[1] == "DC3") cout << state.dc3_name << endl;
            else if (commands[1] == "DC4") cout << state.dc4_name << endl;
            else if (commands[1] == "DC5") cout << state.dc5_name << endl;
            else if (commands[1] == "DEW1") cout << state.dew1_name << endl;
            else if (commands[1] == "DEW2") cout << state.dew2_name << endl;
            else if (commands[1] == "DEW3") cout << state.dew3_name << endl;
            else cout << "Unknown Switch" << endl;
        } else if (commands[0] == "STATEGET") {
            if (commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }

            if (commands[1] == "DC1") cout << state.dc1 << endl;
            else if (commands[1] == "DC2") cout << state.dc2 << endl;
            else if (commands[1] == "DC3") cout << state.dc3 << endl;
            else if (commands[1] == "DC4") cout << state.dc4 << endl;
            else if (commands[1] == "DC5") cout << state.dc5 << endl;
            else cout << false << endl;
        } else if (commands[0] == "ADCGET") {
            if (commands.size() != 2) {
                cout << "INVALID_COMMAND_ARGS" << endl;
                continue;
            }
            float adc = 0;
            if (commands[1] == "DEW1_CURRENT") adc = adc_read(adc1, 2) * DEW1_CURRENT_RESOLUTION;
            if (commands[1] == "DEW2_CURRENT") adc = adc_read(adc1, 1) * DEW2_CURRENT_RESOLUTION;
            if (commands[1] == "DEW3_CURRENT") adc = adc_read(adc1, 0) * DEW3_CURRENT_RESOLUTION;
            if (commands[1] == "DC1_CURRENT") adc = adc_read(adc1, 3) * DC1_CURRENT_RESOLUTION;
            if (commands[1] == "DC2_CURRENT") adc = adc_read(adc2, 0) * DC2_CURRENT_RESOLUTION;
            if (commands[1] == "DC3_CURRENT") adc = adc_read(adc2, 1) * DC3_CURRENT_RESOLUTION;
            if (commands[1] == "DC4_CURRENT") adc = adc_read(adc2, 2) * DC4_CURRENT_RESOLUTION;
            if (commands[1] == "DC5_CURRENT") adc = adc_read(adc2, 3) * DC5_CURRENT_RESOLUTION;
            if (commands[1] == "INPUT_CURRENT") adc = adc_read(adc3, 0) * INPUT_CURRENT_RESOLUTION;
            if (commands[1] == "EXT1_ANALOG_TEMP") adc = temperature_calc_ntc(adc_read(adc3, 1));
            if (commands[1] == "EXT2_ANALOG_TEMP") adc = temperature_calc_ntc(adc_read(adc3, 2));
            if (commands[1] == "EXT3_ANALOG_TEMP") adc = temperature_calc_ntc(adc_read(adc3, 3));
            if (commands[1] == "INPUT_VOLTAGE") {
                const float conversion_factor = 3.3f / (1 << 12);
                adc_select_input(2);
                adc = (float)adc_read() * conversion_factor / INPUT_VOLTAGE_RESISTOR;
            }
            adc = round(adc * 100.0) / 100.0;
            cout << adc << endl;
        } else {
            cout << "UNDEFINED_COMMAND" << endl;
        }
    }
}
