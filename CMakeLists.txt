# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
add_subdirectory(./pico-ads1115/lib ads1115)
add_subdirectory(./pico-sht3x/lib sht3x)
add_subdirectory(./pico-bmi160/lib bmi160)
add_subdirectory(./pico-oldi2c/lib oldi2c)

project(AstroPSU-Pico C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(AstroPSU-Pico AstroPSU-Pico.cpp )

pico_set_program_name(AstroPSU-Pico "AstroPSU-Pico")
pico_set_program_version(AstroPSU-Pico "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(AstroPSU-Pico 0)
pico_enable_stdio_usb(AstroPSU-Pico 1)

# Add the standard library to the build
target_link_libraries(AstroPSU-Pico
        pico_stdlib
        pico_multicore)

# Add the standard include files to the build
target_include_directories(AstroPSU-Pico PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(AstroPSU-Pico 
        hardware_spi
        hardware_i2c
        hardware_watchdog
        hardware_pwm
        hardware_flash
        hardware_adc
        pico-ads1115
        pico-sht3x
        pico-bmi160
        pico-oldi2c
)

pico_add_extra_outputs(AstroPSU-Pico)

set_property(TARGET AstroPSU-Pico APPEND_STRING PROPERTY LINK_FLAGS "-Wl,--print-memory-usage")

