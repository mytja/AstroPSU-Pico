add_library(pico-sht3x INTERFACE)

target_include_directories(pico-sht3x
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(pico-sht3x
  INTERFACE
  hardware_i2c
)

target_sources(pico-sht3x
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/sht3x.cpp
  ${CMAKE_CURRENT_LIST_DIR}/include/sht3x.h
)