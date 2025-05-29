add_library(pico-bmi160 INTERFACE)

target_include_directories(pico-bmi160
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(pico-bmi160
  INTERFACE
  hardware_i2c
)

target_sources(pico-bmi160
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/bmi160.cpp
  ${CMAKE_CURRENT_LIST_DIR}/include/bmi160.h
)