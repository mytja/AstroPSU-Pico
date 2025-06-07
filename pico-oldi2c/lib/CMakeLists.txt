add_library(pico-oldi2c INTERFACE)

target_include_directories(pico-oldi2c
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(pico-oldi2c
  INTERFACE
  hardware_i2c
)

target_sources(pico-oldi2c
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/oldi2c.cpp
  ${CMAKE_CURRENT_LIST_DIR}/include/oldi2c.h
)