# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
set(PICO_SDK_PATH "/home/chris/pico/pico-sdk")
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(mpu6050 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include_directories(
   ${CMAKE_SOURCE_DIR}/include
)

set(SRC
#   ${CMAKE_SOURCE_DIR}/src/main.c
)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()


add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.c ${SRC})
target_link_libraries(${PROJECT_NAME} pico_stdlib cmsis_core hardware_i2c)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Enable usb output, disable uart output
#pico_enable_stdio_usb(${PROJECT_NAME} 1)
#pico_enable_stdio_uart(${PROJECT_NAME} 0)