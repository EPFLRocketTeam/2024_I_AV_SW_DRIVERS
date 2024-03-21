# New CMake build file for Icarus
cmake_minimum_required(VERSION 3.22)
project(2024_I_AV_SW_DRIVERS VERSION 0.1 LANGUAGES C CXX)

# Set the build type to Debug
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb")

# Library for Common Drivers
add_library(2024_I_AV_SW_DRIVERS_COMMON inc/common/SPI_interface.hpp inc/common/I2c_interface.hpp src/common/dummy.cpp)
target_include_directories(2024_I_AV_SW_DRIVERS_COMMON PUBLIC inc/common)

# Library for the BNO
add_library(2024_I_AV_SW_DRIVERS_BNO inc/bno/BNO.hpp src/bno/BNO.cpp)
target_link_libraries(2024_I_AV_SW_DRIVERS_BNO PUBLIC 2024_I_AV_SW_DRIVERS_COMMON)
target_include_directories(2024_I_AV_SW_DRIVERS_BNO PUBLIC inc/bno)

# Library for the BMP
add_library(2024_I_AV_SW_DRIVERS_BMP inc/bmp/BMP.hpp inc/bmp/BMP_def.hpp src/bmp/BMP.cpp)
target_link_libraries(2024_I_AV_SW_DRIVERS_BMP PUBLIC 2024_I_AV_SW_DRIVERS_COMMON)
target_include_directories(2024_I_AV_SW_DRIVERS_BMP PUBLIC inc/bmp)

# Library for the LoRa
add_library(2024_I_AV_SW_DRIVERS_LoRa inc/lora/LoRa.hpp src/lora/LoRa.cpp)
target_link_libraries(2024_I_AV_SW_DRIVERS_LoRa PUBLIC 2024_I_AV_SW_DRIVERS_COMMON)
target_include_directories(2024_I_AV_SW_DRIVERS_LoRa PUBLIC inc/lora)