cmake_minimum_required(VERSION 3.10)

project(pi5ws2812b)

set(CMAKE_C_STANDARD 11)
set(SOURCE_DIR "spi")

add_executable(${PROJECT_NAME} 
    ${SOURCE_DIR}/rpi5Rp1Spi.c 
    ${SOURCE_DIR}/rp1Spi.c
    ${SOURCE_DIR}/rp1SpiUtil.c)

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)