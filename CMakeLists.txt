cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0016 NEW)

project(UART-GPIO)

set(CMAKE_C_STANDARD 99)

add_executable(
  UART-GPIO
        src/main.c
        src/gpio.c
        src/tims.c
        src/SendByte.c
)

target_include_directories(UART-GPIO ${CMAKE_SOURCE_DIR}/inc)