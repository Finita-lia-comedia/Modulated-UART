cmake_minimum_required(VERSION 3.15)

# Название проекта
project(MySTM32Project LANGUAGES C ASM)

# Отключаем добавление ".exe" в имени выходного файла
set(CMAKE_EXECUTABLE_SUFFIX "")

# Указание компилятора
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

# Флаги компилятора
set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -Wall -Og -g -std=gnu11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_FULL_LL_DRIVER -DSTM32F407xx -DHSE_VALUE=25000000")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")


# Флаги линкера
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407VGTX_FLASH.ld)
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -mcpu=cortex-m4 -mthumb -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# Явное указание, что проект предназначен для встраиваемой системы
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Отключение автоматического добавления библиотек для хост-системы
set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET>")

# Пути к заголовочным файлам
include_directories(
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
)

# Исходные файлы
file(GLOB_RECURSE SOURCES
    Core/Src/*.c
    Core/Startup/*.s
    Drivers/STM32F4xx_HAL_Driver/Src/*.c
    Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/*.c
)

# Исполняемый файл
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Генерация .hex файла
add_custom_command(
    TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Generating HEX file"
)

# Генерация .bin файла
add_custom_command(
    TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Generating BIN file"
)

# Вывод размера файла
add_custom_command(
    TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Size of the executable"
)