# Указание компилятора
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

# Убираем добавление ".exe"
set(CMAKE_EXECUTABLE_SUFFIX "" CACHE INTERNAL "")

# Отключение проверки компилятора
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Флаги компилятора
set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -Wall -Og -g -std=gnu11" CACHE INTERNAL "")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_FULL_LL_DRIVER -DSTM32F407xx -DHSE_VALUE=25000000" CACHE INTERNAL "")

# Флаги линкера
set(CMAKE_EXE_LINKER_FLAGS "-T${CMAKE_SOURCE_DIR}/STM32F407VGTX_FLASH.ld -mcpu=cortex-m4 -mthumb -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard" CACHE INTERNAL "")