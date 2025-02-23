set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.25)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(robot C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

# For some reason, CM4 also uses -mcpu=cortex-m7
add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include
    Drivers/CMSIS/DSP/DSP_Lib_TestSuite/RefLibs/inc Libraries/fw-shared-modules/Inc
    Libraries/nRF24L01P-lib Libraries/proto-messages/proto_c Libraries/protobuf
    Drivers/CMSIS/DSP/Include Libraries/hlindgren_lsm6dsl/include)

add_compile_definitions(DEBUG USE_HAL_DRIVER STM32H755xx)

set(DRIVERS_ROOT Drivers/STM32H7xx_HAL_Driver/Src)
set(DSP_ROOT Drivers/CMSIS/DSP/Source)
set(DRIVERS_SOURCE ${DRIVERS_ROOT}/stm32h7xx_hal_cortex.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_tim.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_tim_ex.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_rcc.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_rcc_ex.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_flash.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_flash_ex.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_gpio.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_hsem.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_dma.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_dma_ex.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_mdma.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_pwr.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_pwr_ex.c
       ${DRIVERS_ROOT}/stm32h7xx_hal.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_i2c.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_i2c_ex.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_exti.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_uart.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_uart_ex.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_spi.c
       ${DRIVERS_ROOT}/stm32h7xx_hal_spi_ex.c
       ${DSP_ROOT}/ControllerFunctions/arm_sin_cos_f32.c
       ${DSP_ROOT}/CommonTables/arm_common_tables.c 
       ${DSP_ROOT}/MatrixFunctions/MatrixFunctions.c
       ${DSP_ROOT}/FilteringFunctions/FilteringFunctions.c
       ${DSP_ROOT}/FastMathFunctions/FastMathFunctions.c
)

file(GLOB_RECURSE COMMON_SOURCES "Common/*.*" "Libraries/proto-messages/proto_c/*.*" "Libraries/protobuf/*.*")
file(GLOB NRF_SOURCE "Libraries/nRF24L01P-lib/*.*")

file(GLOB LSM6DSL_SOURCE "Libraries/hlindgren_lsm6dsl/src/HLindgren_LSM6DSL.c")
add_compile_definitions(LSM6DSL_USE_I2C_MEM_READ_AND_WRITE)

file(GLOB_RECURSE CM7_SOURCES "Libraries/fw-shared-modules/Src/*.*")
list(APPEND CM7_SOURCES 
        CM7/Core/Src/main.c
        CM7/Core/Src/nav.c
        CM7/Core/Src/kicker.c
        CM7/Core/Src/motor.c
        CM7/Core/Src/com.c
        CM7/Core/Src/stm32h7xx_hal_msp.c
        CM7/Core/Src/stm32h7xx_it.c
        CM7/Core/Src/ui.c
        CM7/Core/Src/imu.c
        CM7/Core/Src/state_estimator2.c
        CM7/Core/Src/lag_element.c
        CM7/Core/Src/arm_mat_util_f32.c
)


file(GLOB_RECURSE CM4_SOURCES "CM4/Core/*.*")

set(CM7_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Buildfiles/CM7/stm32h755xx_flash_CM7.ld)
set(CM4_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Buildfiles/CM4/stm32h755xx_flash_CM4.ld)

file(GLOB CM7_ASM_SOURCE "Buildfiles/CM7/*.s")
file(GLOB CM4_ASM_SOURCE "Buildfiles/CM4/*.s")

add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork -specs=nano.specs -u _printf_float)

add_executable(robot_CM7.elf ${CM7_SOURCES} ${COMMON_SOURCES} ${DRIVERS_SOURCE} ${NRF_SOURCE}
    ${LSM6DSL_SOURCE} ${CM7_LINKER_SCRIPT} ${CM7_ASM_SOURCE})
add_executable(robot_CM4.elf ${CM4_SOURCES} ${COMMON_SOURCES} ${DRIVERS_SOURCE} ${CM4_LINKER_SCRIPT} ${CM4_ASM_SOURCE})

target_include_directories(robot_CM7.elf PUBLIC CM7/Core/Inc)
target_include_directories(robot_CM4.elf PUBLIC CM4/Core/Inc)

target_compile_definitions(robot_CM7.elf PUBLIC CORE_CM7)
target_compile_definitions(robot_CM4.elf PUBLIC CORE_CM4)

target_link_options(robot_CM7.elf PUBLIC -Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/robot_CM7.map)
target_link_options(robot_CM7.elf PUBLIC -T ${CM7_LINKER_SCRIPT})
target_link_options(robot_CM4.elf PUBLIC -Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/robot_CM4.map)
target_link_options(robot_CM4.elf PUBLIC -T ${CM4_LINKER_SCRIPT})

target_link_libraries(robot_CM7.elf c m nosys)
target_link_libraries(robot_CM4.elf c m nosys)

set(CM7_HEX_FILE ${PROJECT_BINARY_DIR}/robot_CM7.hex)
set(CM7_BIN_FILE ${PROJECT_BINARY_DIR}/robot_CM7.bin)
set(CM4_HEX_FILE ${PROJECT_BINARY_DIR}/robot_CM4.hex)
set(CM4_BIN_FILE ${PROJECT_BINARY_DIR}/robot_CM4.bin)

add_custom_command(TARGET robot_CM7.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:robot_CM7.elf> ${CM7_HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:robot_CM7.elf> ${CM7_BIN_FILE}
    COMMENT "Building ${CM7_HEX_FILE}
    Building ${CM7_BIN_FILE}")

add_custom_command(TARGET robot_CM4.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:robot_CM4.elf> ${CM4_HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:robot_CM4.elf> ${CM4_BIN_FILE}
    COMMENT "Building ${CM4_HEX_FILE}
    Building ${CM4_BIN_FILE}")


add_custom_target(flash_cm7 DEPENDS ${CM7_BIN_FILE} robot_CM7.elf COMMAND STM32_Programmer_CLI -c port=SWD -w ${CM7_BIN_FILE} 0x08000000 -rst)
add_custom_target(flash_cm7_st DEPENDS ${CM7_BIN_FILE} robot_CM7.elf COMMAND st-flash --reset write robot_CM7.bin 0x08000000)
