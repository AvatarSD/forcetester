cmake_minimum_required(VERSION 3.3)

###############################################################################
### toolchain
set(CMAKE_SYSTEM_NAME Generic)

#set(CROSS_NAME arm-none-eabi)
#set(TOOLCHAIN_PATH /opt/gcc-arm-none-eabi-5_4-2016q3)
set(CROSS_NAME arm-atollic-eabi)
set(TOOLCHAIN_PATH /opt/Atollic_TrueSTUDIO_for_ARM_7.1.0/ARMTools)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/${CROSS_NAME}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/${CROSS_NAME}-g++)
#set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PATH}/bin/${CROSS_NAME}-as)
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH}/${CROSS_NAME})
#set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

###############################################################################
### options
# target flag
set(COMMON_FLAGS "-mthumb -mcpu=cortex-m3") #-mfloat-abi=hard -mfpu=fpv4-sp-d16" )
add_definitions(-DSTM32F103xB)

# required flags
add_definitions(-DUSE_HAL_DRIVER -D_IO=volatile ${COMMON_FLAGS} )

# compile flags
add_definitions(-ffunction-sections -fdata-sections -fstack-usage -Wall -O2)
set(C_COVERAGE_FLAGS "-std=gnu11")
set(CXX_COVERAGE_FLAGS "-std=c++14 -fno-threadsafe-statics")
set(LINK_COVERAGE_FLAGS "${COMMON_FLAGS} -Wl,--gc-sections -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group -specs=nano.specs -specs=nosys.specs -static -Wl,-u,Reset_Handler")

# routine
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_COVERAGE_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_COVERAGE_FLAGS}")

# debug
#set(CMAKE_VERBOSE_MAKEFILE ON)

###############################################################################
### files
## project files
project(forcetester C CXX ASM)
include_directories(${CMAKE_SOURCE_DIR}/Inc)
file(GLOB EXE_FILES ${CMAKE_SOURCE_DIR}/Src/* )

## middlewares
set(MIDDLEWARES_PATH ${CMAKE_SOURCE_DIR}/Middlewares)
include_directories(
	${MIDDLEWARES_PATH}/ST/STM32_USB_Device_Library/Core/Inc
	${MIDDLEWARES_PATH}/ST/STM32_USB_Device_Library/Class/CDC/Inc)
file(GLOB MIDDLEWARES_EXE_FILES
        ${MIDDLEWARES_PATH}/ST/STM32_USB_Device_Library/Core/Src/*
        ${MIDDLEWARES_PATH}/ST/STM32_USB_Device_Library/Class/CDC/Src/*)

## freeRTOS
set(FREERTOS_PATH ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS)
include_directories(
        ${FREERTOS_PATH}/Source/CMSIS_RTOS
        ${FREERTOS_PATH}/Source/include
        ${FREERTOS_PATH}/Source/portable/GCC/ARM_CM3)
file(GLOB FREERTOS_EXE_FILES
        ${FREERTOS_PATH}/Source/*
        ${FREERTOS_PATH}/Source/CMSIS_RTOS/*
        ${FREERTOS_PATH}/Source/portable/GCC/ARM_CM3/*
        ${FREERTOS_PATH}/Source/portable/MemMang/*)

## drivers files
set(DRIVERS_PATH ${CMAKE_SOURCE_DIR}/Drivers)
include_directories(
	    ${DRIVERS_PATH}/STM32F1xx_HAL_Driver/Inc
		${DRIVERS_PATH}/STM32F1xx_HAL_Driver/Inc/Legacy
		${DRIVERS_PATH}/CMSIS/Device/ST/STM32F1xx/Include
		${DRIVERS_PATH}/CMSIS/Include )
file(GLOB DRIVERS_EXE_FILES
	    ${DRIVERS_PATH}/STM32F1xx_HAL_Driver/Src/*
		${CMAKE_SOURCE_DIR}/startup/startup_stm32f103xb.s)

## linker script file
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103C8_FLASH.ld)

###############################################################################
### target
## executable
add_executable(${PROJECT_NAME} ${EXE_FILES} ${DRIVERS_EXE_FILES} ${MIDDLEWARES_EXE_FILES} ${FREERTOS_EXE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-T${LINKER_SCRIPT} ${LINK_COVERAGE_FLAGS}")
## objcopy
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
	${TOOLCHAIN_PATH}/bin/${CROSS_NAME}-objcopy -O ihex ${PROJECT_NAME} ${PROJECT_NAME}.hex)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
	${TOOLCHAIN_PATH}/bin/${CROSS_NAME}-size ${PROJECT_NAME})
## program
#add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND
#openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "init" -c "reset init" -c "flash write_image erase %{CurrentProject:Name}.hex" -c "reset" -c "shutdown" )


