# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ultrasonic_data_publisher C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

link_directories(libmicroros)
add_executable(ultrasonic_data_publisher 
    ultrasonic_data_publisher.c 
    pico_uart_transport.c
)

pico_set_program_name(ultrasonic_data_publisher "ultrasonic_data_publisher")
pico_set_program_version(ultrasonic_data_publisher "0.1")

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_usb(ultrasonic_data_publisher 1)
pico_enable_stdio_uart(ultrasonic_data_publisher 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Add the standard library to the build
target_link_libraries(ultrasonic_data_publisher
        pico_stdlib)

# Add any user requested libraries
target_link_libraries(ultrasonic_data_publisher 
    pico_stdlib
    microros
)

# Add the standard include files to the build
target_include_directories(ultrasonic_data_publisher PUBLIC
        libmicroros/include
        ${CMAKE_CURRENT_LIST_DIR}
)



pico_add_extra_outputs(ultrasonic_data_publisher)

