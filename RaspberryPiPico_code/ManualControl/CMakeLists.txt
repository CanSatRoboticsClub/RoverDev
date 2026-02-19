cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(Manual_controller C CXX ASM)

pico_sdk_init()

add_executable(Manual_controller
    Manual_controller.c
    PWMmotorDriver.c
    )

target_link_libraries(Manual_controller 
    pico_stdlib
    pico_cyw43_arch_none
    pico_btstack_ble
    pico_btstack_cyw43
    hardware_pwm
)

target_sources(Manual_controller PRIVATE
    ${PICO_SDK_PATH}/lib/btstack/src/ble/att_db_util.c
)

target_include_directories(Manual_controller PRIVATE
    ${PICO_SDK_PATH}/lib/btstack/src
    ${PICO_SDK_PATH}/lib/btstack/src/ble
)

target_include_directories(Manual_controller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

pico_enable_stdio_usb(Manual_controller 1)
pico_enable_stdio_uart(Manual_controller 0)

pico_add_extra_outputs(Manual_controller)