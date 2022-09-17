# Copyright (c) 2022 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET MasterR2)

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH} ${CMAKE_CURRENT_BINARY_DIR}/mbed-os)

add_executable(${APP_TARGET}
    CMPS12_KRAI/CMPS12_KRAI.cpp
    pidLo/pidLo.cpp
    SMC_KRAI/SMC_KRAI.cpp
    ControlMotor/ControlMotor.cpp
    Motor/Motor.cpp
    millis/millis.cpp
    encoderHAL/encoderHAL.cpp
    encoderKRAI/encoderKRAI.cpp
    odometriKRAI/odometriKRAI.cpp
    JoystickPS3/JoystickPS3.cpp
    StanleyPursuit/StanleyPursuit.cpp
    ControlKRAI/ControlKRAI.cpp
    main.cpp
)

target_link_libraries(${APP_TARGET} mbed-os)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()