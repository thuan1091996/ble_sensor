# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_sensor)

target_sources(app PRIVATE 
                    src/main.c
                    src/imu_driver.c
                    # BLE drivers
                    src/ble/bluetoothle.c
                    src/ble/ble_gatt.c
)

zephyr_library_include_directories(src/ble)