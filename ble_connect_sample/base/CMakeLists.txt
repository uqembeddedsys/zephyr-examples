# SPDX-License-Identifier: Apache-2.0
#Setting to exclusively build for the dongle
set(BOARD nrf52840dongle_nrf52840)
set(CONF_FILE prj.conf usb_cdc.conf bt.conf console.conf)
set(DTC_OVERLAY_FILE dtc_console.overlay)

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(base)

#Add include_directories for libraries, path starts from this files location.
include_directories(inc/)

target_sources(app PRIVATE src/main.c
                           src/ble_base.c
                           )

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
