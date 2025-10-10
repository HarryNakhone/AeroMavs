cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(api)

set(BOARD_FLASH_RUNNER stm32cubeprogrammer)

# Add custom driver as a module

target_sources(app PRIVATE applications/code.c)
