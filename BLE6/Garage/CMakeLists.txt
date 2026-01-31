cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Garage)

target_sources(app PRIVATE src/main.cpp ../src/blinker.cpp src/ble_initiator.cpp ../src/PairingConfig.cpp)
