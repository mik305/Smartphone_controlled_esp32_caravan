cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_actuator_control)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/web)

