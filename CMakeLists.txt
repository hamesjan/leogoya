# ESP-IDF project (v5.x). Build/flash the XIAO ESP32S3 Sense with:
#   idf.py set-target esp32s3
#   idf.py build flash monitor
# Sources live in src/; main/CMakeLists.txt pulls them in.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(leogoya)
