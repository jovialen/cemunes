cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)
project(cnes LANGUAGES C VERSION 0.1.0)

add_executable(cnes "src/main.c" "src/bus.c" "src/cpu.c")

target_include_directories(cnes PRIVATE "src")
