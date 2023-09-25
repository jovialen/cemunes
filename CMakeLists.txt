cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)
project(cnes LANGUAGES C VERSION 0.1.0)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "-Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_RELEASE "-O3")

option(CNES_TRACE_ADDR_MODE "Include the address mode of the instruction in the trace" ON)

add_executable(cnes "src/main.c" "src/bus.c" "src/cpu.c" "src/opcodes.c" "src/cartridge.c")

if (${CNES_TRACE_ADDR_MODE})
  target_compile_definitions(cnes PRIVATE "CNES_TRACE_ADDR_MODE")
endif()

set_target_properties(cnes PROPERTIES
  C_STANDARD 99
  C_STANDARD_REQUIRED ON
  C_EXTENSIONS OFF)

target_include_directories(cnes PRIVATE "src")
