# CMakeLists.txt --- CMake file for 16c
# Copyright (c) Joe Jevnik 2014

set(16C_FILES processor.c
              ../16machine/machine/memory.c
              ../16machine/machine/operations.c)

set(EXECUTABLE_OUTPUT_PATH ${16CANDLES_BINARY_DIR})

add_executable(16c ${16C_FILES})
