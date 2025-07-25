# Toolchain file for cross-compiling to Linux athena (FRC RoboRIO)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR athena)

# Search for compiler executables ending with -gcc and -g++ in the athena toolchain bin directory
file(GLOB ATHENA_GCC_PATH "$ENV{TOOLCHAIN_ROOT}/bin/*-gcc")
file(GLOB ATHENA_GPP_PATH "$ENV{TOOLCHAIN_ROOT}/bin/*-g++")
list(GET ATHENA_GCC_PATH 0 ATHENA_GCC)
list(GET ATHENA_GPP_PATH 0 ATHENA_GPP)
set(CMAKE_C_COMPILER   "${ATHENA_GCC}")
set(CMAKE_CXX_COMPILER "${ATHENA_GPP}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
