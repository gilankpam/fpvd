# Cross toolchain for OpenIPC SSC338Q (armv7l, musl, NEON-VFPv4, static).
# Mirrors wfbng-dynamic-link/drone/Makefile's `ssc338q` target.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv7l)
set(CMAKE_C_COMPILER   armv7l-unknown-linux-musleabihf-gcc)
set(CMAKE_CXX_COMPILER armv7l-unknown-linux-musleabihf-g++)
set(_ssc "-march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -Os")
set(CMAKE_C_FLAGS_INIT   "${_ssc}")
set(CMAKE_CXX_FLAGS_INIT "${_ssc}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-static -static-libstdc++ -static-libgcc")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
