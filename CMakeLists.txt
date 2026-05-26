cmake_minimum_required(VERSION 3.16)
project(fpvd CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# Vendored header-only deps.
add_library(third_party INTERFACE)
target_include_directories(third_party INTERFACE
    ${CMAKE_SOURCE_DIR}/third_party/cpp-httplib
    ${CMAKE_SOURCE_DIR}/third_party/nlohmann_json
    ${CMAKE_SOURCE_DIR}/third_party/doctest
)

find_package(Threads REQUIRED)

# Core library (everything except main.cpp) — so tests can link it.
add_library(fpvd_core STATIC src/placeholder.cpp)
target_include_directories(fpvd_core PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fpvd_core PUBLIC third_party Threads::Threads)
target_sources(fpvd_core PRIVATE
    src/config/store.cpp
    src/config/validate.cpp
)

# Executable.
add_executable(fpvd src/main.cpp)
target_link_libraries(fpvd PRIVATE fpvd_core)

# Tests.
enable_testing()
add_executable(fpvd_tests tests/test_main.cpp)
target_link_libraries(fpvd_tests PRIVATE fpvd_core)
target_sources(fpvd_tests PRIVATE
    tests/unit/test_sanity.cpp
    tests/unit/test_schema.cpp
    tests/unit/test_store.cpp
    tests/unit/test_validate.cpp
)
add_test(NAME fpvd_tests COMMAND fpvd_tests)

# Install rules.
install(TARGETS fpvd RUNTIME DESTINATION bin)
install(PROGRAMS scripts/radio-up.sh DESTINATION libexec/fpvd)
install(PROGRAMS scripts/S99fpvd DESTINATION ../etc/init.d)
install(FILES etc/defaults.json DESTINATION ../rom/etc/fpvd)
