cmake_minimum_required(VERSION 3.16)
project(fpvd CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# PIC for host builds; disabled for the fully-static cross target.
if(NOT CMAKE_CROSSCOMPILING)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

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
    src/dynlink/dedup.cpp
    src/dynlink/encoder_client.cpp
    src/dynlink/radio_txpower.cpp
    src/dynlink/roi_qp.cpp
    src/dynlink/watchdog.cpp
    src/dynlink/wire.cpp
    src/dynlink/hello.cpp
    src/dynlink/idr_listen.cpp
    src/dynlink/runtime_config.cpp
    src/dynlink/osd.cpp
    src/dynlink/controller.cpp
    src/config/store.cpp
    src/config/validate.cpp
    src/config/diff.cpp
    src/config/lock.cpp
    src/translate/waybeam.cpp
    src/waybeam/client.cpp
    src/translate/wfb.cpp
    src/translate/wfb_control.cpp
    src/translate/telemetry.cpp

    src/supervise/process.cpp
    src/supervise/supervisor.cpp
    src/supervise/orchestrator.cpp
    src/supervise/radio.cpp
    src/supervise/beamforming.cpp
    src/http/server.cpp
    src/http/handlers.cpp
    src/status.cpp
    src/daemon.cpp
)

# Executable.
add_executable(fpvd src/main.cpp)
target_link_libraries(fpvd PRIVATE fpvd_core)

# Tests build host-only (doctest runs on the dev machine / qemu-arm).
if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_executable(fpvd_tests tests/test_main.cpp)
    target_link_libraries(fpvd_tests PRIVATE fpvd_core)
    target_sources(fpvd_tests PRIVATE
        tests/unit/test_dl_dedup.cpp
        tests/unit/test_dl_watchdog.cpp
        tests/unit/test_dl_wire.cpp
        tests/unit/test_link_width.cpp
        tests/unit/test_sanity.cpp
        tests/unit/test_schema.cpp
        tests/unit/test_store.cpp
        tests/unit/test_validate.cpp
        tests/unit/test_diff.cpp
        tests/unit/test_link_classify.cpp
        tests/unit/test_lock.cpp
        tests/unit/test_translate_waybeam.cpp
        tests/unit/test_waybeam_client.cpp
        tests/unit/test_translate_wfb.cpp
        tests/unit/test_wfb_control.cpp
        tests/unit/test_translate_telemetry.cpp

        tests/unit/test_dl_apply_direction.cpp
        tests/unit/test_dl_roi_qp.cpp
        tests/integration/test_process.cpp
        tests/integration/test_supervisor.cpp
        tests/integration/test_orchestrator.cpp
        tests/integration/test_radio.cpp
        tests/integration/test_radio_tune_script.cpp
        tests/integration/test_beamforming.cpp
        tests/integration/test_daemon.cpp
        tests/integration/test_http_server.cpp
        tests/integration/test_http_handlers.cpp
        tests/unit/test_dl_encoder_client.cpp
        tests/unit/test_dl_radio_txpower.cpp
        tests/unit/test_dl_hello.cpp
        tests/unit/test_dl_osd.cpp
        tests/unit/test_dl_idr_listen.cpp
        tests/unit/test_dl_runtime_config.cpp
        tests/integration/test_dl_controller.cpp
    )
    add_test(NAME fpvd_tests COMMAND fpvd_tests)
endif()

# Install rules.
install(TARGETS fpvd RUNTIME DESTINATION bin)
install(PROGRAMS scripts/radio-up.sh DESTINATION libexec/fpvd)
install(PROGRAMS scripts/radio-tune.sh DESTINATION libexec/fpvd)
install(PROGRAMS scripts/S99fpvd DESTINATION ../etc/init.d)
install(FILES etc/defaults.json DESTINATION ../rom/etc/fpvd)
