cmake_minimum_required(VERSION 3.18)
project(cnc_hal VERSION 3.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
find_library(GREEMASTER_LIBRARY NAMES GREEMASTER PATHS /opt/GreeMaster/lib REQUIRED)
find_library(XML2_LIBRARY NAMES xml2 PATHS /opt/GreeMaster/lib REQUIRED)
find_package(Threads REQUIRED)
include(GNUInstallDirs)
set(HAL_INSTALLED_DEVICES_JSON "${CMAKE_INSTALL_FULL_DATADIR}/cnc_hal/devices.json")

set(HAL_SOURCES
    src/internal/hal_config.c
    src/internal/hal_context.c
    src/common/devdict.c
    src/Greemaster/main_demo.c
    src/Greemaster/device.c
    src/Greemaster/device_table.c
    src/Greemaster/entry_access.c
    src/Greemaster/servo_step.c
    src/Greemaster/slave_list.c)

add_library(cnc_hal SHARED ${HAL_SOURCES})
target_include_directories(cnc_hal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src src/Greemaster)
target_compile_options(cnc_hal PRIVATE -Wall -Wextra -Werror)
target_compile_definitions(cnc_hal PRIVATE "DEVICES_JSON_PATH=\"${HAL_INSTALLED_DEVICES_JSON}\"")
target_link_libraries(cnc_hal PRIVATE ${GREEMASTER_LIBRARY} ${XML2_LIBRARY} Threads::Threads m)
target_link_options(cnc_hal PRIVATE
    "LINKER:--no-undefined"
    "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/cmake/hal.exports")
set_property(TARGET cnc_hal APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/hal.exports")
set_target_properties(cnc_hal PROPERTIES VERSION 3.0 SOVERSION 3)

option(HAL_BUILD_STATIC "Build the HAL static library" OFF)
if(HAL_BUILD_STATIC)
    add_library(cnc_hal_static STATIC ${HAL_SOURCES})
    target_include_directories(cnc_hal_static PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src src/Greemaster)
    target_compile_options(cnc_hal_static PRIVATE -Wall -Wextra -Werror)
    target_compile_definitions(cnc_hal_static PRIVATE "DEVICES_JSON_PATH=\"${HAL_INSTALLED_DEVICES_JSON}\"")
    # 静态归档保留 SDK 等外部符号，由最终使用者链接这些依赖。
    target_link_libraries(cnc_hal_static PUBLIC ${GREEMASTER_LIBRARY} ${XML2_LIBRARY} Threads::Threads m)
    set_target_properties(cnc_hal_static PROPERTIES OUTPUT_NAME cnc_hal)
endif()

install(TARGETS cnc_hal LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
if(HAL_BUILD_STATIC)
    install(TARGETS cnc_hal_static ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
install(FILES src/Greemaster/devices.json DESTINATION ${CMAKE_INSTALL_DATADIR}/cnc_hal)

include(CTest)
option(HAL_BUILD_HARDWARE_TEST "Build the manually run hardware test program" OFF)
if(HAL_BUILD_HARDWARE_TEST)
    add_executable(hal_hardware_test
        examples/hal_hardware_test.c
        examples/hal_hardware_test_config.c)
    target_link_libraries(hal_hardware_test PRIVATE cnc_hal Threads::Threads m)
    target_compile_options(hal_hardware_test PRIVATE -Wall -Wextra -Werror)
    if(BUILD_TESTING)
        add_test(NAME hal_hardware_selftest COMMAND hal_hardware_test selftest)
    endif()
endif()

if(BUILD_TESTING)
    add_executable(hal_public_link test/public_link.c)
    target_link_libraries(hal_public_link PRIVATE cnc_hal)
    add_test(NAME hal_public_link COMMAND hal_public_link)
    if(HAL_BUILD_STATIC)
        add_executable(hal_static_link test/public_link.c)
        target_link_libraries(hal_static_link PRIVATE cnc_hal_static)
        add_test(NAME hal_static_link COMMAND hal_static_link)
    endif()
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME hal_regressions COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/run_driver_tests.py)
endif()
