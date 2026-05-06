cmake_minimum_required(VERSION 3.10)
project(grblHAL-flexihal-sim C)

set(CMAKE_C_STANDARD 11)

add_compile_definitions(F_CPU=16000000)
add_compile_definitions(GRBL_SIM)
add_compile_definitions(N_AXIS=5)

# Simulator defaults
add_compile_definitions(DEFAULT_HOMING_ENABLE=1)
add_compile_definitions(DEFAULT_HOMING_INIT_LOCK=1)
add_compile_definitions(DEFAULT_HOMING_FORCE_SET_ORIGIN=1)
add_compile_definitions(DEFAULT_X_MAX_TRAVEL=600.0f)
add_compile_definitions(DEFAULT_Y_MAX_TRAVEL=600.0f)
add_compile_definitions(DEFAULT_SOFT_LIMIT_ENABLE=1)
add_compile_definitions(DEFAULT_JOG_LIMIT_ENABLE=1)

# FlexiHAL simulator profile (static, always on)
add_compile_definitions(SDCARD_ENABLE=1)
add_compile_definitions(FS_ENABLE=3)
add_compile_definitions(NGC_EXPRESSIONS_ENABLE=1)
add_compile_definitions(ATCI_ENABLE=1)
add_compile_definitions(EXCLUSION_ZONES_ENABLE=1)

# Bundled plugins
# Keep the simulator build deterministic. Runtime plugin upload/rebuild is
# intentionally not part of the distributable simulator.
set(plugin_SRC
    plugins/sienci-atci-plugin.c
    plugins/exclusion_zones.c
)

# grblHAL core
# Keep the upstream grblHAL/core submodule clean and apply our simulator
# compatibility patch into the build tree.
find_package(Git REQUIRED)

set(GRBL_CORE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/src/grbl")
set(GRBL_CORE_PATCH "${CMAKE_SOURCE_DIR}/packaging/grbl-core-flexihal-sim.patch")
set(GRBL_CORE_BUILD_ROOT "${CMAKE_BINARY_DIR}/patched-src")
set(GRBL_CORE_BUILD_DIR "${GRBL_CORE_BUILD_ROOT}/grbl")

file(REMOVE_RECURSE "${GRBL_CORE_BUILD_DIR}")
file(MAKE_DIRECTORY "${GRBL_CORE_BUILD_DIR}")
file(COPY "${GRBL_CORE_SOURCE_DIR}/" DESTINATION "${GRBL_CORE_BUILD_DIR}"
     PATTERN ".git" EXCLUDE)

execute_process(
    COMMAND "${GIT_EXECUTABLE}" apply --whitespace=nowarn "${GRBL_CORE_PATCH}"
    WORKING_DIRECTORY "${GRBL_CORE_BUILD_DIR}"
    RESULT_VARIABLE GRBL_CORE_PATCH_RESULT
    OUTPUT_VARIABLE GRBL_CORE_PATCH_OUTPUT
    ERROR_VARIABLE GRBL_CORE_PATCH_ERROR
)
if(NOT GRBL_CORE_PATCH_RESULT EQUAL 0)
    message(FATAL_ERROR
        "Failed to apply grblHAL core simulator patch:\n"
        "${GRBL_CORE_PATCH_OUTPUT}\n${GRBL_CORE_PATCH_ERROR}")
endif()

include("${GRBL_CORE_BUILD_DIR}/CMakeLists.txt")

# Platform
if(WIN32)
    add_compile_definitions(PLATFORM_WINDOWS)
    set(platform_SRC
        src/platform_windows.h
        src/platform_windows.c
    )
    set(platform_LIB ws2_32)
endif()

if(UNIX)
    add_compile_definitions(PLATFORM_LINUX)
    set(platform_SRC
        src/platform_linux.h
        src/platform_linux.c
    )
    if(APPLE)
        set(platform_LIB pthread)
    else()
        set(platform_LIB rt pthread)
    endif()
endif()

# Simulator executable
add_executable(grblHAL_flexihal_sim
    src/main.c
    src/simulator.c
    src/driver.c
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/mcu.c
    src/serial.c
    src/grbl_interface.c
    src/websocket.c
    src/my_plugin.c
    src/fs_sim.c
    src/macros.c
    ${plugin_SRC}
    ${platform_SRC}
)

# Plugin include directories (so plugins can #include "driver.h" etc.)
target_include_directories(grblHAL_flexihal_sim PRIVATE
    ${GRBL_CORE_BUILD_ROOT}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/plugins
)

set(sim_LIBS
    grbl
    ${platform_LIB}
)

if(NOT MSVC)
    list(APPEND sim_LIBS m)
endif()

target_link_libraries(grblHAL_flexihal_sim PRIVATE ${sim_LIBS})

if(MSVC)
    file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/src/msvc_compat.h" MSVC_COMPAT_HEADER)
    target_compile_options(grblHAL_flexihal_sim PRIVATE "/FI${MSVC_COMPAT_HEADER}")
    target_compile_definitions(grblHAL_flexihal_sim PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Print plugin summary
message(STATUS "")
message(STATUS "=== grblHAL FlexiHAL Simulator Build ===")
message(STATUS "Bundled plugins:")
foreach(src ${plugin_SRC})
    get_filename_component(name ${src} NAME_WE)
    message(STATUS "  - ${name}")
endforeach()
message(STATUS "")

install(TARGETS grblHAL_flexihal_sim RUNTIME DESTINATION bin)
install(FILES README.md src/grbl/COPYING DESTINATION share/grblhal-flexihal-sim)

option(BUILD_WINDOWS_LAUNCHER "Build the native Windows launcher UI" ${WIN32})
if(BUILD_WINDOWS_LAUNCHER)
    enable_language(CXX)
    add_subdirectory(ui/windows-launcher)
endif()
