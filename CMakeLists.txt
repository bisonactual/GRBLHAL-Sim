cmake_minimum_required(VERSION 3.10)
project(grblHAL-flexihal-sim C)

set(CMAKE_C_STANDARD 11)

add_compile_definitions(F_CPU=16000000)
add_compile_definitions(GRBL_SIM)
add_compile_definitions(N_AXIS=5)

# ── Plugin toggles ──────────────────────────────────────────────────────
# Enable plugins by passing -DPLUGIN_EXCLUSION_ZONES=ON to cmake, e.g.:
#   cmake .. -DPLUGIN_EXCLUSION_ZONES=ON
#
# Or enable them all:
#   cmake .. -DPLUGINS_ALL=ON
#
# Plugins can also be dropped into the plugins/ folder via the web UI.
# The web UI triggers a rebuild that picks them up automatically.

option(PLUGINS_ALL              "Enable all available plugins" OFF)
option(PLUGIN_EXCLUSION_ZONES   "Enable Exclusion Zones plugin" OFF)

# Collect plugin sources and defines
set(plugin_SRC "")
set(plugin_DEFS "")
set(plugin_DIRS "")

# ── Auto-discover plugins dropped via web UI ──
file(GLOB webui_plugins "${CMAKE_SOURCE_DIR}/plugins/*.c")
if(webui_plugins)
    foreach(psrc ${webui_plugins})
        get_filename_component(pname ${psrc} NAME_WE)
        string(REPLACE "-" "_" pname_safe "${pname}")
        list(APPEND plugin_SRC "${psrc}")

        # Scan the .c file for its actual #if XXXX_ENABLE guard
        file(READ "${psrc}" _plugin_src)
        string(REGEX MATCH "#if[^\n]*([A-Z_]+_ENABLE)" _match "${_plugin_src}")
        if(_match)
            string(REGEX MATCH "([A-Z_]+_ENABLE)" _enable_macro "${_match}")
            message(STATUS "Plugin (web UI): ${pname} -> guard: ${_enable_macro}")
            list(APPEND plugin_DEFS "${_enable_macro}=1")
        else()
            # Fallback: uppercase sanitized filename
            string(TOUPPER "${pname_safe}" pname_upper)
            message(STATUS "Plugin (web UI): ${pname} -> guard (guessed): ${pname_upper}_ENABLE")
            list(APPEND plugin_DEFS "${pname_upper}_ENABLE=1")
        endif()
    endforeach()
    list(APPEND plugin_DIRS "${CMAKE_SOURCE_DIR}/plugins")
endif()

# ── Exclusion Zones (manual cmake flag) ──
if(PLUGIN_EXCLUSION_ZONES OR PLUGINS_ALL)
    # Look for the plugin in common locations
    if(EXISTS "${CMAKE_SOURCE_DIR}/../exclusionzones/exclusion_zones/exclusion_zones.c")
        set(EZ_DIR "${CMAKE_SOURCE_DIR}/../exclusionzones/exclusion_zones")
    elseif(EXISTS "${CMAKE_SOURCE_DIR}/plugins/exclusion_zones.c")
        # Already picked up by auto-discover above
        set(EZ_DIR "")
    else()
        message(FATAL_ERROR "Exclusion Zones plugin enabled but source not found. "
                "Expected at ../exclusionzones/exclusion_zones/ or plugins/exclusion_zones.c")
    endif()
    if(EZ_DIR)
        message(STATUS "Plugin (cmake flag): Exclusion Zones (${EZ_DIR})")
        list(APPEND plugin_SRC "${EZ_DIR}/exclusion_zones.c")
        list(APPEND plugin_DEFS "EXCLUSION_ZONES_ENABLE=1")
        list(APPEND plugin_DIRS "${EZ_DIR}")
    endif()
endif()

# ── (Add more plugins here following the same pattern) ──

# ── grblHAL core ────────────────────────────────────────────────────────
include(src/grbl/CMakeLists.txt)

# ── Platform ────────────────────────────────────────────────────────────
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

# ── Simulator executable ────────────────────────────────────────────────
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
    src/uf2.c
    src/my_plugin.c
    ${plugin_SRC}
    ${platform_SRC}
)

# Plugin compile definitions
foreach(def ${plugin_DEFS})
    target_compile_definitions(grblHAL_flexihal_sim PRIVATE ${def})
endforeach()

# Plugin include directories (so plugins can #include "driver.h" etc.)
target_include_directories(grblHAL_flexihal_sim PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${plugin_DIRS}
)

target_link_libraries(grblHAL_flexihal_sim PRIVATE
    m
    grbl
    ${platform_LIB}
)

# ── Print plugin summary ───────────────────────────────────────────────
message(STATUS "")
message(STATUS "=== grblHAL FlexiHAL Simulator Build ===")
if(plugin_SRC)
    message(STATUS "Plugins enabled:")
    foreach(src ${plugin_SRC})
        get_filename_component(name ${src} NAME_WE)
        message(STATUS "  - ${name}")
    endforeach()
else()
    message(STATUS "No plugins enabled. Use -DPLUGIN_EXCLUSION_ZONES=ON etc.")
endif()
message(STATUS "")
