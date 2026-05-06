add_executable(grblHAL_flexihal_launcher WIN32
    src/main.cpp
)

target_compile_features(grblHAL_flexihal_launcher PRIVATE cxx_std_17)
target_link_libraries(grblHAL_flexihal_launcher PRIVATE comdlg32)

if(TARGET grblHAL_flexihal_sim)
    add_dependencies(grblHAL_flexihal_launcher grblHAL_flexihal_sim)
endif()

install(TARGETS grblHAL_flexihal_launcher RUNTIME DESTINATION bin)
