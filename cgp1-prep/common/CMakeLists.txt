set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(COMMON_SOURCES
    stats.cpp
    timer.cpp)

if (BUILD_SOURCE2CPP)
    set(KERNELS
        ../tesselate/shaders/basic.vert
        ../tesselate/shaders/basic.frag
        ../tesselate/shaders/genNormal.vert
        ../tesselate/shaders/genNormal.frag
        ../tesselate/shaders/simple.vert
        ../tesselate/shaders/simple.frag
        ../tesselate/shaders/phong.vert
        ../tesselate/shaders/phong.frag
        ../tesselate/shaders/phongRS.vert
        ../tesselate/shaders/phongRS.frag
        ../tesselate/shaders/phongRSmanip.vert
        ../tesselate/shaders/phongRSmanip.frag
        ../tesselate/shaders/rad_scaling_pass1.vert
        ../tesselate/shaders/rad_scaling_pass1.frag
        ../tesselate/shaders/rad_scaling_pass2.vert
        ../tesselate/shaders/rad_scaling_pass2.frag
        ../clh/texmark.cl)
    add_custom_command(
        OUTPUT source2cpp.cpp
        COMMAND ${PYTHON_EXECUTABLE} source2cpp.py ${KERNELS} ${CMAKE_CURRENT_BINARY_DIR}/source2cpp.cpp
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        MAIN_DEPENDENCY source2cpp.py
        DEPENDS ${KERNELS}
        COMMENT "Creating C++ source for shaders and kernels" VERBATIM)
    list(APPEND COMMON_SOURCES source2cpp.cpp)
endif()

add_library(common ${COMMON_SOURCES})
target_link_libraries(common ${Boost_SERIALIZATION_LIBRARY})

