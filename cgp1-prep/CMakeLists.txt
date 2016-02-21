# This is needed for automoc
cmake_minimum_required(VERSION 2.8.7)
include(CheckLibraryExists)

option(SMOKE_TEST "run a smoke test on every build" 1)
option(COVERAGE "compile for use with gcov" 0)
option(ASAN "compile with the address sanitiser" 0)
option(TSAN "compile with the thread sanitiser" 0)
option(SYNTHESIS_STATS "collect extra statistics about synthesis" 0)
enable_testing()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

### Check for dependencies
set(Boost_USE_MULTITHREADED ON)
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)
find_package(Boost 1.49 REQUIRED COMPONENTS filesystem program_options serialization system)
find_package(Doxygen)
find_package(PkgConfig REQUIRED)
find_package(PythonInterp)
#find_package(OpenCL)
find_package(Qt5Widgets)
find_package(Qt5OpenGL)
find_package(OpenGL)
find_package(GLUT)
pkg_check_modules(CppUnit cppunit)
pkg_check_modules(GLEW glew)

### Set include and link directories and definitions
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/khronos_headers
    ${Boost_INCLUDE_DIRS}
)
link_directories(
    ${PROJECT_SOURCE_DIR}/common
    ${PROJECT_SOURCE_DIR}/tesselate
    ${CppUnit_LIBRARY_DIRS}
    ${Boost_LIB_DIRS}
)

if (Qt5Widgets_FOUND)
    include_directories(${Qt5Widgets_INCLUDE_DIRS})
    add_definitions(${Qt5Widgets_DEFINITIONS})
endif()
if (Qt5OpenGL_FOUND)
    include_directories(${Qt5OpenGL_INCLUDE_DIRS})
    add_definitions(${Qt5OpenGL_DEFINITIONS})
endif()
if (OPENGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIR})
endif()
if (GLUT_FOUND)
    include_directories(${GLUT_INCLUDE_DIR})
endif()
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    link_directories(${GLEW_LIBRARY_DIRS})
endif()
if (CPPUNIT_FOUND)
    include_directories(${CPPUNIT_INCLUDE_DIRS})
    link_directories(${CppUnit_LIBRARY_DIRS})
endif()

### Check which components to build
if (PYTHONINTERP_FOUND AND OPENGL_FOUND)
    set(BUILD_SOURCE2CPP TRUE)
endif()
if (Qt5Widgets_FOUND
        AND Qt5OpenGL_FOUND
        AND OPENGL_FOUND
        AND GLUT_FOUND
        AND GLEW_FOUND
        AND PYTHONINTERP_FOUND)
    set(BUILD_GUI TRUE)
endif()
if (DOXYGEN_FOUND)
    set(BUILD_DOCS TRUE)
endif()
if (CPPUNIT_FOUND)
    set(BUILD_TESTS TRUE)
endif()

### Configure documentation build
if (BUILD_DOCS)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doc/html/index.html
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        MAIN_DEPENDENCY ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        DEPENDS
            ${CMAKE_CURRENT_SOURCE_DIR}/common/*
            ${CMAKE_CURRENT_SOURCE_DIR}/tesselate/*
            ${CMAKE_CURRENT_SOURCE_DIR}/test/*
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation" VERBATIM)
    add_custom_target(doc ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doc/html/index.html)
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES doc/html)
endif()

### Set flags specific to the compiler
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
    # Eigen gives lots of warnings for this
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-enum-compare")
    # On Apple the framework headers are found ahead of the khronos_headers
    # directory, and they give deprecation warnings about OpenCL 1.1 functions.
    if (APPLE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
    endif()
    if (NOT APPLE)
        set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_CXX_LINKER_FLAGS_RELEASE} -s")
    endif()
    # Link time optimization gives much better performance
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -flto")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -flto")
    set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} -flto")
endif()
if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wno-unused-local-typedefs")
    if (${COVERAGE})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    endif()
    if (${ASAN})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libasan")
    endif()
    if (${TSAN})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread -fpie -fno-omit-frame-pointer")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie -static-libtsan")
    endif()
endif()
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Eigen generates lots of warnings due to the register keyword being deprecated
    add_definitions(-Dregister=)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DUTS_DEBUG_CONTAINERS")

add_subdirectory(common)
add_subdirectory(tesselate)
add_subdirectory(test)
