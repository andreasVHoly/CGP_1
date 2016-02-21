if (BUILD_GUI)

    set(CMAKE_AUTOMOC TRUE)
    set(CMAKE_INCLUDE_CURRENT_DIR TRUE)  # MOC files generated in the binary dir

    set(GUI_SOURCES
       glwidget.cpp
       timer.cpp
       shape.cpp
       vecpnt.cpp
       view.cpp
       mesh.cpp
       window.cpp
       shaderProgram.cpp
       renderer.cpp)

    add_library(tess ${GUI_SOURCES})
    target_link_libraries(tess common
        ${Qt5Widgets_LIBRARIES} ${Qt5OpenGL_LIBRARIES}
        ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
    add_executable(tessviewer main.cpp)
    target_link_libraries(tessviewer tess)

endif()
