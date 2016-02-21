set(TEST_SOURCES
    testutil.cpp
    test_mesh.cpp
    tilertest.cpp
)

add_executable(tilertest ${TEST_SOURCES})
# The -Wl,--whole-archive stuff is to prevent the linker from eliminating the
# test libraries, because there are no explicit references to them from the
# main program.
if (APPLE)
    set(WHOLE_ARCHIVE "-all_load")
    set(NO_WHOLE_ARCHIVE)
else()
    set(WHOLE_ARCHIVE "-Wl,--whole-archive")
    set(NO_WHOLE_ARCHIVE "-Wl,--no-whole-archive")
endif()

target_link_libraries(tilertest tess 
    ${WHOLE_ARCHIVE} ${TEST_LIBS} 
    ${NO_WHOLE_ARCHIVE} 
    ${CppUnit_LIBRARIES} 
    ${Boost_SERIALIZATION_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

add_test(
    NAME smoketest
    COMMAND tilertest -v --test=commit
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR})

if (${SMOKE_TEST})
    add_custom_command(TARGET tilertest POST_BUILD
        COMMAND tilertest --test=build
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        COMMENT "Running per-build tests" VERBATIM)
endif()
