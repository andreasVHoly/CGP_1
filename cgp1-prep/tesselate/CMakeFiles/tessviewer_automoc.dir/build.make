# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/Honours/CGP/cgpass1/cgp1-prep

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/Honours/CGP/cgpass1/cgp1-prep

# Utility rule file for tessviewer_automoc.

# Include the progress variables for this target.
include tesselate/CMakeFiles/tessviewer_automoc.dir/progress.make

tesselate/CMakeFiles/tessviewer_automoc:
	$(CMAKE_COMMAND) -E cmake_progress_report /home/user/Honours/CGP/cgpass1/cgp1-prep/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Automoc for target tessviewer"
	cd /home/user/Honours/CGP/cgpass1/cgp1-prep/tesselate && /usr/bin/cmake -E cmake_automoc /home/user/Honours/CGP/cgpass1/cgp1-prep/tesselate/CMakeFiles/tessviewer_automoc.dir/ ""

tessviewer_automoc: tesselate/CMakeFiles/tessviewer_automoc
tessviewer_automoc: tesselate/CMakeFiles/tessviewer_automoc.dir/build.make
.PHONY : tessviewer_automoc

# Rule to build all files generated by this target.
tesselate/CMakeFiles/tessviewer_automoc.dir/build: tessviewer_automoc
.PHONY : tesselate/CMakeFiles/tessviewer_automoc.dir/build

tesselate/CMakeFiles/tessviewer_automoc.dir/clean:
	cd /home/user/Honours/CGP/cgpass1/cgp1-prep/tesselate && $(CMAKE_COMMAND) -P CMakeFiles/tessviewer_automoc.dir/cmake_clean.cmake
.PHONY : tesselate/CMakeFiles/tessviewer_automoc.dir/clean

tesselate/CMakeFiles/tessviewer_automoc.dir/depend:
	cd /home/user/Honours/CGP/cgpass1/cgp1-prep && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Honours/CGP/cgpass1/cgp1-prep /home/user/Honours/CGP/cgpass1/cgp1-prep/tesselate /home/user/Honours/CGP/cgpass1/cgp1-prep /home/user/Honours/CGP/cgpass1/cgp1-prep/tesselate /home/user/Honours/CGP/cgpass1/cgp1-prep/tesselate/CMakeFiles/tessviewer_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tesselate/CMakeFiles/tessviewer_automoc.dir/depend

