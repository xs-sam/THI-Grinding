# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/xs/sifc/project/ros/workProject/tianyuan/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xs/sifc/project/ros/workProject/tianyuan/build

# Include any dependencies generated for this target.
include test/tcp_test/CMakeFiles/tcp_test.dir/depend.make

# Include the progress variables for this target.
include test/tcp_test/CMakeFiles/tcp_test.dir/progress.make

# Include the compile flags for this target's objects.
include test/tcp_test/CMakeFiles/tcp_test.dir/flags.make

test/tcp_test/CMakeFiles/tcp_test.dir/src/main.cpp.o: test/tcp_test/CMakeFiles/tcp_test.dir/flags.make
test/tcp_test/CMakeFiles/tcp_test.dir/src/main.cpp.o: /home/xs/sifc/project/ros/workProject/tianyuan/src/test/tcp_test/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xs/sifc/project/ros/workProject/tianyuan/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/tcp_test/CMakeFiles/tcp_test.dir/src/main.cpp.o"
	cd /home/xs/sifc/project/ros/workProject/tianyuan/build/test/tcp_test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tcp_test.dir/src/main.cpp.o -c /home/xs/sifc/project/ros/workProject/tianyuan/src/test/tcp_test/src/main.cpp

test/tcp_test/CMakeFiles/tcp_test.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tcp_test.dir/src/main.cpp.i"
	cd /home/xs/sifc/project/ros/workProject/tianyuan/build/test/tcp_test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xs/sifc/project/ros/workProject/tianyuan/src/test/tcp_test/src/main.cpp > CMakeFiles/tcp_test.dir/src/main.cpp.i

test/tcp_test/CMakeFiles/tcp_test.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tcp_test.dir/src/main.cpp.s"
	cd /home/xs/sifc/project/ros/workProject/tianyuan/build/test/tcp_test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xs/sifc/project/ros/workProject/tianyuan/src/test/tcp_test/src/main.cpp -o CMakeFiles/tcp_test.dir/src/main.cpp.s

# Object files for target tcp_test
tcp_test_OBJECTS = \
"CMakeFiles/tcp_test.dir/src/main.cpp.o"

# External object files for target tcp_test
tcp_test_EXTERNAL_OBJECTS =

/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: test/tcp_test/CMakeFiles/tcp_test.dir/src/main.cpp.o
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: test/tcp_test/CMakeFiles/tcp_test.dir/build.make
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /opt/ros/noetic/lib/libroscpp.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /opt/ros/noetic/lib/librosconsole.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /opt/ros/noetic/lib/librostime.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /opt/ros/noetic/lib/libcpp_common.so
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test: test/tcp_test/CMakeFiles/tcp_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xs/sifc/project/ros/workProject/tianyuan/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test"
	cd /home/xs/sifc/project/ros/workProject/tianyuan/build/test/tcp_test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tcp_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/tcp_test/CMakeFiles/tcp_test.dir/build: /home/xs/sifc/project/ros/workProject/tianyuan/devel/lib/tcp_test/tcp_test

.PHONY : test/tcp_test/CMakeFiles/tcp_test.dir/build

test/tcp_test/CMakeFiles/tcp_test.dir/clean:
	cd /home/xs/sifc/project/ros/workProject/tianyuan/build/test/tcp_test && $(CMAKE_COMMAND) -P CMakeFiles/tcp_test.dir/cmake_clean.cmake
.PHONY : test/tcp_test/CMakeFiles/tcp_test.dir/clean

test/tcp_test/CMakeFiles/tcp_test.dir/depend:
	cd /home/xs/sifc/project/ros/workProject/tianyuan/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xs/sifc/project/ros/workProject/tianyuan/src /home/xs/sifc/project/ros/workProject/tianyuan/src/test/tcp_test /home/xs/sifc/project/ros/workProject/tianyuan/build /home/xs/sifc/project/ros/workProject/tianyuan/build/test/tcp_test /home/xs/sifc/project/ros/workProject/tianyuan/build/test/tcp_test/CMakeFiles/tcp_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/tcp_test/CMakeFiles/tcp_test.dir/depend

