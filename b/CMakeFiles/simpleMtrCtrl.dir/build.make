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
CMAKE_SOURCE_DIR = /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/b

# Include any dependencies generated for this target.
include CMakeFiles/simpleMtrCtrl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/simpleMtrCtrl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simpleMtrCtrl.dir/flags.make

CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.o: CMakeFiles/simpleMtrCtrl.dir/flags.make
CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.o: ../tests/simpleMtrCtrl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/b/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.o -c /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/tests/simpleMtrCtrl.cpp

CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/tests/simpleMtrCtrl.cpp > CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.i

CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/tests/simpleMtrCtrl.cpp -o CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.s

# Object files for target simpleMtrCtrl
simpleMtrCtrl_OBJECTS = \
"CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.o"

# External object files for target simpleMtrCtrl
simpleMtrCtrl_EXTERNAL_OBJECTS =

simpleMtrCtrl: CMakeFiles/simpleMtrCtrl.dir/tests/simpleMtrCtrl.cpp.o
simpleMtrCtrl: CMakeFiles/simpleMtrCtrl.dir/build.make
simpleMtrCtrl: libAKD_ecat.a
simpleMtrCtrl: extern/SOEM/libsoem.a
simpleMtrCtrl: CMakeFiles/simpleMtrCtrl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/b/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable simpleMtrCtrl"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simpleMtrCtrl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simpleMtrCtrl.dir/build: simpleMtrCtrl

.PHONY : CMakeFiles/simpleMtrCtrl.dir/build

CMakeFiles/simpleMtrCtrl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simpleMtrCtrl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simpleMtrCtrl.dir/clean

CMakeFiles/simpleMtrCtrl.dir/depend:
	cd /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/b && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/b /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/b /home/bench/ws/src/akd_ethercat_lib/AKD-Ethercat-Master/b/CMakeFiles/simpleMtrCtrl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simpleMtrCtrl.dir/depend

