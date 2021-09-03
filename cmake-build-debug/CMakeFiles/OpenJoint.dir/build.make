# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/clion/163/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/163/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/shaw/File/ClionProject/OpenJoint

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shaw/File/ClionProject/OpenJoint/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/OpenJoint.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/OpenJoint.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/OpenJoint.dir/flags.make

CMakeFiles/OpenJoint.dir/library.cpp.o: CMakeFiles/OpenJoint.dir/flags.make
CMakeFiles/OpenJoint.dir/library.cpp.o: ../library.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shaw/File/ClionProject/OpenJoint/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/OpenJoint.dir/library.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/OpenJoint.dir/library.cpp.o -c /home/shaw/File/ClionProject/OpenJoint/library.cpp

CMakeFiles/OpenJoint.dir/library.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/OpenJoint.dir/library.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shaw/File/ClionProject/OpenJoint/library.cpp > CMakeFiles/OpenJoint.dir/library.cpp.i

CMakeFiles/OpenJoint.dir/library.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/OpenJoint.dir/library.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shaw/File/ClionProject/OpenJoint/library.cpp -o CMakeFiles/OpenJoint.dir/library.cpp.s

# Object files for target OpenJoint
OpenJoint_OBJECTS = \
"CMakeFiles/OpenJoint.dir/library.cpp.o"

# External object files for target OpenJoint
OpenJoint_EXTERNAL_OBJECTS =

libOpenJoint.so: CMakeFiles/OpenJoint.dir/library.cpp.o
libOpenJoint.so: CMakeFiles/OpenJoint.dir/build.make
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-gazebo3.so.3.9.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.4.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-gui3.so.3.7.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-common3-profiler.so.3.13.2
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-common3-events.so.3.13.2
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-common3-av.so.3.13.2
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.13.2
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.13.2
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-plugin1-loader.so.1.2.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-plugin1.so.1.2.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-transport8-log.so.8.2.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.2.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.8.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libQt5QuickControls2.so.5.12.8
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libQt5Quick.so.5.12.8
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libQt5Qml.so.5.12.8
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5.12.8
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.6.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.8.0
libOpenJoint.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libOpenJoint.so: CMakeFiles/OpenJoint.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shaw/File/ClionProject/OpenJoint/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libOpenJoint.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/OpenJoint.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/OpenJoint.dir/build: libOpenJoint.so
.PHONY : CMakeFiles/OpenJoint.dir/build

CMakeFiles/OpenJoint.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/OpenJoint.dir/cmake_clean.cmake
.PHONY : CMakeFiles/OpenJoint.dir/clean

CMakeFiles/OpenJoint.dir/depend:
	cd /home/shaw/File/ClionProject/OpenJoint/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shaw/File/ClionProject/OpenJoint /home/shaw/File/ClionProject/OpenJoint /home/shaw/File/ClionProject/OpenJoint/cmake-build-debug /home/shaw/File/ClionProject/OpenJoint/cmake-build-debug /home/shaw/File/ClionProject/OpenJoint/cmake-build-debug/CMakeFiles/OpenJoint.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/OpenJoint.dir/depend

