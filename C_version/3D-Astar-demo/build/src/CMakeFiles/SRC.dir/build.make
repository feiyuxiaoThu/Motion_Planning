# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build

# Include any dependencies generated for this target.
include src/CMakeFiles/SRC.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/SRC.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/SRC.dir/flags.make

src/CMakeFiles/SRC.dir/Astarsearch.cpp.o: src/CMakeFiles/SRC.dir/flags.make
src/CMakeFiles/SRC.dir/Astarsearch.cpp.o: ../src/Astarsearch.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/SRC.dir/Astarsearch.cpp.o"
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SRC.dir/Astarsearch.cpp.o -c /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/src/Astarsearch.cpp

src/CMakeFiles/SRC.dir/Astarsearch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SRC.dir/Astarsearch.cpp.i"
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/src/Astarsearch.cpp > CMakeFiles/SRC.dir/Astarsearch.cpp.i

src/CMakeFiles/SRC.dir/Astarsearch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SRC.dir/Astarsearch.cpp.s"
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/src/Astarsearch.cpp -o CMakeFiles/SRC.dir/Astarsearch.cpp.s

src/CMakeFiles/SRC.dir/Astarsearch.cpp.o.requires:

.PHONY : src/CMakeFiles/SRC.dir/Astarsearch.cpp.o.requires

src/CMakeFiles/SRC.dir/Astarsearch.cpp.o.provides: src/CMakeFiles/SRC.dir/Astarsearch.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/SRC.dir/build.make src/CMakeFiles/SRC.dir/Astarsearch.cpp.o.provides.build
.PHONY : src/CMakeFiles/SRC.dir/Astarsearch.cpp.o.provides

src/CMakeFiles/SRC.dir/Astarsearch.cpp.o.provides.build: src/CMakeFiles/SRC.dir/Astarsearch.cpp.o


src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o: src/CMakeFiles/SRC.dir/flags.make
src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o: ../src/ConvertConfigToIndex.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o"
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o -c /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/src/ConvertConfigToIndex.cpp

src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.i"
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/src/ConvertConfigToIndex.cpp > CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.i

src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.s"
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/src/ConvertConfigToIndex.cpp -o CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.s

src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o.requires:

.PHONY : src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o.requires

src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o.provides: src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/SRC.dir/build.make src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o.provides.build
.PHONY : src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o.provides

src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o.provides.build: src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o


# Object files for target SRC
SRC_OBJECTS = \
"CMakeFiles/SRC.dir/Astarsearch.cpp.o" \
"CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o"

# External object files for target SRC
SRC_EXTERNAL_OBJECTS =

src/libSRC.a: src/CMakeFiles/SRC.dir/Astarsearch.cpp.o
src/libSRC.a: src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o
src/libSRC.a: src/CMakeFiles/SRC.dir/build.make
src/libSRC.a: src/CMakeFiles/SRC.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libSRC.a"
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src && $(CMAKE_COMMAND) -P CMakeFiles/SRC.dir/cmake_clean_target.cmake
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SRC.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/SRC.dir/build: src/libSRC.a

.PHONY : src/CMakeFiles/SRC.dir/build

src/CMakeFiles/SRC.dir/requires: src/CMakeFiles/SRC.dir/Astarsearch.cpp.o.requires
src/CMakeFiles/SRC.dir/requires: src/CMakeFiles/SRC.dir/ConvertConfigToIndex.cpp.o.requires

.PHONY : src/CMakeFiles/SRC.dir/requires

src/CMakeFiles/SRC.dir/clean:
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src && $(CMAKE_COMMAND) -P CMakeFiles/SRC.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/SRC.dir/clean

src/CMakeFiles/SRC.dir/depend:
	cd /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/src /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src /home/echo/Desktop/feiyu/Coding/feiyuxiaothu/Motion_Planning/C_version/3D-Astar-demo/build/src/CMakeFiles/SRC.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/SRC.dir/depend

