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
CMAKE_SOURCE_DIR = /home/khaymon/kis_exam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/khaymon/kis_exam/build

# Include any dependencies generated for this target.
include libpng-1.6.37/CMakeFiles/pngunknown.dir/depend.make

# Include the progress variables for this target.
include libpng-1.6.37/CMakeFiles/pngunknown.dir/progress.make

# Include the compile flags for this target's objects.
include libpng-1.6.37/CMakeFiles/pngunknown.dir/flags.make

libpng-1.6.37/CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.o: libpng-1.6.37/CMakeFiles/pngunknown.dir/flags.make
libpng-1.6.37/CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.o: ../libpng-1.6.37/contrib/libtests/pngunknown.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khaymon/kis_exam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object libpng-1.6.37/CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.o"
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.o   -c /home/khaymon/kis_exam/libpng-1.6.37/contrib/libtests/pngunknown.c

libpng-1.6.37/CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.i"
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khaymon/kis_exam/libpng-1.6.37/contrib/libtests/pngunknown.c > CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.i

libpng-1.6.37/CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.s"
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khaymon/kis_exam/libpng-1.6.37/contrib/libtests/pngunknown.c -o CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.s

# Object files for target pngunknown
pngunknown_OBJECTS = \
"CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.o"

# External object files for target pngunknown
pngunknown_EXTERNAL_OBJECTS =

libpng-1.6.37/pngunknown: libpng-1.6.37/CMakeFiles/pngunknown.dir/contrib/libtests/pngunknown.c.o
libpng-1.6.37/pngunknown: libpng-1.6.37/CMakeFiles/pngunknown.dir/build.make
libpng-1.6.37/pngunknown: libpng-1.6.37/libpng16.so.16.37.0
libpng-1.6.37/pngunknown: /usr/lib/x86_64-linux-gnu/libz.so
libpng-1.6.37/pngunknown: /usr/lib/x86_64-linux-gnu/libm.so
libpng-1.6.37/pngunknown: libpng-1.6.37/CMakeFiles/pngunknown.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/khaymon/kis_exam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable pngunknown"
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pngunknown.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libpng-1.6.37/CMakeFiles/pngunknown.dir/build: libpng-1.6.37/pngunknown

.PHONY : libpng-1.6.37/CMakeFiles/pngunknown.dir/build

libpng-1.6.37/CMakeFiles/pngunknown.dir/clean:
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && $(CMAKE_COMMAND) -P CMakeFiles/pngunknown.dir/cmake_clean.cmake
.PHONY : libpng-1.6.37/CMakeFiles/pngunknown.dir/clean

libpng-1.6.37/CMakeFiles/pngunknown.dir/depend:
	cd /home/khaymon/kis_exam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/khaymon/kis_exam /home/khaymon/kis_exam/libpng-1.6.37 /home/khaymon/kis_exam/build /home/khaymon/kis_exam/build/libpng-1.6.37 /home/khaymon/kis_exam/build/libpng-1.6.37/CMakeFiles/pngunknown.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libpng-1.6.37/CMakeFiles/pngunknown.dir/depend

