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

# Utility rule file for gensym.

# Include the progress variables for this target.
include libpng-1.6.37/CMakeFiles/gensym.dir/progress.make

libpng-1.6.37/CMakeFiles/gensym: libpng-1.6.37/libpng.sym


libpng-1.6.37/libpng.sym: libpng-1.6.37/scripts/sym.out
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/khaymon/kis_exam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating libpng.sym"
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && /usr/bin/cmake -E remove /home/khaymon/kis_exam/build/libpng-1.6.37/libpng.sym
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && /usr/bin/cmake -E copy /home/khaymon/kis_exam/build/libpng-1.6.37/scripts/sym.out /home/khaymon/kis_exam/build/libpng-1.6.37/libpng.sym

libpng-1.6.37/scripts/sym.out: ../libpng-1.6.37/scripts/sym.c
libpng-1.6.37/scripts/sym.out: libpng-1.6.37/pnglibconf.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/khaymon/kis_exam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating scripts/sym.out"
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && /usr/bin/cmake -DINPUT=/home/khaymon/kis_exam/libpng-1.6.37/scripts/sym.c -DOUTPUT=/home/khaymon/kis_exam/build/libpng-1.6.37/scripts/sym.out -P /home/khaymon/kis_exam/build/libpng-1.6.37/scripts/genout.cmake

libpng-1.6.37/pnglibconf.h: libpng-1.6.37/pnglibconf.out
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/khaymon/kis_exam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating pnglibconf.h"
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && /usr/bin/cmake -DOUTPUT=pnglibconf.h -P /home/khaymon/kis_exam/build/libpng-1.6.37/scripts/gensrc.cmake

libpng-1.6.37/pnglibconf.out: libpng-1.6.37/pnglibconf.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/khaymon/kis_exam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating pnglibconf.out"
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && /usr/bin/cmake -DINPUT=/home/khaymon/kis_exam/build/libpng-1.6.37/pnglibconf.c -DOUTPUT=/home/khaymon/kis_exam/build/libpng-1.6.37/pnglibconf.out -P /home/khaymon/kis_exam/build/libpng-1.6.37/scripts/genout.cmake

libpng-1.6.37/pnglibconf.c: ../libpng-1.6.37/scripts/pnglibconf.dfa
libpng-1.6.37/pnglibconf.c: ../libpng-1.6.37/scripts/options.awk
libpng-1.6.37/pnglibconf.c: ../libpng-1.6.37/pngconf.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/khaymon/kis_exam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating pnglibconf.c"
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && /usr/bin/cmake -DOUTPUT=pnglibconf.c -P /home/khaymon/kis_exam/build/libpng-1.6.37/scripts/gensrc.cmake

gensym: libpng-1.6.37/CMakeFiles/gensym
gensym: libpng-1.6.37/libpng.sym
gensym: libpng-1.6.37/scripts/sym.out
gensym: libpng-1.6.37/pnglibconf.h
gensym: libpng-1.6.37/pnglibconf.out
gensym: libpng-1.6.37/pnglibconf.c
gensym: libpng-1.6.37/CMakeFiles/gensym.dir/build.make

.PHONY : gensym

# Rule to build all files generated by this target.
libpng-1.6.37/CMakeFiles/gensym.dir/build: gensym

.PHONY : libpng-1.6.37/CMakeFiles/gensym.dir/build

libpng-1.6.37/CMakeFiles/gensym.dir/clean:
	cd /home/khaymon/kis_exam/build/libpng-1.6.37 && $(CMAKE_COMMAND) -P CMakeFiles/gensym.dir/cmake_clean.cmake
.PHONY : libpng-1.6.37/CMakeFiles/gensym.dir/clean

libpng-1.6.37/CMakeFiles/gensym.dir/depend:
	cd /home/khaymon/kis_exam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/khaymon/kis_exam /home/khaymon/kis_exam/libpng-1.6.37 /home/khaymon/kis_exam/build /home/khaymon/kis_exam/build/libpng-1.6.37 /home/khaymon/kis_exam/build/libpng-1.6.37/CMakeFiles/gensym.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libpng-1.6.37/CMakeFiles/gensym.dir/depend

