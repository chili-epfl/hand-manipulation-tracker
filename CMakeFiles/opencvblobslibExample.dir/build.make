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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474

# Include any dependencies generated for this target.
include example/CMakeFiles/opencvblobslibExample.dir/depend.make

# Include the progress variables for this target.
include example/CMakeFiles/opencvblobslibExample.dir/progress.make

# Include the compile flags for this target's objects.
include example/CMakeFiles/opencvblobslibExample.dir/flags.make

example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o: example/CMakeFiles/opencvblobslibExample.dir/flags.make
example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o: example/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o"
	cd /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/opencvblobslibExample.dir/main.cpp.o -c /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example/main.cpp

example/CMakeFiles/opencvblobslibExample.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencvblobslibExample.dir/main.cpp.i"
	cd /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example/main.cpp > CMakeFiles/opencvblobslibExample.dir/main.cpp.i

example/CMakeFiles/opencvblobslibExample.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencvblobslibExample.dir/main.cpp.s"
	cd /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example/main.cpp -o CMakeFiles/opencvblobslibExample.dir/main.cpp.s

example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o.requires:
.PHONY : example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o.requires

example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o.provides: example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o.requires
	$(MAKE) -f example/CMakeFiles/opencvblobslibExample.dir/build.make example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o.provides.build
.PHONY : example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o.provides

example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o.provides.build: example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o

# Object files for target opencvblobslibExample
opencvblobslibExample_OBJECTS = \
"CMakeFiles/opencvblobslibExample.dir/main.cpp.o"

# External object files for target opencvblobslibExample
opencvblobslibExample_EXTERNAL_OBJECTS =

example/opencvblobslibExample: example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o
example/opencvblobslibExample: example/CMakeFiles/opencvblobslibExample.dir/build.make
example/opencvblobslibExample: library/libopencvblobslib.a
example/opencvblobslibExample: /usr/local/lib/libopencv_aruco.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_videostab.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_videoio.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_video.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_superres.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_stitching.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_shape.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_photo.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_objdetect.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_ml.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_imgproc.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_highgui.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_flann.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_features2d.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_core.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_calib3d.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_features2d.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_ml.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_highgui.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_videoio.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_flann.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_video.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_imgproc.so.3.1.0
example/opencvblobslibExample: /usr/local/lib/libopencv_core.so.3.1.0
example/opencvblobslibExample: example/CMakeFiles/opencvblobslibExample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable opencvblobslibExample"
	cd /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opencvblobslibExample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/CMakeFiles/opencvblobslibExample.dir/build: example/opencvblobslibExample
.PHONY : example/CMakeFiles/opencvblobslibExample.dir/build

example/CMakeFiles/opencvblobslibExample.dir/requires: example/CMakeFiles/opencvblobslibExample.dir/main.cpp.o.requires
.PHONY : example/CMakeFiles/opencvblobslibExample.dir/requires

example/CMakeFiles/opencvblobslibExample.dir/clean:
	cd /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example && $(CMAKE_COMMAND) -P CMakeFiles/opencvblobslibExample.dir/cmake_clean.cmake
.PHONY : example/CMakeFiles/opencvblobslibExample.dir/clean

example/CMakeFiles/opencvblobslibExample.dir/depend:
	cd /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474 /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474 /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example /home/leo/Applications/opencv/blobslib/OpenCVBlobsLib-opencvblobslib-a5eb474/example/CMakeFiles/opencvblobslibExample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/CMakeFiles/opencvblobslibExample.dir/depend
