# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/husky/EvoBinoSDK-Samples/SampleTemp/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/husky/EvoBinoSDK-Samples/SampleTemp/src-build

# Include any dependencies generated for this target.
include CMakeFiles/SampleTemp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SampleTemp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SampleTemp.dir/flags.make

CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o: CMakeFiles/SampleTemp.dir/flags.make
CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o: /home/husky/EvoBinoSDK-Samples/SampleTemp/src/MarkerDetection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/husky/EvoBinoSDK-Samples/SampleTemp/src-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o -c /home/husky/EvoBinoSDK-Samples/SampleTemp/src/MarkerDetection.cpp

CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/husky/EvoBinoSDK-Samples/SampleTemp/src/MarkerDetection.cpp > CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.i

CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/husky/EvoBinoSDK-Samples/SampleTemp/src/MarkerDetection.cpp -o CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.s

CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o.requires:

.PHONY : CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o.requires

CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o.provides: CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o.requires
	$(MAKE) -f CMakeFiles/SampleTemp.dir/build.make CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o.provides.build
.PHONY : CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o.provides

CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o.provides.build: CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o


CMakeFiles/SampleTemp.dir/temp.cpp.o: CMakeFiles/SampleTemp.dir/flags.make
CMakeFiles/SampleTemp.dir/temp.cpp.o: /home/husky/EvoBinoSDK-Samples/SampleTemp/src/temp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/husky/EvoBinoSDK-Samples/SampleTemp/src-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/SampleTemp.dir/temp.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SampleTemp.dir/temp.cpp.o -c /home/husky/EvoBinoSDK-Samples/SampleTemp/src/temp.cpp

CMakeFiles/SampleTemp.dir/temp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SampleTemp.dir/temp.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/husky/EvoBinoSDK-Samples/SampleTemp/src/temp.cpp > CMakeFiles/SampleTemp.dir/temp.cpp.i

CMakeFiles/SampleTemp.dir/temp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SampleTemp.dir/temp.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/husky/EvoBinoSDK-Samples/SampleTemp/src/temp.cpp -o CMakeFiles/SampleTemp.dir/temp.cpp.s

CMakeFiles/SampleTemp.dir/temp.cpp.o.requires:

.PHONY : CMakeFiles/SampleTemp.dir/temp.cpp.o.requires

CMakeFiles/SampleTemp.dir/temp.cpp.o.provides: CMakeFiles/SampleTemp.dir/temp.cpp.o.requires
	$(MAKE) -f CMakeFiles/SampleTemp.dir/build.make CMakeFiles/SampleTemp.dir/temp.cpp.o.provides.build
.PHONY : CMakeFiles/SampleTemp.dir/temp.cpp.o.provides

CMakeFiles/SampleTemp.dir/temp.cpp.o.provides.build: CMakeFiles/SampleTemp.dir/temp.cpp.o


# Object files for target SampleTemp
SampleTemp_OBJECTS = \
"CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o" \
"CMakeFiles/SampleTemp.dir/temp.cpp.o"

# External object files for target SampleTemp
SampleTemp_EXTERNAL_OBJECTS =

SampleTemp: CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o
SampleTemp: CMakeFiles/SampleTemp.dir/temp.cpp.o
SampleTemp: CMakeFiles/SampleTemp.dir/build.make
SampleTemp: /usr/local/EvoBinoSDK/bin/libevo_stereocamera64.so
SampleTemp: /usr/local/EvoBinoSDK/bin/libevo_depthcamera64.so
SampleTemp: libMarkerDetection.a
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
SampleTemp: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
SampleTemp: CMakeFiles/SampleTemp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/husky/EvoBinoSDK-Samples/SampleTemp/src-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable SampleTemp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SampleTemp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SampleTemp.dir/build: SampleTemp

.PHONY : CMakeFiles/SampleTemp.dir/build

CMakeFiles/SampleTemp.dir/requires: CMakeFiles/SampleTemp.dir/MarkerDetection.cpp.o.requires
CMakeFiles/SampleTemp.dir/requires: CMakeFiles/SampleTemp.dir/temp.cpp.o.requires

.PHONY : CMakeFiles/SampleTemp.dir/requires

CMakeFiles/SampleTemp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SampleTemp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SampleTemp.dir/clean

CMakeFiles/SampleTemp.dir/depend:
	cd /home/husky/EvoBinoSDK-Samples/SampleTemp/src-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/husky/EvoBinoSDK-Samples/SampleTemp/src /home/husky/EvoBinoSDK-Samples/SampleTemp/src /home/husky/EvoBinoSDK-Samples/SampleTemp/src-build /home/husky/EvoBinoSDK-Samples/SampleTemp/src-build /home/husky/EvoBinoSDK-Samples/SampleTemp/src-build/CMakeFiles/SampleTemp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SampleTemp.dir/depend

