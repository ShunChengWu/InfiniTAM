cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

UNSET(COMPILE_WITH_CUDA)
OPTION(BUILD_IMGLOADER_TEST "BUILD TEST" OFF)
###############################################################################
### -- 3rd party
SET(EXTCMAKE ${PROJECT_SOURCE_DIR}/cmake)
INCLUDE(${EXTCMAKE}/UseCUDA.cmake)

find_package(PkgConfig REQUIRED)

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIRS})

find_package(Eigen3 REQUIRED)


###########################################################################################
SET(targetname ImageLoader)
file(GLOB_RECURSE srcs *.cpp)
file(GLOB_RECURSE headers *.h *.hpp)
file(GLOB_RECURSE protos *.proto)

SET(sources
	${srcs}
		${protos}
)

SET(headers
		${headers}
	#include/ImageLoader/ImageLoader.hpp
#	Loader/ScanNet/sensorData.h
#	Loader/ScanNet/sensorData/stb_image_write.h
#	Loader/ScanNet/sensorData/stb_image.h
)

INCLUDE(${EXTCMAKE}/SetCUDALibTarget.cmake)
target_link_libraries(${targetname}
		#PRIVATE Utils
#		PRIVATE FrameGrabber
		PRIVATE ORUtils
		#PUBLIC CxxTools
		#PUBLIC ${JSONCPP_LIBRARIES}
		PUBLIC ${OpenCV_LIBS}
)
target_include_directories(${targetname} PUBLIC ${EIGEN3_INCLUDE_DIR})

#target_include_directories(${targetname} PUBLIC ${PROJECT_SOURCE_DIR}/ThirdParty/stb)
target_include_directories(${targetname} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_compile_definitions(${targetname} PUBLIC -DWITH_OPENCV PUBLIC -DWITH_REALSENSE2 PUBLIC -DCOMPILE_WITH_OPENCV)
set_target_properties(${targetname} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${targetname} PROPERTIES CXX_STANDARD 11)
target_include_directories(${targetname} PUBLIC ./include)
target_include_directories(${targetname} PUBLIC ./Loader)
