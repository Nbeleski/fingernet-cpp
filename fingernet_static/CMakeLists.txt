cmake_minimum_required(VERSION 3.10)
# Every project needs a name.  We call this the "examples" project.

#
# config de cmake para incluir recursos acessíveis como arquivo
# https://github.com/vector-of-bool/cmrc/blob/master/CMakeRC.cmake
#
include(CMakeRC.cmake)
cmrc_add_resource_library(resources_lib ALIAS resources::rc NAMESPACE resources fingernet.xml fingernet.bin)

# Project output alias
set(ProjectOutputName fingernet_static)

# 
# include opencv
#
message("MSVC compiler")
  if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/external/opencv/x64-Debug/x64/vc17/staticlib")
  else()
    set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/external/opencv/x64-Release/x64/vc17/staticlib")
  endif()

find_package(OpenCV REQUIRED core imgproc imgcodecs)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

#
# Static dependency
#
###

#
# Openvino
#
set(OpenVINO_DIR "${CMAKE_SOURCE_DIR}/external/openvino/runtime/cmake")
find_package(OpenVINO REQUIRED)
message(STATUS "OpenVINO OK...")

# main component, src files
add_library(${ProjectOutputName} STATIC "fingernet.h" "fingernet_static.cpp" /
			"FingerNetModel.h" "FingerNetModel.cpp"   "resources_api.hpp"   )

# link libs and include dirs
target_link_libraries(${ProjectOutputName} PUBLIC openvino::runtime ${OpenCV_LIBS} resources::rc)
#target_include_directories(${ProjectOutputName} PUBLIC ${CMAKE_SOURCE_DIR}/include)


# Properties
set_target_properties(${ProjectOutputName} PROPERTIES PUBLIC_HEADER fingernet.h)
set_property(TARGET ${ProjectOutputName} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${ProjectOutputName} PROPERTY CXX_STANDARD_REQUIRED ON)

# Installation targets
install(TARGETS ${ProjectOutputName} 
	CONFIGURATIONS Debug 
	DESTINATION ${CMAKE_SOURCE_DIR}/out/Debug
	PUBLIC_HEADER DESTINATION ${CMAKE_SOURCE_DIR}/out/include
)

install(TARGETS ${ProjectOutputName} 
	CONFIGURATIONS Release 
	DESTINATION ${CMAKE_SOURCE_DIR}/out/Release
	PUBLIC_HEADER DESTINATION ${CMAKE_SOURCE_DIR}/out/include
)