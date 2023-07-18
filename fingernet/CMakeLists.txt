cmake_minimum_required(VERSION 3.10)
# Every project needs a name.  We call this the "examples" project.

#
# config de cmake para incluir recursos acessíveis como arquivo
# https://github.com/vector-of-bool/cmrc/blob/master/CMakeRC.cmake
#
#include(CMakeRC.cmake)
#cmrc_add_resource_library(emb_lib ALIAS embedding::rc embedding.cnn)


# main component, src files
add_library(fingernet SHARED dllmain.cpp exports.def)

# Link dependencies
target_link_libraries(fingernet PRIVATE fingernet_static)

# Add dependencies headers
target_include_directories(fingernet PRIVATE $<TARGET_PROPERTY:fingernet_static,INCLUDE_DIRECTORIES>)

# Properties
set_property(TARGET fingernet PROPERTY CXX_STANDARD 20)
set_property(TARGET fingernet PROPERTY CXX_STANDARD_REQUIRED ON)

# Installation targets
install(TARGETS fingernet 
	CONFIGURATIONS Debug 
	DESTINATION ${CMAKE_SOURCE_DIR}/out/Debug
	PUBLIC_HEADER DESTINATION ${CMAKE_SOURCE_DIR}/out/include
)

install(TARGETS fingernet 
	CONFIGURATIONS Release 
	DESTINATION ${CMAKE_SOURCE_DIR}/out/Release
	PUBLIC_HEADER DESTINATION ${CMAKE_SOURCE_DIR}/out/include
)



