#
# CMake test .exe
#
cmake_minimum_required (VERSION 3.8)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add dependencies
#add_subdirectory(../visa_uv_verification visa_uv_verification_build)

# Add source to this project's executable.
add_executable (test_project main.cpp base64_codec.h base64_codec.cpp)

# Link dependencies
#target_link_libraries(test_project PRIVATE visa_verification_static)
target_link_libraries(test_project PUBLIC fingernet)

# Add dependencies headers
#target_include_directories(test_project PRIVATE $<TARGET_PROPERTY:visa_verification_static,INCLUDE_DIRECTORIES>)
target_include_directories(test_project PRIVATE ${CMAKE_SOURCE_DIR}/fingernet_static)

# Set properties
set_property(TARGET test_project PROPERTY CXX_STANDARD 20)
set_property(TARGET test_project PROPERTY CXX_STANDARD_REQUIRED ON)

# TODO: Add tests and install targets if needed.
# include(GNUInstallDirs)
install(TARGETS test_project 
	CONFIGURATIONS Debug 
	DESTINATION ${CMAKE_SOURCE_DIR}/out/Debug
)

install(TARGETS test_project 
	CONFIGURATIONS Release 
	DESTINATION ${CMAKE_SOURCE_DIR}/out/Release
)

#install(elft Release ${PROJECT_BINARY_DIR}/Release/lib)