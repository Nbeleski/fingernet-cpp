#
# CMake root CMakeLists.txt
#
cmake_minimum_required (VERSION 3.10)

project ("FingerNet")

#
# Força msvc a compilar tudo como MT/MTd
#
if (POLICY CMP0054)
   cmake_policy(SET CMP0054 NEW)
endif()

if (MSVC OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
   option (DLIB_FORCE_MSVC_STATIC_RUNTIME "use static runtime" ON)
   if (DLIB_FORCE_MSVC_STATIC_RUNTIME)
      foreach(flag_var
         CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
         CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
         if(${flag_var} MATCHES "/MD")
            string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
         endif()
      endforeach(flag_var)
   endif ()
endif()

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Configs
set(BUILD_SHARED_LIBS False)
set(CMAKE_BUILD_TYPE Release)					    # set for Release

# Include sub-projects.

add_subdirectory ("fingernet_static")       # build the static libs with opencv / openvino
add_subdirectory ("fingernet")              # dynamic library that exports static lib function(s)
add_subdirectory ("test_project")					# build test, linked to the dynamic lib

# Properties
# ---

# Install
# ---