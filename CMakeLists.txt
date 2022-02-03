cmake_minimum_required(VERSION 3.16)

project(iwr6843aop VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Only set the standart settings if this is not part of another project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
        #set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
        set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
    endif()

    # Set a default build type if none was specified
    if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
      message(STATUS "Setting build type to 'Debug' as none was specified.")
      set(CMAKE_BUILD_TYPE
        Debug
          CACHE STRING "Choose the type of build." FORCE)
      # Set the possible values of build type for cmake-gui, ccmake
      set_property(
        CACHE CMAKE_BUILD_TYPE
        PROPERTY STRINGS
                "Debug"
                "Release"
                "MinSizeRel"
                "RelWithDebInfo")
    endif()

  # Disable in-source builds to prevent source tree corruption.
  if (" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(
          FATAL_ERROR
          " FATAL: In-source builds are not allowed.You should create a separate directory for build files."
  )
  endif (" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")

  set(CMAKE_INSTALL_PREFIX
  	"${CMAKE_BINARY_DIR}/INSTALL/" CACHE
    PATH "Directory used as INSTALL prefix" FORCE)
endif(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

include(FetchContent)

FetchContent_Declare(serial GIT_REPOSITORY git@github.com:kria-system/serial.git
   			GIT_TAG a1193a9b3b03389f7b79c9bdb6192c9d986fa87c
   			GIT_PROGRESS TRUE)
FetchContent_GetProperties(serial)
if(NOT serial_POPULATED)
    FetchContent_MakeAvailable(serial)
endif()

add_subdirectory(src)
add_subdirectory(test)
