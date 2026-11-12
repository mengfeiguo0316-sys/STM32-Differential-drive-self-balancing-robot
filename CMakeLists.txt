cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Balance_car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM)
# 如果 User/ 下存在 C++ 源文件，自动启用 C++ 支持
file(GLOB_RECURSE _USER_CXX_CHECK CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.cxx
)
if(_USER_CXX_CHECK)
    enable_language(CXX)
endif()

# ============================================================================
# 用户代码（User/ 目录）：自动收集，无需手动维护本文件
# 新增/删除 User/ 下的文件后，CMake 会自动重新配置（CONFIGURE_DEPENDS）
# ============================================================================

# 自动收集 User/ 下所有源文件
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.s
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.S
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.cxx
)
list(REMOVE_DUPLICATES USER_SOURCES)

# 自动收集 User/ 下所有头文件所在目录作为 include 路径
file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/User/*.hpp
)
set(USER_INCLUDE_DIRS "")
foreach(_header ${USER_HEADERS})
    get_filename_component(_header_dir "${_header}" DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS "${_header_dir}")
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    ${USER_SOURCES}
    ${USER_HEADERS}
    User/Middleware/Inc/VOFA+.h
    User/Middleware/Inc/PID.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
