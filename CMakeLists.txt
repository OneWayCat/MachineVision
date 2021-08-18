cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)

    SET(dirlist "")
    FOREACH (child ${children})
        IF (IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${curdir}/${child})
        ENDIF ()
    ENDFOREACH ()
    SET(${result} ${dirlist})
ENDMACRO()

project(Measuring)
set(CMAKE_SHARED_LIBRARY_PREFIX "")

message(STATUS "Building and finding all third party libraries")
set(BOOST_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/boost_1_73_0)
find_package(Boost 1.60.0 REQUIRED)
find_package(OpenCV REQUIRED PATHS "D:/Programming/External Libs/OpenCV/install" NO_DEFAULT_PATH)
#add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/glm_0_9_9_8) # This line doesn't actually affect tinynurbs finding GLM for itself. In practice you can completely remove this line. There are no other dependencies for GLM in this project.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/tinynurbs)

message(STATUS "Adding all core libraries")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/core)

message(STATUS "Adding all utility libraries")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/drawing)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/generators)

message(STATUS "Adding all test files")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)

message(STATUS "Adding uniform noise testing file")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/noise_testing)