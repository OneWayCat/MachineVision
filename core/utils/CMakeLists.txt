add_library(UTILS_LIB SHARED
    src/mathutils.cpp
    src/arrutils.cpp
)

target_include_directories(UTILS_LIB PUBLIC ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} include)