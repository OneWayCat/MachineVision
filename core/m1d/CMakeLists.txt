SUBDIRLIST(M1D_LIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
LIST(APPEND M1D_LIB_INCLUDE_DIRS "include")

file(GLOB M1D_LIB_INCLUDE
        "include/edge_elements/*.hpp"
        "include/fuzzy/*.hpp"
        "include/util/*.hpp"
        "include/*.hpp")

file(GLOB M1D_LIB_SRC
        "src/edge_elements/*.cpp"
        "src/fuzzy/*.cpp"
        "src/util/*.cpp"
        "src/*.cpp")

add_library(M1D_LIB SHARED ${M1D_LIB_INCLUDE} ${M1D_LIB_SRC})

target_include_directories(M1D_LIB PUBLIC ${Boost_INCLUDE_DIR} ${M1D_LIB_INCLUDE_DIRS})
target_link_libraries(M1D_LIB UTILS_LIB ${OpenCV_LIBS})