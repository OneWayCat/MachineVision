add_library(GENERATOR_LIB SHARED
    src/metrologyGen.cpp
    src/arcGen.cpp
    src/ellipseGen.cpp
    src/lineGen.cpp
    src/rectGen.cpp
    
)

target_include_directories(GENERATOR_LIB PUBLIC include)
target_link_libraries(GENERATOR_LIB M2D_LIB)
