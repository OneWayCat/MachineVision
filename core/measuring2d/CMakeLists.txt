add_library(M2D_LIB SHARED
        src/objects/metrologyObject.cpp
        src/objects/arcObject.cpp
        src/objects/ellipseObject.cpp
        src/objects/rectObject.cpp
        src/objects/lineObject.cpp
        src/objects/arcProjectionObject.cpp

        src/applyMetrologyModel.cpp
        src/metcout.cpp
        src/inRegion.cpp

        src/ransac/ransacFit.cpp
        src/ransac/ransacError.cpp        
        )

target_include_directories(M2D_LIB PUBLIC include)
target_link_libraries(M2D_LIB M1D_LIB)