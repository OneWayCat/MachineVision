add_library(BEAD_LIB SHARED
        src/beadModel.cpp
        src/beadMeasuring.cpp
        src/fitSpline.cpp
        src/trackBead.cpp
        src/bead.cpp
        src/pickContour.cpp
        src/posBead.cpp)

target_include_directories(BEAD_LIB PUBLIC
        #tinynurbs/include
        #glm
        include)

target_link_libraries(BEAD_LIB M1D_LIB glm tinynurbs)