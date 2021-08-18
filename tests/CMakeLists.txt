message(STATUS "Building 1dm test files")
add_executable(measuring_test measuring/measuring_test.cpp)
add_executable(fuzzy_measuring_test measuring/fuzzyMeasuring_test.cpp)

message(STATUS "Building metrology test files")
add_executable(metrology_testarcs metrology/metrology_testarcs.cpp)
add_executable(metrology_testlines metrology/metrology_testlines.cpp)
add_executable(metrology_testrects metrology/metrology_testrects.cpp)
add_executable(metrology_testellipse metrology/metrology_testellipse.cpp)
add_executable(metrology_testfull metrology/metrology_testfull.cpp)
add_executable(metrology_testgenerators metrology/metrology_testgenerators.cpp)

message(STATUS "Building meniscus test files")
add_executable(meniscus_testing meniscus/meniscus_testing.cpp)

message(STATUS "Building bead test files")
add_executable(bead_testing bead/beadTest.cpp)

message(STATUS "Linking all relevant libraries to executables")
target_link_libraries(measuring_test DRAW_MEASURING_LIB)
target_link_libraries(fuzzy_measuring_test DRAW_MEASURING_LIB)

target_link_libraries(metrology_testarcs DRAW_METROLOGY_LIB)
target_link_libraries(metrology_testlines DRAW_METROLOGY_LIB)
target_link_libraries(metrology_testrects DRAW_METROLOGY_LIB)
target_link_libraries(metrology_testellipse DRAW_METROLOGY_LIB)
target_link_libraries(metrology_testfull DRAW_METROLOGY_LIB)
target_link_libraries(metrology_testgenerators DRAW_METROLOGY_LIB GENERATOR_LIB)

target_link_libraries(meniscus_testing DRAWING_LIB MENISCUS_LIB)

target_link_libraries(bead_testing BEAD_LIB DRAW_MEASURING_LIB DRAW_BEAD_LIB )