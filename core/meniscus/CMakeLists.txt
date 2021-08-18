add_library(MENISCUS_LIB SHARED
	src/meniscus.cpp
)

target_include_directories(MENISCUS_LIB PUBLIC include)
target_link_libraries(MENISCUS_LIB M1D_LIB)
