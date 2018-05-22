file(GLOB SOURCES
	*.cpp
	include/*.h
)

add_library(search ${SOURCES})

include_directories(../visualizer/include)
include_directories(../utilities/include)

target_link_libraries(search utilities)
target_link_libraries(search visualizer)

target_include_directories(search PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE include)

target_compile_features(search
    PUBLIC cxx_lambdas cxx_auto_type
    PRIVATE cxx_lambdas cxx_auto_type)