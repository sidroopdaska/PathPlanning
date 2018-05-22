file(GLOB SOURCES
	*.cpp
	include/*.h
)

add_library(utilities ${SOURCES})

target_include_directories(utilities PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE include)

target_compile_features(utilities
    PUBLIC cxx_lambdas cxx_auto_type
    PRIVATE cxx_lambdas cxx_auto_type)