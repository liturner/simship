
add_library(ttsimship
    include/TT/data.h
    include/TT/model_flight_dynamics.h
    include/TT/model_radar.h
)

set_target_properties(ttsimship PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 17
)

target_include_directories(ttsimship
    PUBLIC
    include
    /usr/include/JSBSim
    PRIVATE
    src
)

target_link_libraries(ttsimship
    ttsim
    /usr/lib/libJSBSim.a
)


add_executable(simship src/main.cpp)
target_link_libraries(simship ttsimship)
install(TARGETS simship
    LIBRARY
    PUBLIC_HEADER
)


add_executable(ttsimshipTests
    src/model_radar.tests.cpp
)

set_target_properties(ttsimshipTests PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 17
)

target_link_libraries(ttsimshipTests GTest::gtest_main ttsimship)

gtest_discover_tests(ttsimshipTests
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

enable_testing()

install(TARGETS ttsimship
    LIBRARY
    PUBLIC_HEADER
)

install(DIRECTORY include/ DESTINATION include)