
add_library(ttsim
    include/TT/math.h
    include/TT/model.h
    include/TT/rpr_fom.h
    include/TT/transform.h
    src/math.cpp
    src/transform.cpp
    src/model.cpp
    include/TT/simulation.h
    src/simulation.cpp
    include/TT/logging.h
)

set_target_properties(ttsim PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 17
)

target_include_directories(ttsim
    PUBLIC include
    PRIVATE src
)

target_link_libraries(ttsim Eigen3::Eigen)


add_executable(ttsimTests
    src/transform.tests.cpp
)

set_target_properties(ttsimTests PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 17
)

target_link_libraries(ttsimTests
    GTest::gtest_main
    ttsim
)

gtest_discover_tests(ttsimTests
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

enable_testing()

install(TARGETS ttsim
    LIBRARY
    PUBLIC_HEADER
)

install(DIRECTORY include/ DESTINATION include)