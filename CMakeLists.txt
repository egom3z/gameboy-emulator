cmake_minimum_required(VERSION 3.14)     # 3.14 gives us FetchContent convenience
project(GameBoyEmulator)

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Source and include paths
include_directories(${CMAKE_SOURCE_DIR}/include)

file(GLOB ALL_SRC "src/*.cpp")                                   # grab every .cpp
list(FILTER ALL_SRC EXCLUDE REGEX ".*/main\\.cpp$")              # remove main.cpp

# Core emulator library (static)
add_library(gbcore STATIC ${ALL_SRC})
target_include_directories(gbcore PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Build the emulator executable
add_executable(gbemu src/main.cpp)
target_link_libraries(gbemu PRIVATE gbcore)

# GoogleTest (downloaded automatically)
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0     # stable release tag
)
FetchContent_MakeAvailable(googletest)    # makes gtest + gtest_main targets

# Unit-test target
file(GLOB TEST_SOURCES "test/*.cpp")
add_executable(unit_tests ${TEST_SOURCES})
target_link_libraries(unit_tests
    PRIVATE
        gtest_main   # from googletest
        gbcore       # link the emulator library, NOT the gbemu executable
)

# Enable `ctest`
enable_testing()

include(GoogleTest)
gtest_discover_tests(unit_tests
    DISCOVERY_TIMEOUT 30)

add_test(NAME AllUnitTests COMMAND unit_tests)
