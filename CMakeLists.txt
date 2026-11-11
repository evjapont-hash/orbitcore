cmake_minimum_required(VERSION 3.20)
project(orbitcore VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ORBITCORE_BUILD_TESTS "Build the test suite" ON)
option(ORBITCORE_SANITIZE "Enable AddressSanitizer + UBSan (GCC/Clang)" OFF)

add_library(orbitcore
  src/time.cpp
  src/frames.cpp
  src/tle.cpp
  src/sgp4.cpp
  src/catalog.cpp
  src/conjunction.cpp)
target_include_directories(orbitcore PUBLIC include)
target_compile_features(orbitcore PUBLIC cxx_std_20)

if(MSVC)
  target_compile_options(orbitcore PRIVATE /W4)
else()
  target_compile_options(orbitcore PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

if(ORBITCORE_SANITIZE AND NOT MSVC)
  target_compile_options(orbitcore PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(orbitcore PUBLIC -fsanitize=address,undefined)
endif()

if(EMSCRIPTEN)
  # Browser build: orbitcore compiled to WebAssembly with JavaScript bindings (embind).
  add_executable(orbitcore_wasm wasm/orbitcore_wasm.cpp)
  target_link_libraries(orbitcore_wasm PRIVATE orbitcore)
  target_link_options(orbitcore_wasm PRIVATE
    -lembind
    -sMODULARIZE=1
    -sEXPORT_NAME=createOrbitcore
    -sALLOW_MEMORY_GROWTH=1
    -sENVIRONMENT=web,worker,node)
else()
  add_executable(orbit_cli tools/orbit_cli.cpp)
  target_link_libraries(orbit_cli PRIVATE orbitcore)

  add_executable(catalog_bench tools/catalog_bench.cpp)
  target_link_libraries(catalog_bench PRIVATE orbitcore)

  add_executable(conjunction_cli tools/conjunction_cli.cpp)
  target_link_libraries(conjunction_cli PRIVATE orbitcore)

  if(ORBITCORE_BUILD_TESTS)
    enable_testing()
    add_executable(orbitcore_tests
      tests/test_main.cpp
      tests/test_time_frames.cpp
      tests/test_tle.cpp
      tests/test_sgp4_vallado.cpp
      tests/test_catalog.cpp
      tests/test_engine.cpp
      tests/test_visibility.cpp
      tests/test_conjunction.cpp)
    target_include_directories(orbitcore_tests PRIVATE tests wasm)
    target_link_libraries(orbitcore_tests PRIVATE orbitcore)
    target_compile_definitions(orbitcore_tests PRIVATE
      ORBITCORE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
    add_test(NAME orbitcore_tests COMMAND orbitcore_tests)
  endif()
endif()