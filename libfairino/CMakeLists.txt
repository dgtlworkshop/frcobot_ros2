cmake_minimum_required(VERSION 3.13)
project(libfairino VERSION 3.9.9 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

set(SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/fairino-cpp-sdk/libfairino")
if(NOT EXISTS "${SDK_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "Fairino SDK submodule is missing. Run: git submodule update --init --recursive")
endif()

# Use the SDK's sources and build settings, but not its standalone test program.
add_subdirectory("${SDK_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/sdk" EXCLUDE_FROM_ALL)
set_target_properties(fairino PROPERTIES
  EXCLUDE_FROM_ALL FALSE
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
)
target_compile_features(fairino PUBLIC cxx_std_17)
target_include_directories(fairino PUBLIC
  $<BUILD_INTERFACE:${SDK_DIR}/src/include/Robot-EN>
  $<INSTALL_INTERFACE:include/libfairino>
)
target_link_libraries(fairino PRIVATE Threads::Threads)

install(TARGETS fairino
  EXPORT export_libfairino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY "${SDK_DIR}/src/include/Robot-EN/"
  DESTINATION include/libfairino
  FILES_MATCHING PATTERN "*.h"
)
install(FILES fairino-cpp-sdk/LICENSE DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_libfairino HAS_LIBRARY_TARGET)
ament_package()
