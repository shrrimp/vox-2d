#Change this if you need to target a specific CMake version
cmake_minimum_required(VERSION 3.16)

# Enable debug symbols by default
# must be done before project() statement
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()
# (you can also set it on the command line: -D CMAKE_BUILD_TYPE=Release)

project(SFML_Project)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

# Set version information in a config.h file
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
configure_file(
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  )
include_directories("${PROJECT_BINARY_DIR}")

# Define sources and executable
set(EXECUTABLE_NAME "SFML_Project")
add_executable(${EXECUTABLE_NAME}
  main.cpp
  Engine.cpp
  PixelBody.cpp
  XPM.cpp)
target_compile_features(${EXECUTABLE_NAME} PRIVATE cxx_std_17)


# Detect and add SFML
find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System Audio Network)
target_link_libraries(${EXECUTABLE_NAME} PRIVATE
  SFML::Graphics SFML::Window SFML::System SFML::Audio SFML::Network)


# Install target
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)


# CPack packaging
include(InstallRequiredSystemLibraries)
set(CPACK_PACKAGE_VERSION_MAJOR "${VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${VERSION_MINOR}")
include(CPack)

