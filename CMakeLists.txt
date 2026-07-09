cmake_minimum_required(VERSION 3.10)
project(bno055lib VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    src/bno055.cpp
)

# Header files
set(HEADERS
    include/bno055lib/bno055.hpp
)

# Add library
add_library(bno055lib SHARED ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(bno055lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(bno055lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(Threads REQUIRED)
target_link_libraries(bno055lib PRIVATE Threads::Threads)

# Example executable
add_executable(calibrate_imu examples/calibrate.cpp)
target_link_libraries(calibrate_imu PRIVATE bno055lib)

# Installation definitions
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Install targets
install(TARGETS bno055lib
    EXPORT bno055libTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS calibrate_imu
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install headers
install(DIRECTORY include/bno055lib
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export targets
install(EXPORT bno055libTargets
    FILE bno055libTargets.cmake
    NAMESPACE bno055lib::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bno055lib
)

# Configure config file
configure_package_config_file(
    cmake/bno055libConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/bno055libConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bno055lib
)

# Write version file
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/bno055libConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install config files
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/bno055libConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/bno055libConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bno055lib
)
