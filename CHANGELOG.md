# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2026-07-10

### Added
- Three ROS 2 publisher nodes (`bno055_publisher_node`, `bno055_perf_publisher_node`, and `bno055_lifecycle_publisher_node`) in `src/ros2/`.
- ROS 2 Python Launch file (`launch/bno055_launch.py`) and parameters configuration file (`config/bno055_params.yaml`) to dynamically configure and launch the nodes.
- Mock-based ROS 2 node integration tests using GoogleTest in `tests/test_ros2_nodes.cpp`.
- Formatter validation (`clang-format`) checks in the GitHub Actions CI pipeline.
- Flexible GTest offline fallback resolution in `CMakeLists.txt` for offline environments (e.g., ROS buildfarms).

## [1.0.0] - 2026-07-10

### Added
- Robust, thread-safe, and dependency-free BNO055 library for Linux.
- Simplified API for beginners and visual debugging features.
- `vcpkg` and `Conan` integration support.
- Comprehensive Sphinx-based documentation (using Furo theme) including architecture, integration, and troubleshooting guides.
- GitHub Actions CI workflows and contribution guidelines (`CONTRIBUTING.md`).

### Changed
- Renamed the library and CMake target to `libbno055-linux`.

### Optimized
- Eliminated heap allocations in burst write functions to improve sensor write performance.
