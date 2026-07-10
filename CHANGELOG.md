# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-07-10

### Added
- Robust, thread-safe, and dependency-free BNO055 library for Linux.
- Simplified API for beginners and visual debugging features.
- ROS 2 publisher node example.
- `vcpkg` and `Conan` integration support.
- Comprehensive Sphinx-based documentation (using Furo theme) including architecture, integration, and troubleshooting guides.
- GitHub Actions CI workflows and contribution guidelines (`CONTRIBUTING.md`).

### Changed
- Renamed the library and CMake target to `libbno055-linux`.

### Optimized
- Eliminated heap allocations in burst write functions to improve sensor write performance.
