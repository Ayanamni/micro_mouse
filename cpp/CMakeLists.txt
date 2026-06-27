cmake_minimum_required(VERSION 3.18)
project(micromouse_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# UTF-8 source encoding (suppress C4819 warnings for Chinese comments)
add_compile_options("/utf-8")

# On Windows, ensure .pyd extension
if(WIN32)
    set(CMAKE_SHARED_MODULE_SUFFIX ".pyd")
endif()

# Find pybind11 (installed via pip)
find_package(pybind11 CONFIG REQUIRED)

# Interface library: include path so `#include "shared/types.hpp"` works
add_library(micromouse_shared INTERFACE)
target_include_directories(micromouse_shared INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}   # cpp/  → #include "shared/types.hpp" resolves
)

# ---- Localize Core (.pyd) ----
add_subdirectory(localize_core)

# ---- Control Core (.pyd) ----
add_subdirectory(control_core)
