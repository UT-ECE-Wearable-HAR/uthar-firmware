cmake_minimum_required(VERSION 3.4...3.18)
project(dmp)

add_subdirectory(pybind11)
pybind11_add_module(dmp src/dmp.cpp)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(dmp PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
