cmake_minimum_required(VERSION 3.18)
project(baro_sim LANGUAGES CXX)

# If you need C++17 for <filesystem> etc.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 CONFIG REQUIRED)

# Source files
set(SRC
    noise_generator.cpp
    bindings.cpp
)

pybind11_add_module(baro_sim ${SRC})   # Produces baro_sim.*.so / .pyd
install(TARGETS baro_sim DESTINATION .)   # ← add THIS line