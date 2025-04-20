#include <pybind11/pybind11.h>
#include "noise_generator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(baro_sim, m) {
    m.doc() = "Barometer noise / compensation engine (C++ core)";

    //Environment class
    py::class_<Environment>(m, "Environment")
        .def(py::init<>())
        .def("compute_pressure", &Environment::compute_pressure,
             py::arg("height_m"))
        .def("compute_temp",     &Environment::compute_temp,
             py::arg("height_m"));

    //Barometer class
    py::class_<Barometer>(m, "Barometer")
        .def(py::init<>())
        .def("forward_calculation", &Barometer::forward_calculation,
             py::arg("D1"), py::arg("D2"))
        .def("guess_and_check", &Barometer::guess_and_check,
             py::arg("pressure"), py::arg("temperature"));

    //Free functions
    m.def("gaussian_random",
        &gaussian_random,                             
        py::arg("mean") = 0.0,
        py::arg("stddev") = 1.0);
}

