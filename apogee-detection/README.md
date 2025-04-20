*REPO LAYOUT*

noise_generator.cpp/.hpp --> Core physics & barometer math 

height_data.txt --> altitude readings captured from MASTRAN

AD.cpp --> Full C++ simulation of our apogee-detection system

Makefile --> creates the .exe files, necessary for pure C++ builds

apogee-task.c/h --> stripped down C version of AD.cpp for use on the Flight computer
This is the code that will fly on Limelight. 

*Files to use with simulator.py SEB python wrapper*
noise_generator.cpp/.hpp 
height_data.txt 
bindings.cpp --> uses pybind11 
CMakeLists.txt --> build recipe used by scikit-build-core (auto-generated)
pyproject.toml --> necessary metadata
testing-library.py --> how to use the library 

*Build command for library* 
1. Create / activate your python environment 
2. Run - 
        python -m pip install --upgrade pip
        python -m pip install scikit-build-core[pyproject] pybind11[global]
3. Build / install module
        # From the repo root (where pyproject.toml sits)
p       ython -m pip install -v .          # standard install
        #   or
        python -m pip install -e .          # editable / dev install

*Extras*
Kalman_rocket_apogee.cpp --> experimental Kalman filter technique, still in the works
comp-formulas-example.py --> simple example of the conversion math used for noise compensation
in the MS5611 barometer

