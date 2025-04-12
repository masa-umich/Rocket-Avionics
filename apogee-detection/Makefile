# Makefile
# Build rules for general.cpp

# Compiler
CXX ?= g++

# Compiler flags
CXXFLAGS ?= --std=c++17 -Wall -pedantic -g -Wno-sign-compare -Wno-comment -Werror

# CXXFLAGS ?= -std=c++1z -Werror -Wall -Wconversion -Wpedantic

test: main.exe
	echo PASS

noise_generator: noise_generator.cpp
	$(CXX) $(CXXFLAGS) noise_generator.cpp -o noise_generator.exe

AD : AD.cpp noise_generator.cpp
	$(CXX) $(CXXFLAGS) AD.cpp noise_generator.cpp -o AD.exe

.SUFFIXES:

.PHONY: clean

clean:
	rm -rvf *.out *.exe *.dSYM *.stackdump

