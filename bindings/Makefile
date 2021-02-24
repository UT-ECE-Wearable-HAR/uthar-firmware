PYBIND_INCS = $(shell python3 -m pybind11 --includes)
EXT = $(shell python3-config --extension-suffix)


all: build

build: dmp.cpp
	c++ -O3 -Wall -shared -fPIC $(PYBIND_INCS) dmp.cpp -o dmp$(EXT)
