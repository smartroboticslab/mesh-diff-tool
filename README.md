# mesh-diff-tool

This is a tool for computing the accuracy and completeness of 3D mesh
reconstructions.


## Dependencies

* GCC 7+ or clang 6+ (for C++ 17 features and OpenMP)
* CMake 3.0+
* PCL

To install the dependencies on Ubuntu 20.04 run

``` sh
sudo apt-get --yes install g++ cmake libpcl-dev
```


## Building

To clone this repository and all submodules run

``` sh
git clone --recurse-submodules https://github.com/smartroboticslab/mesh-diff-tool.git
```

or if you already cloned without --recurse-submodules run

``` sh
git submodule update --init --recursive
```

Then build the project using CMake

```
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```


## Usage

Two executables will be built inside the `build` directory, `compare-multiple`
and `compare-single`. Run either one with the `--help` option to see usage
information.


## License

Copyright 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich</br>
Copyright 2021-2022 Dimos Tzoumanikas</br>
Copyright 2022-2023 Sotiris Papatheodorou</br>

mesh-diff-tool is distributed under the
[BSD 3-clause license](LICENSES/BSD-3-Clause.txt).
