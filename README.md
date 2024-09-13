# Dubins Path Smoothing (DPS)

- [Dubins Path Smoothing (DPS)](#dubins-path-smoothing-dps)
- [Introduction](#introduction)
- [Download](#download)
- [Dependencies](#dependencies)
- [Project Structure](#project-structure)
- [Compile](#compile)
    - [CUDA compilation](#cuda-compilation)
    - [Test suite compilation](#test-suite-compilation)
- [Usage](#usage)
- [Test Suite](#test-suite)
    - [Interpolation](#interpolation)
    - [Path finding](#path-finding)
    - [Roadmap](#roadmap)
- [Documentation](#documentation)
- [Contributors](#contributors)
- [Licence](#licence)

----------

# Introduction

This framework uses a novel and efficient algorithm for smoothing polylines in motion planning tasks. The output is a bounded curvature-path (Dubins path) which approximates the polyline and at the same time:
1) Has minimal length
2) Is ${G^1}$ continous
3) Ensures collision freedom if the hypothesis are respected

The framework implements its core features in both c++20 and CUDA with standard c++17. The user needs to compile the desired version and then they only need to provide the curvature radius and the series of points which represent the polyline; the framework will calculate the final trajectory. Furthermore, we implemented different tests to compare our algorithm with the state-of-the-art.

# Download

To download the code, you can clone the repository:

```shell
git clone --recurse-submodules https://www.github.com/KubriksCodeTN/DPS
```
The external modules that are downloaded are:
- [Open Motion Planning Library](https://github.com/ompl/ompl) to which we compare since it contains state-of-the-art algorithms for motion planning.
- [Multi-Point Dynamic Programming](https://github.com/icosac/mpdp) which is the state-of-the-art algorithm for the Multi-Point Markov-Dubins Problem.
- [Nanoflann](https://github.com/jlblancoc/nanoflann) for internal usage
- [Visilibity](https://github.com/karlobermeyer/VisiLibity1) for internal usage
- [Clipper2](https://github.com/AngusJohnson/Clipper2) for internal usage
- [Bench_mr](#https://github.com/robot-motion/bench-mr) for extracting obstacles from maps saved as SVG images (already present in `tests/dubins_ompl`)
  - [Collision-2d](https://github.com/eric-heiden/collision2d) used within Bench-mr

# Dependencies

The code is written in C++20 and requires the following libraries:
- [Boost](https://www.boost.org/) for boost_program_option;
- [TBB](https://github.com/oneapi-src/oneTBB) for parallelization

For compilation, it requires 
- A C/C++ compiler supporting C++20;
- If you want to compile the code with CUDA support, you need to have a CUDA-enabled GPU and the CUDA toolkit installed with CUDA at least 11 (std-c++17) or 12 (std-c++20).
- CMake >= 3.16
- [OPTIONAL] Doxygen for creating the documentation
- [OPTIONAL] Matplotlib for plotting data with Python

# Project Structure

This repository is organized in different folders:
- ```src/``` contains the C++ and CUDA versions of our algorithm
- ```scripts/``` contains useful scripts to generate points and to visualize the tests' output data
- ```tests/``` contains the python scripts to run the test suite and the different programs we tested together with dps:
  - ```dubins_ompl/``` contains the programs which use OMPL that are run during the tests
  - ```mpdp/``` contains the implementation of Multi-Point Dynamic Programming
  - ```roadmap/``` inserts our algorithm inside a scenario with obstacles: it creates a roadmap of the environment, find a path inside it and, finally, it calls DPS to obtain the final trajectory
  - ```rrt/``` contains our implementation of RRT* to find a valid polyline to be used by DPS

# Compile

The code can be compiled with cmake by running:

```shell
cmake -B build -S . 
cmake --build build
```

You can find the executables inside the `build` directory.

### CUDA compilation

By default the compilation is disabled, if you want to enable it, just set to `ON` the option `DPS_BUILD_CUDA` inside the top-level CMakeLists.

It's possible that you need to change the `CUDA_ARCHITECTURES` in the CMake file to match your GPU architecture.

### Test suite compilation

By default the compilation of the test suite is disabled, if you want to enable it, just set to `ON` the option `DPS_BUILD_TESTS` inside the top-level CMakeLists.

> **_NOTE:_** To run the test suite you also need to compile the CUDA version.

# Usage

You have 2 possibilities when using our framework: you may want to use DPS inside your project or you may want to run the test suite.

If you are here to run the test suite, please read [Test Suite](#test-suite).

To add DPS in your project is really simple. The first step is to compile the library as explained in [Compile](#compile) and to link it while compiling your project; then you need to go inside `src/src/` and include `planner.hpp` that you'll find inside.

A simple code snippet could look like this:

```cpp
#include "planner.hpp"

...

// create a Planner object
auto planner = Planner();

// retrieve the polyline in some way...
auto polyline = get_polyline(); 

double time, len;

// call the test method to calculate the final path
auto path = planner.dps(polyline, time, len);

...
```

# Test Suite

The test suite is composed of 3 different types of tests:

1. Interpolation
2. Path finding
3. Roadmap

Each one can be run individually and will produce a separated output file under ```test_out/```

### Interpolation

Here we test how DPS performs at handling different polylines with growing number of points. OMPL and MPDP are also tested to have a reference with the state-of-the-art.
To run this first test you first need to create the files which contain the polylines:

```shell
cd scripts
python3 makepoint_dps.py <n_points>
```

We usually go for 10, 100, 1000, ... , 1000000 but you can choose freely. Then you have to create the same polylines for MPDP and OMPL by running:

```shell
cd scripts
python3 makepoint_mpdp.py
python3 makepoint_ompl.py
```

Now you can launch the test by running:

```shell
cd tests
python3 test_interpolation.py
```

When finished, the output will be written in ```test_out/interpolation_data.csv``` and it can be visualized by running:

```shell
cd scripts
python3 plot_interpolation.py
```

### Path finding

In this test, we added a map with obstacles to the scenario. Before running DPS we now need to calculate the polyline without crossing any obstacle, which is done by using RRT*. We tested:

- The default RRT* provided by OMPL in combination with DPS
- The RRT-Dubins method provided again by OMPL, which does both the path finding and the interpolation
- Our RRT* implementation which samples the points accordingly with our hypothesis.

We tested 100 randomly chosen pairs of points in 2 different ways:

- With a fixed timeout of 3 seconds
- With 6 different timeouts from 0.05s to 2s

To run this test all you have to do is:
```shell
cd tests
python3 test_timeout.py
```

The results will be inside ```test_out/ompl_survival_3_seconds_fixed.csv``` and ```test_out/ompl_survival_multi_timeout.csv```, which can be displayed by running:

```shell
cd scripts
python3 plot_timeout.py
```

### Roadmap

In this test we consruct a roadmap from a given map, then we sample 10000 pairs of points and we calculate the time that the program takes to find the path for each pair. To run this test you need to provide a map and a tolerance value, which represents the minimum distance that two points must have to be considered two different points.

You can launch this test by running:

```shell
cd tests/rodmap
../../build/tests/roadmap/roadmap tolerance < map/map_file
```

# Documentation

The documentation can be automatically created with doxygen by running:

```shell
cd doc
doxygen .doxcfg 
```

And viewed by opening `html/index.html` on the browser

# Contributors

Pastorelli Patrick (patrick.pastorelli@studenti.unitn.it)

Dagnino Simone (simone.dagnino@studenti.unitn.it)

# Licence

GPL3 licence. Please refer to the LICENCE file for more details.
