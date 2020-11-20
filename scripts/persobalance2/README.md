# PersoBalance2

The `persobalance2.dll` library exposes C functions for solving MCLP
and acyclic Cholesky factorization. The library itself is written in
C++11 and uses the Eigen and Boost libraries (header-only).

The library is meant to be used from Python using ctypes. See the
corresponding python files (bin/persobalance/mlcp.py,
bin/persobalance/graph.py) for python wrappers and example uses.

# Requirements

- [cmake]](https://cmake.org/) version >= 3.1
- [eigen3](http://eigen.tuxfamily.org/)
- [boost](http://www.boost.org/)

# Building

Use cmake to generate build files:

  cmake .

To generate Visual Studio projects, do:

  cmake -G 'Visual Studio 14 2015 Win64' .

Please refer to the CMake documentation for more information.


