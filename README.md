# osqp-eigen
Simple C++ wrapper for [osqp](http://osqp.readthedocs.io/en/latest/index.html) library.

## 📚 Documentation
The documentation is available online at the accompanying [website](https://robotology.github.io/osqp-eigen).


## 📄 Dependencies
The project depends only on [`osqp`](http://osqp.readthedocs.io/en/latest/index.html) and [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page). Please install [Catch2](https://github.com/catchorg/Catch2)  if you want to run the tests only for testing.

## 📄 OSQP Installation
Please follow this [link](https://osqp.org/docs/get_started/sources.html#install-the-binaries) for OSQP installation.


## 🛠️ Usage

### ⚙️ Build from source (advanced)

1. Clone the repository
   ```
   git clone https://github.com/robotology/osqp-eigen.git
   ```
2. Build it
   ```
   cd osqp-eigen
   mkdir build
   cd build
   cmake -DCMAKE_INSTALL_PREFIX:PATH=<custom-folder> ../
   make
   make install
   ```
3. Add the following environmental variable
   ```
   OsqpEigen_DIR=/path/where/you/installed/
   ```
## 📝 Building the example
   cd example
   mkdir build
   cd build
   cmake ../
   make

   find the mpc_log.csv file inside build
## 🖥️ How to use the library
**osqp-eigen** provides native `CMake` support which allows the library to be easily used in `CMake` projects.
**osqp-eigen** exports a CMake target called `OsqpEigen::OsqpEigen` which can be imported using the `find_package` CMake command and used by calling `target_link_libraries` as in the following example:
```cmake
cmake_minimum_required(VERSION 3.0)
project(myproject)
find_package(OsqpEigen REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example OsqpEigen::OsqpEigen)
```

##  🐛 Bug reports and support
All types of [issues](https://github.com/robotology/osqp-eigen/issues/new) are welcome.

## 📝 License
Materials in this repository are distributed under the following license:


> All software is licensed under the BSD 3-Clause License. See [LICENSE](https://github.com/robotology/osqp-eigen/blob/master/LICENSE) file for details.

> All software is licensed under the BSD 3-Clause License. See [LICENSE](https://github.com/robotology/osqp-eigen/blob/master/LICENSE) file for details.
