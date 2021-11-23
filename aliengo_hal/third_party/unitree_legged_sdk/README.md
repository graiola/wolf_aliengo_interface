# Unitree-Legged SDK 

This repository contains a fork of [`unitree_legged_sdk`](https://github.com/unitreerobotics/unitree_legged_sdk).

The unitree_legged_sdk is mainly used for communication between PC and Controller board. It can also be used in other PCs with UDP.

## Maintaining the repository

To update this repository to new updates of Unitree's [`unitree_legged_sdk`](https://github.com/unitreerobotics/unitree_legged_sdk) you need to:

1. Modify the file `include/unitree_legged_sdk/comm.h`, and replace the `C-style` arrays by their array-container equivalent (`std::array`) in all the `structures`. For example:
```cpp
typedef struct
{
std::array<float, 4> quaternion; // Before was: float quaternion[4];
std::array<float, 3> gyroscope; // Before was: float gyroscope[3];
std::array<float, 3> accelerometer; // Before was: float accelerometer[3];
std::array<float, 3> rpy; // Before was: float rpy[3];
} IMU;
```

2. Keep the current `CMakeLists.txt` file or update it if required. Our `CMakeLists.txt` installs the SDK system-wide and generates the corresponding CMake configuration files. You might also need to keep/update the files `cmake/unitreesdkConfig.cmake.in` and `examples/CMakeLists.txt`.

## Dependencies
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)  : This is only required for compiling the examples

```bash
cd lcm-x.x.x
mkdir build
cd build
cmake ../
make
sudo make install
```

## Build
```bash
mkdir build
cd build
cmake ../
make
```

## Usage
You might need to run the examples with 'sudo' for memory locking.


