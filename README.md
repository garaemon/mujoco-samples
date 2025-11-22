# MuJoCo Samples

This repository contains sample programs for MuJoCo physics simulation engine.

## Samples

- [01-simple-cube](01-simple-cube/): A simple example showing a cube with a free joint falling under gravity

## Requirements

- CMake >= 2.8.3
- MuJoCo
- GLFW3
- OpenGL

## Building

Each sample directory contains its own CMakeLists.txt. To build a sample:

```bash
cd 01-simple-cube
mkdir build
cd build
cmake ..
make
```

## Running

After building, you can run the executable from the build directory:

```bash
./main
```
