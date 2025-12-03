# Free Hinge Sample

This sample demonstrates a basic MuJoCo simulation with a cube that has a free hinge joint.
The cube swings like a pendulum.

It uses the same main.cpp from ../01-simple-cube.

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Running

From the build directory:

```bash
./main
```

## Controls

- **Left Mouse Button**: Rotate camera (hold and drag)
  - With Shift: Rotate horizontally
- **Right Mouse Button**: Move camera (hold and drag)
  - With Shift: Move horizontally
  - Without Shift: Move vertically
- **Middle Mouse Button**: Zoom (hold and drag)

## Files

- `main.cpp`: Main application with OpenGL rendering and GLFW window management
- `model.xml`: MuJoCo model definition (MJCF format)
- `CMakeLists.txt`: Build configuration
