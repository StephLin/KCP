# C++ example with CMake

This is a C++ example with CMake management. Note that this example requires the
[Point Cloud Library (PCL) &#x2265; 1.8](https://pointclouds.org/) for reading
`.pcd` files. You can install the library by the following command:

```bash
sudo apt install -y libpcl-dev
```

## :gear: Building

To build the project, please choose a proper one and execute the corresponding
script:

### Using `add_subdirectory` (Default)

- Pros: You don't need to install the KCP library (libkcp) to the system.
- Cons: You have to set a path to the KCP repository, and you need to re-compile
  KCP for each project.

```bash
mkdir build && cd build
cmake ..
make
```

### Using `find_package`

- Pros: You can import the KCP library anywhere, without the path to the KCP
  repository. Moreover, you can re-use the shared library.
- Cons: You have to install the KCP library (libkcp) to the system.

```bash
mkdir build && cd build
cmake .. -DFIND_KCP_FROM_SYSTEM=ON
make
```

## :running: Execution

```bash
./main
```

The main function will read two point clouds from [data](../data), and perform
the scan matching using KCP.
