# Installation

The project is originally developed in **Ubuntu 18.04**, and the following
instruction supposes that you are using Ubuntu 18.04 as well. I am not sure if
it also works with other Ubuntu versions of other Linux distributions, but maybe
you can give it a try :+1:

Also, please feel free to open an issue if you encounter any problems of the
following instruction.

## Step 1. Preparing the Dependencies

You have to prepare the following packages or libraries used in KCP:

1. A C++ compiler supporting C++14 and OpenMP (e.g. [GCC](https://gcc.gnu.org/) 7.5).
2. [CMake](https://cmake.org/) ≥ **3.11**
3. [Git](https://git-scm.com/)
4. [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) ≥ 3.3
5. [nanoflann](https://github.com/jlblancoc/nanoflann)
6. [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus) ≥ [d79d0c67](https://github.com/MIT-SPARK/TEASER-plusplus/tree/d79d0c67)

### GCC, CMake, Git, and Eigen3

```bash
sudo apt update
sudo apt install -y g++ build-essential libeigen3-dev git software-properties-common lsb-release

# If you want to obtain newer version of cmake, run the following commands: (Ref: https://apt.kitware.com/)
# wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
# echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
# sudo apt update

sudo apt install cmake
```

### nanoflann

```bash
cd ~
git clone https://github.com/jlblancoc/nanoflann
cd nanoflann
mkdir build && cd build
cmake .. -DNANOFLANN_BUILD_EXAMPLES=OFF -DNANOFLANN_BUILD_TESTS=OFF
make
sudo make install
```

### TEASER++

```bash
cd ~
git clone https://github.com/MIT-SPARK/TEASER-plusplus
cd TEASER-plusplus
git checkout d79d0c67
mkdir build && cd build
cmake .. -DBUILD_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_DOC=OFF
make
sudo make install
```

## Step 2. Preparing Dependencies of Python Binding (Optional)

The Python binding of KCP (pykcp) uses
[pybind11](https://github.com/pybind/pybind11) to achieve operability between
C++ and Python. KCP will automatically download and compile pybind11 during the
compilation stage. However, you need to prepare a runable Python environment
with header files for the Python C API (python3-dev):

```bash
sudo apt install -y python3 python3-dev
```

## Step 3. Building KCP

Execute the following commands to build KCP:

### Without Python Binding

```bash
git clone https://github.com/StephLin/KCP
cd KCP
mkdir build && cd build
cmake ..
make
```

### With Python Binding

```bash
git clone https://github.com/StephLin/KCP
cd KCP
mkdir build && cd build
cmake .. -DKCP_BUILD_PYTHON_BINDING=ON -DPYTHON_EXECUTABLE=$(which python3)
make
```

## Step 4. Installing KCP to the System (Optional)

This will make the KCP library available in the system, and any C++ (CMake)
project can find the package by `find_package(KCP)`. Think twice before you
enter the following command!

```bash
# Under /path/to/KCP/build
sudo make install
```
