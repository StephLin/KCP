# KCP

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg?style=flat)](https://opensource.org/licenses/BSD-3-Clause)
[![Build](https://github.com/StephLin/KCP/actions/workflows/build-deploy.yml/badge.svg)](https://StephLin.github.io/KCP)

The official implementation of KCP: K-Closest Points and Maximum Clique Pruning
for Efficient and Effective 3D Laser Scan Matching, accepted for publication in
the IEEE Robotics and Automation Letters (RA-L).

![](docs/images/snapshot.gif)

KCP is an efficient and effective local point cloud registration approach
targeting for real-world 3D LiDAR scan matching problem. A simple (and naive)
understanding is: **I**CP iteratively considers the closest point of each source
point, but **K**CP considers the **k** closest points of each source point in
the beginning, and outlier correspondences are mainly rejected by the maximum
clique pruning method. KCP is written in **C++** and we also support **Python**
binding of KCP (pykcp).

For more, please refer to our paper:

- Yu-Kai Lin, Wen-Chieh Lin, Chieh-Chih Wang, **K-Closest Points and Maximum Clique Pruning for Efficient and Effective 3-D Laser Scan Matching**. _IEEE Robotics and Automation Letters (RA-L)_, vol.7, no. 2, pp. 1471 -- 1477, Apr. 2022. ([paper](https://doi.org/10.1109/LRA.2021.3140130)) ([preprint](https://gpl.cs.nycu.edu.tw/Steve-Lin/KCP/preprint.pdf)) ([code](https://github.com/StephLin/KCP)) ([video](https://youtu.be/ZaDLEOz_yYc))

If you use this project in your research, please cite:

```bibtex
@article{lin2022kcp,
  title={K-Closest Points and Maximum Clique Pruning for Efficient and Effective 3-D Laser Scan Matching},
  author={Lin, Yu-Kai and Lin, Wen-Chieh and Wang, Chieh-Chih},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={2},
  pages={1471--1477},
  year={2022},
  doi={10.1109/LRA.2021.3140130},
}
```

and if you find this project helpful or interesting, please **:star:Star** the
repository. Thank you!

**Table of Contents**

- [KCP](#kcp)
  - [:package: Resources](#package-resources)
  - [:gear: Installation](#gear-installation)
    - [Step 1. Preparing the Dependencies](#step-1-preparing-the-dependencies)
      - [GCC, CMake, Git, and Eigen3](#gcc-cmake-git-and-eigen3)
      - [nanoflann](#nanoflann)
      - [TEASER++](#teaser)
    - [Step 2. Preparing Dependencies of Python Binding (Optional)](#step-2-preparing-dependencies-of-python-binding-optional)
    - [Step 3. Building KCP](#step-3-building-kcp)
      - [Without Python Binding](#without-python-binding)
      - [With Python Binding](#with-python-binding)
    - [Step 4. Installing KCP to the System (Optional)](#step-4-installing-kcp-to-the-system-optional)
  - [:seedling: Examples](#seedling-examples)
  - [:memo: Some Remarks](#memo-some-remarks)
    - [Tuning Parameters](#tuning-parameters)
    - [Controlling Computational Cost](#controlling-computational-cost)
    - [Torwarding Global Registration Approaches](#torwarding-global-registration-approaches)
  - [:gift: Acknowledgement](#gift-acknowledgement)

## :package: Resources

- [API Documentation](https://stephlin.github.io/KCP)
- [Complete result of the experiment of robustness](https://gpl.cs.nycu.edu.tw/Steve-Lin/KCP/robustness.html)

## :gear: Installation

The project is originally developed in **Ubuntu 18.04**, and the following
instruction supposes that you are using Ubuntu 18.04 as well. I am not sure if
it also works with other Ubuntu versions or other Linux distributions, but maybe
you can give it a try :+1:

Also, please feel free to open an issue if you encounter any problems of the
following instruction.

### Step 1. Preparing the Dependencies

You have to prepare the following packages or libraries used in KCP:

1. A C++ compiler supporting C++14 and OpenMP (e.g. [GCC](https://gcc.gnu.org/) 7.5).
2. [CMake](https://cmake.org/) &#x2265; **3.11**
3. [Git](https://git-scm.com/)
4. [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) &#x2265; 3.3
5. [nanoflann](https://github.com/jlblancoc/nanoflann)
6. [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus) &#x2265; [d79d0c67](https://github.com/MIT-SPARK/TEASER-plusplus/tree/d79d0c67)

#### GCC, CMake, Git, and Eigen3

```bash
sudo apt update
sudo apt install -y g++ build-essential libeigen3-dev git software-properties-common lsb-release

# If you want to obtain newer version of cmake, run the following commands: (Ref: https://apt.kitware.com/)
# wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
# echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
# sudo apt update

sudo apt install cmake
```

#### nanoflann

```bash
cd ~
git clone https://github.com/jlblancoc/nanoflann
cd nanoflann
mkdir build && cd build
cmake .. -DNANOFLANN_BUILD_EXAMPLES=OFF -DNANOFLANN_BUILD_TESTS=OFF
make
sudo make install
```

#### TEASER++

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

### Step 2. Preparing Dependencies of Python Binding (Optional)

The Python binding of KCP (pykcp) uses
[pybind11](https://github.com/pybind/pybind11) to achieve operability between
C++ and Python. KCP will automatically download and compile pybind11 during the
compilation stage. However, you need to prepare a runable Python environment
with header files for the Python C API (python3-dev):

```bash
sudo apt install -y python3 python3-dev
```

### Step 3. Building KCP

Execute the following commands to build KCP:

#### Without Python Binding

```bash
git clone https://github.com/StephLin/KCP
cd KCP
mkdir build && cd build
cmake ..
make
```

#### With Python Binding

```bash
git clone https://github.com/StephLin/KCP
cd KCP
mkdir build && cd build
cmake .. -DKCP_BUILD_PYTHON_BINDING=ON -DPYTHON_EXECUTABLE=$(which python3)
make
```

### Step 4. Installing KCP to the System (Optional)

This will make the KCP library available in the system, and any C++ (CMake)
project can find the package by `find_package(KCP)`. Think twice before you
enter the following command!

```bash
# Under /path/to/KCP/build
sudo make install
```

## :seedling: Examples

We provide two examples (one for C++ and the other for Python 3) These examples
take nuScenes' LiDAR data to perform registration. Please check

- [C++ example with CMake](examples/cpp), and
- [Python example](examples/python)

for more information.

## :memo: Some Remarks

### Tuning Parameters

The major parameters are

- `kcp::KCP::Params::k` and
- `kcp::KCP::Params::teaser::noise_bound`,

where `k` is the number of nearest points of each source point selected to be
part of initial correspondences, and `noise_bound` is the criterion to determine
if a correspondence is correct. In our paper, we suggest `k=2` and `noise_bound`
the 3-sigma (we use `noise_bound=0.06` meters for nuScenes data), and those are
default values in the library.

There is also a boolean parameter to enable debug messages called
`kcp::KCP::Params::verbose` (default: `false`).

To use different parameters to the KCP solver, please refer to the following
snippets:

**C++**

```cpp
#include <kcp/solver.hpp>

auto params = kcp::KCP::Params();

params.k                  = 2;
params.verbose            = false;
params.teaser.noise_bound = 0.06;

auto solver = kcp::KCP(params);
```

**Python**

```python
import pykcp

params = pykcp.KCPParams()
params.k = 2
params.verbose = False
params.teaser.noise_bound = 0.06

solver = pykcp.KCP(params)
```

### Controlling Computational Cost

Instead of
[correspondence-free registration in TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus/issues/120),
KCP considers k closest point correspondences to reduce the major computational
cost of the maximum clique algorithm, and we have expressed the ability for
real-world scenarios without any complicate or learning-based feature descriptor
in the paper. However, it is still possible to encounter computational time or
memory issue if there are too many correspondences fed to the solver.

We suggest controlling your keypoints around 500 for k=2 (in this way the
computational time will be much closer to the one presented in the paper).

### Torwarding Global Registration Approaches

It is promising that KCP can be extended to a global registration approach if a
fast and reliable sparse feature point representation method is employed.

In this way, the role of RANSAC, a fast registration approach usually used in
learning based approaches, is similar to KCP's, but the computation results of
KCP are deterministic, and also, KCP has better theoretical supports.

## :gift: Acknowledgement

This project refers to the computation of the smoothness term defined in
[LOAM](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf)
(implemented in [Tixiao Shan](https://github.com/TixiaoShan)'s excellent project
[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), which is licensed under
BSD-3). We modified the definition of the smoothness term (and it is called the
multi-scale curvature in this project).
