# AUV Core Helper Package

TO DO

## Dependencies

To build and use the `auv_core_helper` Package, the following dependencies must be installed on your system:

- **qpOASES**: An open-source C++ implementation of the online active set strategy for quadratic programming (QP).
- **GRAAL Utils**: A collection of software utils and guides.
- **PkgConfig**: A program which helps to configure compiler and linker flags for development libraries

## Installation Instructions

### Install qpOASES
The auv_core_helper Package depends on the shared library version of qpOASES. Follow these steps to install qpOASES from its GitHub repository.

```bash
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make
sudo make install
```

### Install GRAAL Utils

My advice is to install the full graal_utils libs by creating a graal_ws and running the bash script.

```bash
git clone git@bitbucket.org:isme_robotics/graal_utils.git
cd graal_utils/scripts
bash install_update_graal_libs.sh
```
### Install PkgConfig

Just to use libconfig I am using PkgConfig as i found some pcs find trouble finding libconfig.

```bash
sudo apt-get install pkg-config
```

