# Kinematic Control Layer Package

TO DO

## Dependencies

To build and use the `kcl` Package, the following dependencies must be installed on your system:

- **GRAAL Utils**: A collection of software utils and guides.
- **PkgConfig**: A program which helps to configure compiler and linker flags for development libraries

## Installation Instructions

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

