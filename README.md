# Cartographer

## Purpose

[Cartographer](https://github.com/cartographer-project/cartographer) is a system that provides real-time simultaneous localization
and mapping [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) in 2D and 3D across multiple platforms and sensor
configurations.

## Getting started

Learn to use Cartographer at [the official Docs site](https://google-cartographer.readthedocs.io)

##  Viam Specific Instructions

### Raspberry Pi (Or other Debian 11+, including canon-shell)

```bash
sudo apt install cmake \
ninja-build \
libgmock-dev \
libboost-iostreams-dev \
liblua5.3-dev \
libcairo2-dev \
python3-sphinx \
libabsl-dev \
libceres-dev \
libprotobuf-dev \
protobuf-compiler \
libpcl-dev
```

### Cartographer

```bash
git clone git@github.com:cartographer-project/cartographer.git (change to slam viamrobotics?)
mkdir cartographer/build && cd cartographer/build
cmake .. -G Ninja -DCMAKE_CXX_STANDARD=17
ninja
```

### Example script for running cartographer
You can view an example of how to run the compiled main file in [run_carto_main.sh](./run_carto_main.sh).

You can run the script by executing: `./run_carto_main.sh`.
