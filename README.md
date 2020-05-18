# PID Controls

[![CarND](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)

The goal of this project is to implement a PID controller that determines the steering angle in order to 
keep a car in the center of the lane track during driving.
---
## Demostration


## PID Fine-Tune


## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Source code structure
The directory structure of this repository is as follows:

```shell script
${ROOT}
├──build.sh
├──clean.sh
├──CMakeLists.txt
├──README.md
├──run.sh
├──src/
    ├──json.hpp
    ├──main.cpp
    ├──PID.cpp
    ├──PID.h
```

## Dependencies
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)

## How to compile and run
1. Download the Term 2 Simulator [here](https://github.com/udacity/self-driving-car-sim/releases).
2. Install `uWebSocketIO`: <br>
This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) 
for either Linux or Mac systems. For windows you can use either Docker, VMware, 
or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)<br>
You can execute the `install-ubuntu.sh` to install uWebSocketIO.
3. Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

```shell script
mkdir build
cd build
cmake ..
make
./pid
```