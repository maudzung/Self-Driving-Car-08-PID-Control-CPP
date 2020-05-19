# PID Controls

[![CarND](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)

The goal of this project is to implement a PID controller that determines the steering angle in order to 
keep a car in the center of the lane track during driving.

---
## Demostration
![demo](./demo.gif)

The full demostration is at [https://youtu.be/jeNe3Oa1ZVo](https://youtu.be/jeNe3Oa1ZVo)

## Terminologies
- **Cross Track Error (CTE)**: The distance from the car to the trajectory
- **PID**: Proportional-Integral-Derivative
- **P** component: The steering angle is set to proportion of CTE with a proportional factor tau_p.
    ```
    steer_angle = - tau_p * cte
    ```
- **D** component: The differential component of the controller which helps to take temporal derivative of error. 
When the car turned enough to reduce the error, it will help not to overshoot the road.
    ```
    diff_cte = cte - prev_cte
    prev_cte = cte
    steer_angle += - tau_d * diff_cte
    ```
- **I** component: The sum of cte overtime to minimize the average CTE.
    ```
    sum_cte += cte
    steer_angle += tau_i * sum_cte
    ```
- The **PID** controller controls the steering value.
    ```
    cte = robot.y
    diff_cte = cte - prev_cte
    prev_cte = cte
    sum_cte += cte
    steer = -tau_p * cte - tau_d * diff_cte - tau_i * sum_cte
    ```

## PID Fine-Tune
### 1. Manually fine-tune parameters for Kp, Ki, Kd 
- **Step 1**: I set parameters to zeros. Obviously, the car drives straight. 
- **Step 2**: I increased the Kp of the P component with an increment of 0.01 until the car start going on following the road 
and drives with constant oscillations.
- **Step 3**: I increased the Kd of the D component with an increment of 0.1 to try to reduce oscillations. <br>
 *Loop the 2nd step and the 3rd step.*
- **Step 4**: When the car drives the track with really small oscillations and without going out of the road, 
I increased the Ki of the I component to minimize the average CTE.

Finally, I ended up with the parameters: ```Kp = 0.15, Ki = 0.0001, Kd = 1.0```

### 2. Applying **Twiddle** algorithm to optimize the parameters
I also implemented the Twiddle algorithm to optimize the parameters. 
I'll update the results of the algorithm later.

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

## References
1. [PID controller](https://www.wikiwand.com/en/PID_controller#/overview)
