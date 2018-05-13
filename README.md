# CarND-Controls-MPC

This is project number 5 of the Self-Driving Car Engineer Nanodegree Program in term 2.

In this project I've implemented Model Predictive Control to drive the car from the simulator around the track. While receiving track waypoints, cars position / angle, I calculate the steering angle and throttle value using MPC! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.


## The Model

MPC uses an optimizer to find the actuator values (steering angle + throttle value) that minimizes a given cost function for the current state of the vehicle.

The model includes the vehicle model and constraints such as actuator limitations:

### Constraints
The actuators can control the steering angle and the throttle value.
The steer angle can be between -25° and 25°. Since this value is reported as -1.0 and 1.0 to the simulators, [-1.0, 1.0] are the contraints.
The throttle value also can be between -1.0 and 1.0. To accelerate fully, the value would be set to 1.0.

### Vehicle model
The vehicles model is defined by following equations which are implemented in the MPC.cpp class from line 121 - 135.

``` c++
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

The cost function references state costs. This includes cross track errors and direction errors, as well as penaltings to high actuator values or high changes of actuator values.
 
The cost function is optimized by tuning some weight values for the several parameters.

``` C++
const double Weight_cte = 3.0;
const double Weight_errorPsi = 1.0;
const double Weight_v = 1.0;
const double Weight_lowActuatorVals = 5.0;
const double Weight_lowAcutatorChanges = 300.0;
```

The solver I used to minimize the cost function is called IPOPT.


## Timestep Length and Elapsed Duration (N & dt)

The MPC is able to calculate control values for a number of timesteps beforehand. N describes the number of steps for which the actuator values are calculated. dt describes the time between two steps. E.g. if N = 10 and dt = 1sec, the actuator prediction for the next 10 seconds would be calculated in one MPC loop. N * dt results in the duration of the trajectory. After each loop, (only) the first value set of the calculated actuator values is applied to the vehicle.

The choosen values for N and dt are
``` c++
size_t N = 10;
double dt = 0.05;
```

This means, the solver tries the find a minimized input control vector for a trajectory for the next 0.5 seconds of driving.

This fits quite good to a velocity of ~70mph. Choosing a longer duration may does not fit to a 3 dimensional polynom for the trajectory. 

There is also a trade-off between the solver calculation time and the resolution defined by the number of trajectory steps 'N'. If N is to high, the solver sometimes does not find a good cost minimized solution at a given time.

An improvement could be, to choose the timestep dt depending on the current velocity.


## Polynomial Fitting and MPC Preprocessing

The simulator passes the next 6 waypoints of the track, depending on the cars current position. A polynomial is fitted to waypoints. Before calculating the coefficents for a 3 dimensional polynomial, the waypoints are transformed from map coordinates to vehicle coordinates (current vehicle coordinate is [x,y,psi] = {0,0,0} ). Both, the current state (including x, y, psi, CTE and psi error) and the coefficients of the waypoint polynominal is processed to the MPC solver.

For each loop cycle:
The initial cross track error is calculated by evaluating the y value at the waypoints polynominal at the current x position (vehicles view) and subtracting the current y position.
Due to starting with psi = 0, the initial error of psi is  -f'(current_x):
```c++
double epsi = - atan(coeffs[1]);
```

## Model Predictive Control with Latency

Setting the actuator values has a latency of 100 millisecond (+ calculation time) before it is applied to the vehicle in the simulator. This latency has to be handled. There are different opportunities to do this.

In the current solution, the application runs the simulation using the vehicle model starting from the current state for the duration of the latency. Applying the kinematic model update equations to the current state given by the simulator, and the latency of 100ms, allows to predict the vehicles state in 100ms. This predicted state is used as the initial state for the MPC solver.

The implementation for this can be found in the main.cpp lines 101 to 106.

Another solution would be, to introduce the latency into the MPC. This could be done by setting the model constraints in that way, that the initial state shall be not changed at all steps within the latency.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Code Style

Code Style sticks to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
