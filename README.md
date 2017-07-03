# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## About the Project
In this project a Model Predictive Control for a self driving car is implemented. As the name suggests, the goal is to build a model that closely mimics the kinematic properties of the car as it moves. Using this model, the car can be actuated to run as close to a predefine path as possible. The model depends on such properties of the car as `speed`, `acceleration`, `x and y coordinates`, `center of gravity`, `steering angle` etc. Given a predefined path, this method attempts to fit a `local path` on which the car runs for a period of time until the next time step.

## Kinematic Model
The current state of the car is defined by a six variable vector:

1. x position(`x_t`)
2. y position (`y_t`)
3. orientation (`psi_t`)
4. velocity (`v_t`)
5. cross track error (`cte_t`)
6. orientations error (`epsi_t`)

The actuation of the vehicle is controlled by the throttle and the steering angle. Let these variables be 'a' and 'delta' respectively.

at the next state, we predict that the state of the car should be as follows:
1. new x position (`x_t1`) = `x_t + v_t * cos(psi_t)`
2. new y position (`y_t1`) = `y_t + v_t * sin(psi_t)`
3. new orientation (`psi_t1`) = `(psi_t + v_t * delta / Lf * dt)` where `Lf` is the distance from the front wheels of the car to the center of gravity and `dt` is the elapsed time between actuations
4. new velocity (`v_t1`) = `v_t + a* dt`

A section of the path is delivered to the car and with this, we fit a function to the path in terms of x and y coordinates. The function fit to the path is used to calculate the new cross track error and orientation error. If the fit function at the current time is `f_t`, The desired orientation `psi_des` is the tan inverse of the derivative of `f_t` 

5. new cross track error (`cte_t1`) = `(f_t - y_t) + (v_t * sin(epsi_t)*dt)`
6. new orientation error (`epsi_t1`) = `(psi_t + psi_des)-(v_t * delta_t/Lf * dt)`

Using what we know about the current state and what we expect the next state to be, we can project the actuations(acceleration and steering angle) that can produce the best path that follows the provided path

The task to obtain the best path will can be determined by optimization based on constraints placed on the kinematic model. The constraints used(without equations) are listed below

1. The car's new cross track error should be as close as possible to the reference cross track error
2. The car should try to maintain its velocity at the reference velocity
3. The car new orientation error should be as close as possible to the reference orientation error
4. The car should mute the acceleration and steering angle as much as possible but not too much to impair the motion of the car
5. The car should try to keep changes actuation as little as possible but should be responsive enough 

The optimization returns the next state and actuation variables, these are used actuate the car in the next step

## Latency
Given that there may be a delay between when the car's new actuation values are delivered to the wheels. In the simulator, there is a 100 ms delay that simulates the latency. In this system, we pick the actuation values that are expected to occur at the current time + 100 ms



## Timeestep Length and Elapsed Duration
The chosen time duration is 1 second. The number of steps is chosen as 10 and the timestep is chosen as 0.1s.

## Results
We are able to drive the car around the track at an average speed of 38mph with set reference velocity of 70mph 



