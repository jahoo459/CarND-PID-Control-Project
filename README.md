# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Project description

In this project The PID controller is implemented in order to steer the car around the track. The goal of this project is to 
finish the complete lap without driving off the track.

Implemented PID controller consists of three parts:

**P - Proportional factor**
The steering angle is proportional to the CTE with the factor Kp. The larger Kp is, the faster and more aggresive
the system will react to the changes. On the other hand it can easily lead to oscillations and make the system unstable
what results in driving off the road.

_`steering_angle = -Kp * cte`_

**I - Integrating factor**
The integral factor Ki removes the bias in the PID controller. It works on a sum of errors from the past. It main
goal is to prevent the PD controller to steer the car with a drift from a center line.

_`total_cte =+ cte;
steering_angle += -Ki * cte`_

**D - Differential factor** 
The differential factor Kd takes the derivative of the error (cte - prev_cte) to the calculations. Thanks to that it prevents
overshooting and reduces oscillations.

_`steering_angle = -Kd * (cte - prev_cte)`_

**The complete PID model equation**
The complete model of a PID controller is implemented as follows:
_`steering_angle = (-Kp * p_error) - (Kd * d_error) - (Ki * i_error)`_

**Parameter tuning**
The parameter tuning was done manually in a following way;
1. The I and D factors were set to 0, the P factor was increased till getting oscillations (0.5)
2. The D factor was increased in order to remove the oscillations (1.5)
3. As I did not see the 'bias' effect in the simulator I set the I factor to a very low number (0.0001). I noticed, that
it didn't really have an influence on the drive.
4. With those parameters the car was driving smootly on the track but it had some problems with taking the sharp turns
with high speed. Because of that I increased the P factor to 0.07 and D to 1.8. Additionally I increased the I factor to 0.005.
It turned out that increasing the I factor improved the behaviour on the curves but also introduced some oscillation 
to the path.

With this set of parameters the car is able to drive the whole lap without driving off the street.







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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

