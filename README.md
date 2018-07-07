# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

This is my submission for Term2 Project 2 of Udacity's Self Driving Car Nanodegree.

In this project I utilized an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Code Stylesed 

Used [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.
Used Eclipse plugin https://github.com/google/styleguide/blob/gh-pages/eclipse-cpp-google-style.xml to automate the formatting.

## Project Rubric points

### Your code should compile.
The code can be compiled using cmake and make. No changes were made to CMakeLists.txt

### Your px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30]. 
The application,UnscentedKF, provides following RMSE values when run in different modes,

1. Use LASER and RADAR measurements.
./build/UnscentedKF
RMSE: 0.0686 0.0826 0.3364 0.2182

2. Use LASER measurements only.
DISABLE_RADAR=1 ./build/UnscentedKF
RMSE: 0.1713 0.1481 0.6180 0.2607
 
3. Use RADAR measurements only.
DISABLE_LASER=1 ./build/UnscentedKF
RMSE: 0.2113 0.2656 0.3827 0.2946

### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
Code in ukf.cpp follows the general processing flow of Kalaman filters i.e Initialization -> Prediction ->  Update.

### Your Kalman Filter algorithm handles the first measurements appropriately.
State vectors and covariance matrices are initialized properly in ukf.cpp

### Your Kalman Filter algorithm first predicts then updates.
Function ProcessMeasurements() first predicts state/covariance to current timestamps and then updates using RADAR/LASER measurements.

### Your Kalman Filter can handle radar and lidar measurements.
ukf.cpp contains functions UpdateRadar() and UpdateLidar() to handle update step with RADAR/LASER measurments.

### Your algorithm should avoid unnecessary calculations.
The code does not have any unnecessary processing blocks. There could be potential optimizations I have not explored.

### Observation of the results.
1. The RMSE values are significantly lower when using sensor fusion compared to RADAR or LASER alone.
2. The RMSE values for vx and vy are lower for UKF compared to EKF. However I see that px and py RMSE values are higher for UKF compared to EKF when using LASER measurements alone. This could be due to sub optimal tuning of noise values.
3. The initial value for std_a was based on max acceleration observed in the ground truth values (Added a code section under DUMP_GROUND_TRUTH_STATS in tools.cpp to dump these values). The max acceleration observed in x direction was ~2.8 so I chose 1.40 as initial value.
4. The initial value for std_yawdd was set to M_PI/16 assuming M_PI/8 as typical yaw rate as described in "Parameters and Consistency" section in lectures. I played around with different values and then settled on M_PI/4 by looking at NIS values (specially RADAR) and RMSE values.
