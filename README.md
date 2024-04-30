# Advanced Kalman Filtering and Sensor Fusion Simulation #

This project is an advanced Kalman Filtering and Sensor Fusion Simulation exercise, developed by Dr. Steven Dumble for the Technitute Course on Advanced Kalman Filtering and Sensor Fusion.

Link to the original github repository can be found [here](https://github.com/technitute/AKFSF-Simulation-CPP)

In this project, different types of Kalman Filters which are used to estimate the navigation state of a 2D vehicle problem; such as that would be found on a self-driving car!


![AKFSF-Simulation](/AKFSF-Simulation.gif)


This README is broken down into the following sections:

- [Setup](#setup) -  Environment and code setup required to get started and a brief overview of the project structure.
 - [The Tasks](#the-tasks) - Tasks you will need to complete for the project

 ## Setup ##

This project will use the Ubuntu 64 20.04.2.0 LTS  C++ development environment that is setup for this course. Follow the steps below to compile the simulation.

 - Install the dependencies
 ```
 sudo apt install libeigen3-dev libsdl2-dev libsdl2-ttf-dev
 ```
 
 - Clone the repository
 ```
 git clone https://github.com/phanikiran1169/SensorFusion
 ```
 - Setup the cmake build
 ```
 cd SensorFusion
 mkdir build
 cd build
 cmake ../
 ```

 -  Compile the code
 ```
 make
 ```
 
 -  You should now be able to and run the estimation simulator
 ```
 ./SensorFusion
 ```

## Simulation Operation ##
The simulation can be run with different motion and sensor profiles to test the different scenarios and evaluate how the filter performs with different conditions. The different profiles can be activated pressing the number keys 1-9,0, to active the corresponding profile.

### Motion Profiles ###
* 1 - Constant Velocity + GPS + GYRO + Zero Initial Conditions
* 2 - Constant Velocity + GPS + GYRO + Non-zero Initial Conditions
* 3 - Constant Speed Profile + GPS + GYRO
* 4 - Variable Speed Profile + GPS + GYRO
* 5 - Constant Velocity + GPS + GYRO + LIDAR+ Zero Initial Conditions
* 6 - Constant Velocity + GPS + GYRO + LIDAR + Non-zero Initial Conditions
* 7 - Constant Speed Profile + GPS + GYRO + LIDAR
* 8 - Variable Speed Profile + GPS + GYRO + LIDAR
* 9 - CAPSTONE
* 0 - CAPSTONE BONUS (with No Lidar Data Association)



## The Tasks ##
The tasks for this simulator can be broken down into 4 different areas:
1. Linear Kalman Filter
2. Extended Kalman Filter
3. Unscented Kalman Filter
4. Capstone Project

### Linear Kalman Filter (LKF Exercise 1)
 Initialize the filter on first prediction step and then implement the 2D Vehicle process model and Linear Kalman Filter Prediction steps. Test and verify good performance with profile 1.

### Linear Kalman Filter (LKF Exercise 2)
Continuing on from Exercise 2, Implement the GPS update step using the Linear Kalman Filter Update equations. Test and verify good performance with profile 1. Play around with tunings and see results for profiles 2,3 and 4.

### Linear Kalman Filter (LKF Exercise 3)
Continuing on from Exercise 3, Modify the filter to initialize the filter on first GPS measurement rather than prediction. Use the flag ```INIT_ON_FIRST_PREDICTION = false``` to enable this functionality. Test and verify good performance with profile 1 and 2.

### Extended Kalman Filter (EKF Exercise 1)
Implement the 2D Vehicle process model and Extended Kalman Filter Prediction steps. Test and verify good performance with profiles 1-4.

### Extended Kalman Filter (EKF Exercise 2)
Continuing on from EKF Exercise 2, Implement the LIDAR update step using the Extended Kalman Filter Update equations. Test and verify good performance with profiles 1-8.

### Unscented Kalman Filter (UKF Exercise 1)
Implement the 2D Vehicle Process model and Unscented Kalman Filter Prediction steps. Test and verify good performance with profiles 1-4.

### Unscented Kalman Filter (UKF Exercise 2)
Continuing on from UKF Exercise 2, Implement the LIDAR update step using the Unscented Kalman Filter Update equations. Test and verify good performance with profiles 1-8.

### Capstone Project
Program a filter to provide the best estimation performance for profiles 9,0 (It should also work on any other profiles as well).
