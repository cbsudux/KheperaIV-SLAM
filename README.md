# KheperaIV-SLAM
This repository contains some code I wrote to implement [BreezySLAM](https://github.com/simondlevy/BreezySLAM) on the [KheperaIV](https://www.k-team.com/khepera-iv) robots in [ARGoS](http://www.argos-sim.info/).(A multi-physics robot simulator)

This code allows you to simulate and manually control the KheperaIVs in ARGoS and generate LIDAR and Oodmetry readings as a .dat file. To get a map, you should call log2pgm ( In BreezySLAM ) along with the .dat file.
(This is currently an offline version of BreezySLAM, I will soon create a repo for the online version where the map is created in ARGoS itself)

# Initial Configurations

BreezySLAM is created for the Mines Rover developed by Paris Mines Tech. To adapt it for the Khepera, you need to change a few parameters in log2pgm.cpp. In class Rover, change wheelRadiusMillimeters to 21 mm and halfAxleLengthMillimeters to 52.7 mm ( From the KheperaIV manual )

# BreezySLAM
BreezySLAM can create a map in 3 ways.

1. Only using Odometry
2. Only with a particle filter
3. With Odometry and particle filter

Creating a map only with Odometry is found to produce better results.

# Instructions 

1. Install ARGoS [through source](https://github.com/ilpincy/argos3)
2. Install BreezySLAM (for C++)
3. Create an ARGoS workspace and clone repo into argos_ws and build.
5. Run the controller with 
"$ DRI_PRIME=1 argos3 -c experiments/offline_slam.argos"
and the ARGoS simulator will start with a simulated KheperaIV in an arena. 
6. To select the robot, hold SHIFT and left click on it. 
7. Move the Khepera with i,j,k,l --> i - Move forward && j - Rotate left && l - Rotate right && k - rotate in place ( Anticlockwise )
8. The odometry and LIDAR will be collected in slamData.dat.
9. Copy slamData.dat to the BreezySLAM/examples and run 
"$ ./log2pgm slamData.dat 1"
for the best map  ( Usage : ./log2pgm <dataset> <use_odometry> <random_seed> )
10. A map ( slamData.pgm ) will be created in the examples folder. 

