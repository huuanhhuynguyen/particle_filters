# Particle Filter 

This project is based on the template [code](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project) 
within the scope of the localization course in Udacity Self-Driving Car Nanodegree.

<a href="https://www.youtube.com/watch?v=qKuo6CHQeHk" target="_blank">
<img src="demo.gif" alt="circuit" width="500" height="341"/></a>

## Project Introduction
A robot has been kidnapped and transported to a new location! Luckily it has a 
map of this location, a (noisy) GPS estimate of its initial location, and lots 
of (noisy) sensor and control data.

In this project a two-dimensional particle filter is implemented in C++. The 
particle filter will be given a map and some initial localization information 
(analogous to what a GPS would provide). At each time step your filter will also
 get observation and control data.

## Running the Code
1. Download Term 2 Simulator [here](https://github.com/udacity/self-driving-car-sim/releases) and launch it.
2. Install uWebSocketIO:
- This repository includes two bash files for installing uWebSocketIO on Linux or Mac systems.
- For Windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.
3. Navigate to root `cd particle_filters`
4. Build
```
$ mkdir build && cd build
$ cmake .. && make
$ ./particle_filter
```
Alternatively, run:
```
$ ./clean.sh
$ ./build.sh
$ ./run.sh
```

# Structure

```
.particle_filters           # project root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data                    # map data input to the filter
|   |   map_data.txt
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

`map_data.txt` has three columns [x, y, landmark_id].
