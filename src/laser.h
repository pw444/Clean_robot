//This file is used to define some arguments about lidar sensor

#include "ThisRobot.h"

// Variance of the laser measurements in Grid Squares
#define LOW_VARIANCE (0.017 * MAP_SCALE*MAP_SCALE) 
#define HIGH_VARIANCE (0.17 * MAP_SCALE*MAP_SCALE)

// Set the maximum usuable distance for the laser range finder. 
#define MAX_SENSE_RANGE 7.95 * MAP_SCALE
