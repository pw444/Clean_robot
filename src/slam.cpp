// This is the main file for SLAM
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <math.h>
#include <strings.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "high.h"
#include "mt-rand.h"

// The initial seed used for the random number generated 
#define SEED 1

// Default names for printing map files.

#define MAP_PATH_NAME "map"
#define PARTICLES_PATH_NAME "particles"


int VIDEO = 10;
double RotationSpeed, TranslationSpeed;
int continueSlam;
int PLAYBACK_COMPLETE = 0;



//
// InitializeRobot
//
// Calls the routines in 'ThisRobot.c' to initialize the necessary hardware and software.
//
//
int InitializeRobot(int argc, char *argv[]) {
  if (InitializeThisRobot(argc, argv) == -1) {
    fprintf(stderr, "Start up initialization of the robot has failed.\n");
    return -1;
  }

  fprintf(stderr, "Connect Odometry.\n");
  if (ConnectOdometry(argc, argv) == -1) {
    fprintf(stderr, "Unable to connect to the robot's odometry.\n");
    return -1;
  }

  fprintf(stderr, "Connect Laser.\n");
  if (ConnectLaser(argc, argv) == -1) {
    fprintf(stderr, "Unable to connect to the robot's laser.\n");
    return -1;
  }

  fprintf(stderr, "Connect Drive.\n");
  if (ConnectDrive(argc, argv) == -1) {
    fprintf(stderr, "Unable to connect to the robot's drive motors.\n");
    return -1;
  }

  return 0;
}


//
// WriteLog
//
// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This function is used for debug
//
void WriteLog(FILE *logFile, TSense sense) 
{ 
  int i;

  fprintf(logFile, "Odometry %.6f %.6f %.6f \n", odometry.x, odometry.y, odometry.theta);
  fprintf(logFile, "Laser %d ", SENSE_NUMBER);
  for (i = 0; i < SENSE_NUMBER; i++)
    fprintf(logFile, "%.6f ", sense[i].distance/MAP_SCALE);
  fprintf(logFile, "\n");
}
 


//This is the SLAM thread call
void *Slam(void *a)
{
  TPath *path, *trashPath;
  TSenseLog *obs, *trashObs;

  InitHighSlam();
  InitLowSlam();

  while (continueSlam) {
    LowSlam(continueSlam, &path, &obs);
    HighSlam(path, obs);

    while (path != NULL) {
      trashPath = path;
      path = path->next;
      free(trashPath);
    }
    while (obs != NULL) {
      trashObs = obs;
      obs = obs->next;
      free(trashObs);
    }
  }

  CloseLowSlam();
  return NULL;
}


int main (int argc, char *argv[])
{
  int x;
  int WANDER, EXPLORE, DIRECT_COMMAND;
  pthread_t slam_thread;
    
  RECORDING = "";
  PLAYBACK = "";
  for (x = 1; x < argc; x++) {
    if (!strncmp(argv[x], "-R", 2))
      RECORDING = "current.log";
    if (!strncmp(argv[x], "-r", 2)) {
      x++;
      RECORDING = argv[x];
    }
    else if (!strncmp(argv[x], "-p", 2)) {
      x++;
      PLAYBACK = argv[x];
    }
    else if (!strncmp(argv[x], "-P", 2))
      PLAYBACK = "current.log";
  }

  fprintf(stderr, "********** Localization Example *************\n");
  if (PLAYBACK == "")
    if (InitializeRobot(argc, argv) == -1)
      return -1;

  fprintf(stderr, "********** World Initialization ***********\n");

  seedMT(SEED);
  continueSlam = 1;
  pthread_create(&slam_thread, (pthread_attr_t *) NULL, Slam, &x);

  fprintf(stderr, "*********** Main Loop (Movement) **********\n");

  WANDER = 0;
  EXPLORE = 0;
  DIRECT_COMMAND = 0;
  RotationSpeed = 0.0;
  TranslationSpeed = 0.0;

  pthread_join(slam_thread, NULL);
  return 0;
}

