//This file defines the whole process of SLAM in low level
#include <string.h>
#include "low.h"
#include "mt-rand.h"

struct THold {
  TSense sense;
  double C, D, T;
};

// Threshold for culling particles. 
#define THRESH 13.0

// Number of passes to use to cull good particles
#define PASSES 9

// Maximum error we will allow for a trace in grid squares. 
#define MAX_TRACE_ERROR exp(-24.0/LOW_VARIANCE)
// A constant used for culling in Localize
#define WORST_POSSIBLE -10000

// Used for recognizing the format of some data logs.
#define LOG 0
#define REC 1
int FILE_FORMAT;
FILE *readFile;


struct TSample_struct {
   // The position of the sample, with theta being the facing angle of the robot.
  double x, y, theta; 
  // The arguments used for error motion model
  double C, D, T;
   // The current probability of this sample, given the observations and this sample's map
  double probability;
   // The index into the array of particles
  int parent;
};
typedef struct TSample_struct TSample;

 // The number of iterations between writing out the map as a png. 0 is off.
int L_VIDEO = 0;

 // Every particle needs a unique ID number. This stack keeps track of the unused IDs.
int cleanID;
int availableID[ID_NUMBER];


TSample newSample[SAMPLE_NUMBER];
double lastX, lastY, lastTheta;
 // No. of children each particle gets for resampling
int children[PARTICLE_NUMBER];

//reserved space for reasmpling
TParticle savedParticle[PARTICLE_NUMBER];
int cur_saved_particles_used;

 // Keeps track of what iteration the SLAM process is currently on.
int curGeneration;
 // Stores the most recent set of laser observations.
TSense sense;
unsigned char map[MAP_WIDTH][MAP_HEIGHT];
THold hold[LOW_DURATION];



void AddToWorldModel(TSense sense, int particleNum) {
  int j;

  // Run through each point that the laser found an obstruction at
  for (j=0; j < SENSE_NUMBER; j++) 
    // Normalize readings relative to the pose of current assumed position
    LowAddTrace(l_particle[particleNum].x, l_particle[particleNum].y, sense[j].distance, (sense[j].theta + l_particle[particleNum].theta), 
		l_particle[particleNum].ancestryNode->ID, (sense[j].distance < MAX_SENSE_RANGE));
}



//
// CheckScore
//
// Determine the fitness of a laser endpoint
inline double CheckScore(TSense sense, int index, int sampleNum) 
{
  double a;

  a = LowLineTrace(newSample[sampleNum].x, newSample[sampleNum].y, (sense[index].theta + newSample[sampleNum].theta), 
		   sense[index].distance, l_particle[ newSample[sampleNum].parent ].ancestryNode->ID, 0);
  return MAX(MAX_TRACE_ERROR, a);
}



//
// Localize
//
// This is where the bulk of evaluating and resampling the particles takes place. 
// Also applies the motion model
//
void Localize(TSense sense)
{
  double ftemp; 
  double threshold;  // threshhold for discarding particles (in log prob.)
  double total; 
  double turn, distance, moveAngle; // The incremental motion reported by the odometer
  double CCenter, DCenter, TCenter, CCoeff, DCoeff, TCoeff;
  double tempC, tempD;  // Temporary variables for the motion model. 
  int i, j, k, p, best;  // Incremental counters.
  int keepers = 0; // How many particles finish all rounds
  int newchildren[SAMPLE_NUMBER]; // Used for resampling
  
  // Take the odometry readings from both this time step and the last, in order to figure out
  // the base level of incremental motion. Convert our measurements from meters and degrees 
  // into terms of map squares and radians
  distance = sqrt( ((odometry.x - lastX) * (odometry.x - lastX)) 
		 + ((odometry.y - lastY) * (odometry.y - lastY)) ) * MAP_SCALE;
  turn = (odometry.theta - lastTheta);

  // Keep motion bounded between pi and -pi
  if (turn > M_PI/3)
    turn = turn - 2*M_PI;
  else if (turn < -M_PI/3)
    turn = turn + 2*M_PI;


  i = 0;
  // Iterate through each of the old particles, to see how many times it got resampled.
  for (j = 0; j < PARTICLE_NUMBER; j++) {
    // Now create a new sample for each time this particle got resampled 
    for (k=0; k < children[j]; k++) {
      newSample[i].parent = j;
      tempC = 0;
      tempD = 0;
      double tempT = 0;

      newSample[i].C = tempC;
      newSample[i].D = tempD;
      newSample[i].T = tempT;
      newSample[i].theta = l_particle[j].theta + turn;

      // Assuming that the robot turned continuously throughout the time step, 
      moveAngle = (newSample[i].theta + l_particle[j].theta)/2.0;

      newSample[i].x = l_particle[j].x + distance * cos(moveAngle);
      newSample[i].y = l_particle[j].y + distance * sin(moveAngle);
      newSample[i].probability = 0.0;
      i++;
    }
  }

  // Go through these particles in a number of passes, in order to find the best particles. 
  threshold = WORST_POSSIBLE-1;  // ensures that we accept anything in 1st round
  for (p = 0; p < PASSES; p++){
    best = 0;
    for (i = 0; i < SAMPLE_NUMBER; i++) {
      if (newSample[i].probability >= threshold) {
	for (k = p; k < SENSE_NUMBER; k += PASSES) 
	  newSample[i].probability = newSample[i].probability + log(QuickScore(sense, k, i)); 
	if (newSample[i].probability > newSample[best].probability) 
	  best = i;
      }
      else 
	newSample[i].probability = WORST_POSSIBLE;
    }
    threshold = newSample[best].probability - THRESH;
  }

  keepers = 0;
  for (i = 0; i < SAMPLE_NUMBER; i++) {
    if (newSample[i].probability >= threshold) {
      keepers++;
      newSample[i].probability = 0.0;
    }
    else
      newSample[i].probability = WORST_POSSIBLE;
  }

  
  fprintf(stderr, "Better %d ", keepers);
  threshold = -1;

  // Now reevaluate all of the surviving samples, using the full laser scan to look for possible
  // obstructions, in order to get the most accurate weights. 
  keepers = 0;
  for (p = 0; p < PASSES; p++){
    best = 0;
    for (i = 0; i < SAMPLE_NUMBER; i++) {
      if (newSample[i].probability >= threshold) {
	if (p == PASSES -1)
	  keepers++;
	for (k = p; k < SENSE_NUMBER; k += PASSES) 
	  newSample[i].probability = newSample[i].probability + log(CheckScore(sense, k, i)); 
	if (newSample[i].probability > newSample[best].probability) 
	  best = i;
      }
      else 
	newSample[i].probability = WORST_POSSIBLE;
    }
    threshold = newSample[best].probability - THRESH; 
  }

  // Report how many samples survived the second cut. 
  fprintf(stderr, "Best of %d ", keepers);

  total = 0.0;
  threshold = newSample[best].probability;
  for (i = 0; i < SAMPLE_NUMBER; i++) {
    // If the sample was culled, it has a weight of 0
    if (newSample[i].probability == WORST_POSSIBLE)
      newSample[i].probability = 0.0;
    else {
      newSample[i].probability = exp(newSample[i].probability-threshold);
      total = total + newSample[i].probability;
    }
  }

  // Renormalize to ensure that the total probability is now equal to 1.
  for (i=0; i < SAMPLE_NUMBER; i++)
    newSample[i].probability = newSample[i].probability/total;

  total = 0.0;
  // Count how many children each particle will get in next generation
  // This is done through random resampling.
  for (i = 0; i < SAMPLE_NUMBER; i++) {
    newchildren[i] = 0;
    total = total + newSample[i].probability;
  }

  i = j = 0;  // i = no. of survivors, j = no. of new samples
  while ((j < SAMPLE_NUMBER) && (i < PARTICLE_NUMBER)) {
    k = 0;
    ftemp = MTrandDec()*total;
    while (ftemp > (newSample[k].probability)) {
      ftemp = ftemp - newSample[k].probability;
      k++;
    }    
    if (newchildren[k] == 0)
      i++;
    newchildren[k]++;
    j++;
  }

  // Report exactly how many samples are kept as particles, since they were actually
  // resampled.
  fprintf(stderr, "(%d kept) ", i);

  // Do some cleaning up
  for (i = 0; i < PARTICLE_NUMBER; i++) {
    children[i] = 0;
    savedParticle[i].probability = 0.0;
  }

  // Now copy over new particles to savedParticles
  best = 0;
  k = 0; // pointer into saved particles
  for (i = 0; i < SAMPLE_NUMBER; i++)
    if (newchildren[i] > 0) {
      savedParticle[k].probability = newSample[i].probability;
      savedParticle[k].x = newSample[i].x;
      savedParticle[k].y = newSample[i].y;
      savedParticle[k].theta = newSample[i].theta;
      savedParticle[k].C = newSample[i].C;
      savedParticle[k].D = newSample[i].D;
      savedParticle[k].T = newSample[i].T;
      savedParticle[k].ancestryNode = l_particle[ newSample[i].parent ].ancestryNode;
      savedParticle[k].ancestryNode->numChildren++;
      children[k] = newchildren[i];

      if (savedParticle[k].probability > savedParticle[best].probability) 
	best = k;

      k++;
    }

  cur_saved_particles_used = k;

  if (j < SAMPLE_NUMBER) {
    total = 0.0;
    // Normalize particle probabilities. Note that they have already been exponentiated
    for (i = 0; i < cur_saved_particles_used; i++) 
      total = total + savedParticle[i].probability;

    for (i=0; i < cur_saved_particles_used; i++)
      savedParticle[i].probability = savedParticle[i].probability/total;

    total = 0.0;
    for (i = 0; i < cur_saved_particles_used; i++) 
      total = total + savedParticle[i].probability;

    while (j < SAMPLE_NUMBER) {
      k = 0;
      ftemp = MTrandDec()*total;
      while (ftemp > (savedParticle[k].probability)) {
	ftemp = ftemp - savedParticle[k].probability;
	k++;
      }    
      children[k]++;

      j++;
    }
  }

  // Some useful information concerning the current generation of particles, and the parameters for the best one.
  fprintf(stderr, "-- %.3d (%.4f, %.4f, %.4f) : %.4f\n", curGeneration, savedParticle[best].x, savedParticle[best].y, 
	  savedParticle[best].theta, savedParticle[best].probability);
}



//
// DisposeAncestry
//
// When the SLAM process is complete, this function will clean up the memory being used by the ancestry
// tree, and remove all of the associated entries from the low level map.
//
void DisposeAncestry(TAncestor particleID[]) 
{
  int i, j;
  TPath *tempPath, *trashPath;
  TEntryList *entry;

  for (i = 0; i < ID_NUMBER; i++) {
    if (particleID[i].ID == i) {
      // Free up memory
      entry = particleID[i].mapEntries;
      for (j=0; j < particleID[i].total; j++)
	LowDeleteObservation(entry[j].x, entry[j].y, entry[j].node);
      free(entry);
      particleID[i].mapEntries = NULL;
      
      tempPath = particleID[i].path;
      while (tempPath != NULL) {
	trashPath = tempPath;
	tempPath = tempPath->next;
	free(trashPath);
      }
      particleID[i].path = NULL;

      particleID[i].ID = -123;
    }

    for (cleanID=0; cleanID < ID_NUMBER; cleanID++)
      availableID[cleanID] = cleanID;
    cleanID = ID_NUMBER;
  }
}



//
// UpdateAncestry
//
// Every iteration, after particles have been resampled and evaluated (ie after Localize has been run),
// we will need to update the ancestry tree. This consists of four main steps.
// a) Remove dead nodes. 
// b) Collapse branches of the tree. 
// c) Add the new particles into the tree. 
// d) Update the map for each new particle. 
//
void UpdateAncestry(TSense sense, TAncestor particleID[])
{
  int i, j;
  TAncestor *temp, *hold, *parentNode;
  TEntryList *entry, *workArray;
  TMapStarter *node;
  TPath *tempPath, *trashPath;

  // Remove Dead Nodes - 
  for (i=0; i < l_cur_particles_used; i++) {
    temp = l_particle[i].ancestryNode;

    // This is a "while" loop for purposes of recursing up the tree.
    while (temp->numChildren == 0) {
      // Free up the memory in the map by deleting the associated observations.
      for (j=0; j < temp->total; j++) 
	LowDeleteObservation(temp->mapEntries[j].x, temp->mapEntries[j].y, temp->mapEntries[j].node);

      // Get rid of the memory being used by this ancestor
      free(temp->mapEntries);
      temp->mapEntries = NULL;
      tempPath = temp->path;
      while (tempPath != NULL) {
	trashPath = tempPath;
	tempPath = tempPath->next;
	free(trashPath);
      }
      temp->path = NULL;

      // Recover the ID
      cleanID++;
      availableID[cleanID] = temp->ID;
      temp->generation = curGeneration;
      temp->ID = -42;

      // Remove this node from the tree, while keeping track of its parent
      hold = temp;
      temp = temp->parent;
      hold->parent = NULL;

      // Note the disappearance of this particle (may cause telescoping of particles, or outright deletion)
      temp->numChildren--;
    }
  }

  // Collapse Branches -
  for (i = 0; i < ID_NUMBER-1; i++) {
    if ((particleID[i].ID == i) && (particleID[i].parent != NULL) && (particleID[i].parent->numChildren == 1)) {
      while (particleID[i].parent->generation == -111)
	particleID[i].parent = particleID[i].parent->parent;

      parentNode = particleID[i].parent;

      // Check to make sure that the parent's array is large enough to accomadate all of the entries of the child
      if (parentNode->size < (parentNode->total + particleID[i].total)) {
	parentNode->size = (int)(ceil((parentNode->size + particleID[i].size)*1.75));
	workArray = (TEntryList *)malloc(sizeof(TEntryList)*parentNode->size);
	if (workArray == NULL) fprintf(stderr, "Malloc failed for workArray\n");

	for (j=0; j < parentNode->total; j++) {
	  workArray[j].x = parentNode->mapEntries[j].x;
	  workArray[j].y = parentNode->mapEntries[j].y;
	  workArray[j].node = parentNode->mapEntries[j].node;
	}
	free(parentNode->mapEntries);
	parentNode->mapEntries = workArray;
      }

      // Change all map entries of the parent to have the ID of the child
      entry = particleID[i].mapEntries;
      for (j=0; j < particleID[i].total; j++) {
	node = lowMap[entry[j].x][entry[j].y];

	// Change the ID
	node->array[entry[j].node].ID = parentNode->ID;
	node->array[entry[j].node].source = parentNode->total;

	parentNode->mapEntries[parentNode->total].x = entry[j].x;
	parentNode->mapEntries[parentNode->total].y = entry[j].y;
	parentNode->mapEntries[parentNode->total].node = entry[j].node;
	parentNode->total++;

	// Check for pre-existing observation in the parent's list
	if (node->array[entry[j].node].parentGen >= parentNode->generation) {
	  node->array[entry[j].node].parentGen = -1;
	  node->dead++;
	}
      }
      for (j=0; j < particleID[i].total; j++) {
	node = lowMap[entry[j].x][entry[j].y];
	if ((node->total - node->dead)*2.5 < node->size) 
	  LowResizeArray(node, -7);
      }

      free(particleID[i].mapEntries);
      particleID[i].mapEntries = NULL;

      // Inherit the path
      tempPath = parentNode->path;
      while (tempPath->next != NULL) 
	tempPath = tempPath->next;
      tempPath->next = particleID[i].path;
      particleID[i].path = NULL;

      // Inherit the number of children
      parentNode->numChildren = particleID[i].numChildren;
      particleID[i].generation = -111;
    }
  }

  for (i=0; i < ID_NUMBER-1; i++) 
    if (particleID[i].ID == i) {
      while (particleID[i].parent->generation == -111) 
	particleID[i].parent = particleID[i].parent->parent;
    }

  // Wipe the slate
  LowInitializeFlags();


  // Add the current savedParticles into the ancestry tree, and copy them over into the 'real' particle array
  j = 0;
  for (i = 0; i < cur_saved_particles_used; i++) {
    while (savedParticle[i].ancestryNode->generation == -111) 
      savedParticle[i].ancestryNode = savedParticle[i].ancestryNode->parent;

    if (savedParticle[i].ancestryNode->numChildren == 1) {
      // Change the generation of the node
      savedParticle[i].ancestryNode->generation = curGeneration;
      savedParticle[i].ancestryNode->numChildren = 0;
      l_particle[j].ancestryNode = savedParticle[i].ancestryNode;

      // Add a new entry to the path of the ancestor node.
      trashPath = (TPath *)malloc(sizeof(TPath));
      trashPath->C = savedParticle[i].C;
      trashPath->D = savedParticle[i].D;
      trashPath->T = savedParticle[i].T;
      trashPath->next = NULL;
      tempPath = l_particle[i].ancestryNode->path;
      while (tempPath->next != NULL)
	tempPath = tempPath->next;
      tempPath->next = trashPath;

      l_particle[j].x = savedParticle[i].x;
      l_particle[j].y = savedParticle[i].y;
      l_particle[j].theta = savedParticle[i].theta;
      l_particle[j].probability = savedParticle[i].probability;
      j++;
    }

    // IF the parent has multiple children, then each child needs its own new ancestor node in the tree
    else if (savedParticle[i].ancestryNode->numChildren > 0) {
      // Find a new entry in the array of ancestor nodes. 
      temp = &(particleID[ availableID[cleanID] ]);
      temp->ID = availableID[cleanID];
      // That ID on the top of the stack is now being used.
      cleanID--;

      if (cleanID < 0) {
	fprintf(stderr, " !!! Insufficient Number of Particle IDs : Abandon Ship !!!\n");
	cleanID = 0;
      }

      // This new node needs to have its info filled in
      temp->parent = savedParticle[i].ancestryNode;
      temp->mapEntries = NULL;
      temp->total = 0;
      temp->size = 0;
      temp->generation = curGeneration;
      temp->numChildren = 0;
      temp->seen = 0;

      trashPath = (TPath *)malloc(sizeof(TPath));
      trashPath->C = savedParticle[i].C;
      trashPath->D = savedParticle[i].D;
      trashPath->T = savedParticle[i].T;
      trashPath->next = NULL;
      temp->path = trashPath;

      // Transfer this entry over to the main particle array
      l_particle[j].ancestryNode = temp;
      l_particle[j].x = savedParticle[i].x;
      l_particle[j].y = savedParticle[i].y;
      l_particle[j].theta = savedParticle[i].theta;
      l_particle[j].probability = savedParticle[i].probability;
      j++;
    }
  }

  l_cur_particles_used = cur_saved_particles_used;

  for (i=0; i < l_cur_particles_used; i++) 
    AddToWorldModel(sense, i);

  // Clean up the ancestry particles which disappeared in branch collapses.
  for (i=0; i < ID_NUMBER-1; i++) 
    if (particleID[i].generation == -111) {
      particleID[i].generation = -1;
      particleID[i].numChildren = 0;
      particleID[i].parent = NULL;
      particleID[i].mapEntries = NULL;
      particleID[i].path = NULL;
      particleID[i].seen = 0;
      particleID[i].total = 0;
      particleID[i].size = 0;

      // Recover the ID. 
      cleanID++;
      availableID[cleanID] = i;
      particleID[i].ID = -3;
    }
}



//
// ReadLog
//
// Reads back into the sensor data structures the raw readings that were stored to file by WriteLog (above)
//
int ReadLog(FILE *logFile, TSense &sense, int &continueSlam) {
  int i, max;
  char line[4096];

  if (fgets(line, 4096, logFile) == NULL) {
    fprintf(stderr, "End of Log File.\n");
    continueSlam = 0;
    return 1;
  }

  if (!strncmp(line, "LASER", 5)) {
      strtok(line, " ");   // This is to remove the keyword
      strtok(NULL, " ");   // Second item is the time of the reading, in seconds. We don't care.
      strtok(NULL, " ");   // Third item is the usecs of the time. We still don't care.
      i = (int) (atof(strtok(NULL, " ")));   // This item is the laser ID
      if (i != 0)
	return 0;

      max = (int) (atof(strtok(NULL, " ")));
      if (max > SENSE_NUMBER)
	max = SENSE_NUMBER;
      strtok(NULL, " ");
      
      // Now read in the whole list of laser readings
      for (i = 0; i < max; i++) {
	sense[i].distance = atof(strtok(NULL, " "))*MAP_SCALE/100.0;
	if (sense[i].distance > MAX_SENSE_RANGE)
	  sense[i].distance = MAX_SENSE_RANGE;
      }
    }
    else 
      fprintf(stderr, "Uninterpretable Line (.rec) : \n %s\n", line);
  }
  else {
    if (!strncmp(line, "Odometry", 8)) {
      strtok(line, " ");
      odometry.x = atof(strtok(NULL, " "));
      odometry.y = atof(strtok(NULL, " "));
      odometry.theta = atof(strtok(NULL, " "));

      if (odometry.theta > M_PI) 
	odometry.theta = odometry.theta - 2*M_PI;
      else if (odometry.theta < -M_PI) 
	odometry.theta = odometry.theta + 2*M_PI;
    }
    else if (!strncmp(line, "Laser", 5)) {
      strtok(line, " ");
      max = (int) (atof(strtok(NULL, " ")));
      if (max > SENSE_NUMBER)
	max = SENSE_NUMBER;
      for (i = 0; i < max; i++) {
	sense[i].distance = atof(strtok(NULL, " "))*MAP_SCALE;
      }
    }
    else 
      fprintf(stderr, "Uninterpretable Line : \n %s\n", line);
  }

  return 0;
}



//
// PrintMap
//
// dump the content into a png file
//
void PrintMap(char *name, TAncestor *parent, int particles, double overlayX, double overlayY, double overlayTheta)
{
  FILE *printFile;
  int x, y, i;
  int width, height;
  int startx, starty, lastx, lasty;
  char sysCall[128];
  double hit, theta;

  width = MAP_WIDTH;
  height = MAP_HEIGHT;

  for(x=0; x < width; x++)
    for(y=0; y<height; y++)
      map[x][y] = 0;

  lastx = 0;
  lasty = 0;
  startx = width-1;
  starty = height-1;

  for (x = 0; x < width; x++) 
    for (y = 0; y < height; y++) {
      hit = LowComputeProb(x, y, 1.4, parent->ID);
      if (hit == UNKNOWN) 
	map[x][y] = 255;
      else {
	map[x][y] = (int) (230 - (hit * 230));
	if (x > lastx)
	  lastx = x;
	if (y > lasty)
	  lasty = y;
	if (x < startx)
	  startx = x;
	if (y < starty)
	  starty = y;
      }
    }

  if (particles) 
    for (i = 0; i < l_cur_particles_used; i++) 
      if ((l_particle[i].x > 0) && (l_particle[i].x < MAP_WIDTH) && (l_particle[i].y > 0) && (l_particle[i].y < MAP_HEIGHT))
	map[(int) (l_particle[i].x)][(int) (l_particle[i].y)] = 254;


  if (overlayX != -1) {
    map[(int) (overlayX)][(int) (overlayY)] = 254;
    for (i = 0; i < SENSE_NUMBER; i++) {
      theta = overlayTheta + sense[i].theta;
      x = (int) (overlayX + (cos(theta) * sense[i].distance));
      y = (int) (overlayY + (sin(theta) * sense[i].distance));

      if ((map[x][y] < 250) || (map[x][y] == 255)) {
	if (sense[i].distance < MAX_SENSE_RANGE) {
	  if (map[x][y] < 200)
	    map[x][y] = 251;
	  else 
	    map[x][y] = 252;
	}
	else
	  map[x][y] = 253;
      }
    }
  }


  // Header file for a ppm
  sprintf(sysCall, "%s.ppm", name);
  printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n",
	  lastx-startx+1, lasty-starty+1);
  fprintf(printFile, "255\n");


  for (y = lasty; y >= starty; y--) 
    for (x = startx; x <= lastx; x++) {
      if (map[x][y] == 254) 
	fprintf(printFile, "%c%c%c", 255, 0, 0);
      else if (map[x][y] == 253) 
	fprintf(printFile, "%c%c%c", 0, 255, 200);
      else if (map[x][y] == 252) 
	fprintf(printFile, "%c%c%c", 255, 55, 55);
      else if (map[x][y] == 251) 
	fprintf(printFile, "%c%c%c", 50, 150, 255);
      else if (map[x][y] == 250) 
	fprintf(printFile, "%c%c%c", 250, 200, 200);
      else if (map[x][y] == 0) 
	fprintf(printFile, "%c%c%c", 100, 250, 100);
      else
	fprintf(printFile, "%c%c%c", map[x][y], map[x][y], map[x][y]);
    }
      

  fclose(printFile);
  sprintf(sysCall, "convert %s.ppm %s.png", name, name);
  system(sysCall);
  sprintf(sysCall, "chmod 666 %s.ppm", name);
  system(sysCall);
  sprintf(sysCall, "chmod 666 %s.png", name);
  system(sysCall);
  fprintf(stderr, "Map dumped to file\n");
}



//
// The function to call (only once) before LowSlam is called, and initializes all values.
//
void InitLowSlam()
{
  int i, j;
  char name[32];

  // Set up the variables to open the correct data log, and identify its format.
  if (PLAYBACK != "") {
    readFile = fopen(PLAYBACK, "r");
    strcpy(name, &PLAYBACK[strlen(PLAYBACK)-3]);
    if (strncmp(name, "rec", 3) == 0) 
      FILE_FORMAT = REC;
    else
      FILE_FORMAT = LOG;
  }

  // All angle values will remain static
  for (i = 0; i < SENSE_NUMBER; i++) 
    sense[i].theta = -(i*M_PI/180.0) * 1.4;

  curGeneration = 0;
  if (PLAYBACK == "") {
    GetSensation(sense);
    GetOdometry(odometry);
  }
  else {
    // Read through the file the specified number of iterations, in order to get to a 
    // later portion of the sensor log.
    for (i=0; i < START_ITERATION; i++) {
      ReadLog(readFile, sense, j);
      ReadLog(readFile, sense, j);
    }

    // Read in the first data before starting SLAM.
    ReadLog(readFile, sense, i);
    ReadLog(readFile, sense, i);
  }
}



//
// This function cleans up the memory and maps that were used by LowSlam.
//
void CloseLowSlam()
{
  if (PLAYBACK != "")
    fclose(readFile);
}


//
// The main function for performing SLAM at the low level. T
//
void LowSlam(int &continueSlam, TPath **path, TSenseLog **obs)
{
  double moveAngle;
  int counter;
  int i, j, overflow = 0;
  char name[32];
  TPath *tempPath;
  TSenseLog *tempObs;
  TAncestor *lineage;

  // Initialize the worldMap
  LowInitializeWorldMap();

  // Initialize the ancestry and particles
  cleanID = ID_NUMBER - 2;    // ID_NUMBER-1 is being used as the root of the ancestry tree.

  // Initialize all of our unused ancestor particles to look unused.
  for (i = 0; i < ID_NUMBER; i++) {
    availableID[i] = i;

    l_particleID[i].generation = -1;
    l_particleID[i].numChildren = 0;
    l_particleID[i].ID = -1;
    l_particleID[i].parent = NULL;
    l_particleID[i].mapEntries = NULL;
    l_particleID[i].path = NULL;
    l_particleID[i].seen = 0;
    l_particleID[i].total = 0;
    l_particleID[i].size = 0;
  }

  // Initialize the root of our ancestry tree.
  l_particleID[ID_NUMBER-1].generation = 0;
  l_particleID[ID_NUMBER-1].numChildren = 1;
  l_particleID[ID_NUMBER-1].size = 0;
  l_particleID[ID_NUMBER-1].total = 0;
  l_particleID[ID_NUMBER-1].ID = ID_NUMBER-1;
  l_particleID[ID_NUMBER-1].parent = NULL;
  l_particleID[ID_NUMBER-1].mapEntries = NULL;

  // Create all of our starting particles at the center of the map.
  for (i = 0; i < PARTICLE_NUMBER; i++) {
    l_particle[i].ancestryNode = &(l_particleID[ID_NUMBER-1]);
    l_particle[i].x = MAP_WIDTH / 2;
    l_particle[i].y = MAP_HEIGHT / 2;
    l_particle[i].theta = 0.001;
    l_particle[i].probability = 0;
    children[i] = 0;
  }
  // We really only use the first particle, since they are all essentially the same.
  l_particle[0].probability = 1;
  l_cur_particles_used = 1;
  children[0] = SAMPLE_NUMBER;

  cur_saved_particles_used = 0;

  lastX = odometry.x;
  lastY = odometry.y;
  lastTheta = odometry.theta;

  overflow = 1;

  if (curGeneration == 0) {
    AddToWorldModel(sense, 0);
    for (i=0; i < SENSE_NUMBER; i++) {
      hold[0].sense[i].distance = sense[i].distance;
      hold[0].sense[i].theta = sense[i].theta;
    }
    curGeneration++;
  }

  else {
    LowInitializeFlags();
    // Add our first observation to our map of the world. This will serve as the basis for future localizations
    AddToWorldModel(hold[(int)(LOW_DURATION*.5)].sense, 0);
    for (i=(int)(LOW_DURATION*.5)+1; i < LOW_DURATION; i++) {
      LowInitializeFlags();
      // Move the particles one step
      moveAngle = l_particle[0].theta + (hold[i].T/2.0);
      l_particle[0].x = l_particle[0].x + (TURN_RADIUS * (cos(l_particle[0].theta + hold[i].T) - cos(l_particle[0].theta))) +
	(hold[i].D * cos(moveAngle)) + (hold[i].C * cos(moveAngle + M_PI/2));
      l_particle[0].y = l_particle[0].y + (TURN_RADIUS * (sin(l_particle[0].theta + hold[i].T) - sin(l_particle[0].theta))) +
	(hold[i].D * sin(moveAngle)) + (hold[i].C * sin(moveAngle + M_PI/2));
      l_particle[0].theta = l_particle[0].theta + hold[i].T;
      
      AddToWorldModel(hold[i].sense, 0);
    }

    for (i=0; i < SENSE_NUMBER; i++) {
      hold[0].sense[i].distance = hold[LOW_DURATION-1].sense[i].distance;
      hold[0].sense[i].theta = hold[LOW_DURATION-1].sense[i].theta;
    }
  }

  // Get our observation log started.
  (*obs) = (TSenseLog *)malloc(sizeof(TSenseLog));
  for (i=0; i < SENSE_NUMBER; i++) {
    (*obs)->sense[i].distance = hold[0].sense[i].distance;
    (*obs)->sense[i].theta = hold[0].sense[i].theta;
  }
  (*obs)->next = NULL;

  continueSlam = 1;
  counter = 0;
  while ((continueSlam) && (counter < LOW_DURATION)) {
    if (PLAYBACK == "") {
      GetOdometry(odometry);
    }
    else {
      // Collect information from the data log. 
      if ((ReadLog(readFile, sense, continueSlam) == 1) || (ReadLog(readFile, sense, continueSlam) == 1))
	overflow = 0;
      else 
	overflow = 1;
    }

    if ((sqrt(SQUARE(odometry.x - lastX) + SQUARE(odometry.y - lastY)) < 0.05) && (fabs(odometry.theta - lastTheta) < 0.03))
      overflow = 0;

    if (overflow > 0) {
      overflow--;

      // Record and preprocess the current laser reading
      if (PLAYBACK == "")
	GetSensation(sense);

      // Wipe the slate clean 
      LowInitializeFlags();
      Localize(sense);
      UpdateAncestry(sense, l_particleID);

      // Update the observation log 
      tempObs = (*obs);
      while (tempObs->next != NULL)
	tempObs = tempObs->next;
      tempObs->next = (TSenseLog *)malloc(sizeof(TSenseLog));
      if (tempObs->next == NULL) fprintf(stderr, "Malloc failed in making a new observation!\n");
      for (i=0; i < SENSE_NUMBER; i++) {
	tempObs->next->sense[i].distance = sense[i].distance;
	tempObs->next->sense[i].theta = sense[i].theta;
      }
      tempObs->next->next = NULL;
.
      for (i=0; i < SENSE_NUMBER; i++) {
	hold[counter].sense[i].distance = sense[i].distance;
	hold[counter].sense[i].theta = sense[i].theta;
      }

      curGeneration++;
      counter++;

      // Remember these odometry readings for next time. This is what lets us know the incremental motion.
      lastX = odometry.x;
      lastY = odometry.y;
      lastTheta = odometry.theta;
    }
  }


  // Find the most likely particle. Return its path
  j = 0;
  for (i=0; i < l_cur_particles_used; i++) 
    if (l_particle[i].probability > l_particle[j].probability)
      j = i;

  (*path) = NULL;
  i = 0;
  lineage = l_particle[j].ancestryNode;
  while ((lineage != NULL) && (lineage->ID != ID_NUMBER-1)) {
    tempPath = lineage->path;
    i++;
    while (tempPath->next != NULL) {
      i++;
      tempPath = tempPath->next;
    }
    tempPath->next = (*path);

    (*path) = lineage->path;
    lineage->path = NULL;
    lineage = lineage->parent;
  }

  tempPath = (*path);
  i = 0;
  while (tempPath != NULL) {
    hold[i].C = tempPath->C;
    hold[i].D = tempPath->D;
    hold[i].T = tempPath->T;
    tempPath = tempPath->next;
    i++;
  }

  // Print out the map.
  sprintf(name, "lmap%.2d", (int) (curGeneration/LOW_DURATION)-1);
  j = 0;
  for (i = 0; i < l_cur_particles_used; i++)
    if (l_particle[i].probability > l_particle[j].probability)
      j = i;
  PrintMap(name, l_particle[j].ancestryNode, FALSE, -1, -1, -1);
  sprintf(name, "rm lmap%.2d.ppm", (int) (curGeneration/LOW_DURATION)-1);
  system(name);

  // Clean up the memory being used.
  DisposeAncestry(l_particleID);
  LowDestroyMap();
}

