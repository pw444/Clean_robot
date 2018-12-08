// define the basic arguments for generated map

#include "laser.h"

#define UNKNOWN -2


#define START_ITERATION 0


// We need to know, for various purposes, how big our map is allowed to be
#define MAP_WIDTH  1700
#define MAP_HEIGHT 1700

// This is the number of particles that we are keeping at the low level
#define PARTICLE_NUMBER 50
// This is the number of samples that we will generate each iteration. 
#define SAMPLE_NUMBER (PARTICLE_NUMBER*10)
// Number of unique particle ID numbers. Each particle (and ancestry particle) gets its own 
#define ID_NUMBER (int) (PARTICLE_NUMBER*2.25)


// A bounding box on the semicircle of possible observations
#define AREA (int) (3.1415 * 55.0 *MAP_SCALE*MAP_SCALE)


//The basic cell for map
struct MapNode_struct;
struct MapNode_struct {
  // An index into the array of observations kept by the associated ancestor node. 
  int source;
  // The total distance that laser traces have been observed to pass through this grid square.
  float distance;
  // The number of times that a laser has been observed to stop in this grid square (implying a possible object)
  short int hits;
  // The ID of the ancestor node which made this observation
  short int ID;
  short int parentGen;
};
typedef struct MapNode_struct *PMapNode;
typedef struct MapNode_struct TMapNode;

struct MapNodeStarter_struct;
struct MapNodeStarter_struct {
  // Total is the number of entries in the array which are currently being used.
  // Size is the total size of the array.
  // Dead indicates how many of those slots currently in use are taken up by obsolete entries
  short int total, size, dead;
  PMapNode array;
};
typedef struct MapNodeStarter_struct TMapStarter;
typedef struct MapNodeStarter_struct *PMapStarter;


struct TEntryList_struct;
struct TEntryList_struct {
  short int node;
  short int x, y;
};
typedef struct TEntryList_struct TEntryList;


// Holds the information needed for the ancestry tree. 
struct TAncestor_struct;
struct TAncestor_struct {
  struct TAncestor_struct *parent;
  TEntryList *mapEntries;
  int size, total;
  short int generation, ID, numChildren;
  TPath *path;  // An addition for hierarchical- maintains the partial robot path represented by this particle
  char seen;  // Used by various functions for speedy traversal of the tree. 
};
typedef struct TAncestor_struct TAncestor;
typedef struct TAncestor_struct *PAncestor;


struct TParticle_struct {
  float x, y, theta; // The current position of the particle, in terms of grid squares and radians
  float C, D, T;  // Minor and major axis of motion, and change of facing, respectively
  double probability; // The proability of the particle
  TAncestor_struct *ancestryNode; 
};
typedef struct TParticle_struct TParticle;


// These are structures used to speed up the code, and allow for an efficient use of the observation cache.
extern int flagMap[H_MAP_WIDTH][H_MAP_HEIGHT];
extern short int obsX[AREA], obsY[AREA];

// This is where the actual observation cache is stored. 
extern short int observationArray[AREA][TOP_ID_NUMBER];

// The number of entries of observationArray currently being used.
extern int observationID;
