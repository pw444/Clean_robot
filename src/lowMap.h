
#include "map.h"

// The global map used by the low level slam process. T
extern PMapStarter lowMap[MAP_WIDTH][MAP_HEIGHT];
// The nodes of the ancestry tree are stored here. 
extern TAncestor l_particleID[ID_NUMBER];


extern TParticle l_particle[PARTICLE_NUMBER];
extern int l_cur_particles_used;


void LowInitializeFlags();
void LowInitializeWorldMap();
void LowDestroyMap();
void LowResizeArray(TMapStarter *node, int deadID);
void LowDeleteObservation(short int x, short int y, short int node);
double LowComputeProb(int x, int y, double distance, int ID);

void LowAddTrace(double startx, double starty, double MeasuredDist, double theta, int parentID, int addEnd);
double LowLineTrace(double startx, double starty, double theta, double MeasuredDist, int parentID, float culling);
