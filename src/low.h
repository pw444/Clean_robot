// This file is to generate local partial map 
#include "lowMap.h"

// The function to call only once before LowSlam is called, and initializes all values.
void InitLowSlam();
// This function cleans up the memory and maps that were used by LowSlam.
void CloseLowSlam();
// The main function for performing SLAM at the low level. 
void LowSlam(int &continueSlam, TPath **path, TSenseLog **obs);



