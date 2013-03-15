#ifndef SAVEDATA_H_
#define SAVEDATA_H_

#include "Struct.h"
#include "fstream"

// SaveResult & SaveParameter : Used to save MATLAB plot data

void SaveMatData(FILE *fOut, MatrixXd matIn);

void SaveVecData(FILE *fOut, VectorXd vecIn);

void SaveWorld(FILE *fOut, worldLine_t world, int num);

void SaveKnownWorld(FILE *fOut, worldLine_t world, int num);

#endif /* SAVEDATA_H_ */
