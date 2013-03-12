#ifndef SAVEDATA_H_
#define SAVEDATA_H_

#include "Struct.h"
#include "fstream"

/// SaveResult & SaveParameter : Used to save MATLAB plot data

void SaveMatData(FILE *fOut, MatrixXd matIn);

void SaveVecData(FILE *fOut, VectorXd vecIn);

void SaveResult(FILE *fOut, MatrixXd matTree, MatrixXd matPathFinal, MatrixXd matFinalPoint, VectorXd vecT,
		VectorXd vecFsx,	VectorXd vecFsy,	VectorXd vecFsz,
		VectorXd vecFsKappa1,	VectorXd vecFsKappa2, 	VectorXd vecFsKappa3, 	VectorXd vecFsKappa4,
		VectorXd vecLs1, 	VectorXd vecLs2, 	VectorXd vecLs3,
		VectorXd vecFx, 	VectorXd vecFy, 	VectorXd vecFz,
		VectorXd vecFKappa1, 	VectorXd vecFKappa2, 	VectorXd vecFKappa3, 	VectorXd vecFKappa4,
		VectorXd vecL1, 	VectorXd vecL2, 	VectorXd vecL3,
		VectorXd vecFsPsi, 	VectorXd vecFPsi);

void SaveResult(FILE *fOut,
		VectorXd vecFsxs, 	VectorXd vecFsys, 	VectorXd vecFszs, 	VectorXd vecFsPsis,
		VectorXd vecFsKappa1s, 	VectorXd vecFsKappa2s, 	VectorXd vecFsKappa3s, 	VectorXd vecFsKappa4s,
		VectorXd vecFsLs, 	VectorXd vecLss1, 	VectorXd vecLss2, 	VectorXd vecLss3);

void SaveParameter(FILE *fOut, double d, double SZR, double SZH, int nns, int nn);

void SaveWorld(FILE *fOut, world_t world);

void SaveWorld(FILE *fOut, worldRect_t world);

void SaveWorld(FILE *fOut, worldLine_t world, int num);

void SaveKnownWorld(FILE *fOut, worldLine_t world, int num);


#endif /* SAVEDATA_H_ */