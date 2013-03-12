#include "SaveData.h"
#include <iostream>

void SaveMatData(FILE* fOut, MatrixXd matIn)
{
	int row, col;
	
	for (row=0; row<matIn.rows(); row++)
	{
		for (col=0; col<matIn.cols(); col++)
			fprintf(fOut, "%f\t", matIn(row,col));

		fprintf(fOut, "\n");
	}
}


void SaveVecData(FILE* fOut, VectorXd vecIn)
{
	int idx;
	
	for (idx=0; idx<vecIn.size(); idx++)
		fprintf(fOut, "%lf\t", vecIn(idx));
}


void SaveResult(FILE *fOut, MatrixXd matTree, MatrixXd matPathFinal, MatrixXd matFinalPoint, VectorXd vecT,
		VectorXd vecFsx, 	VectorXd vecFsy, 	VectorXd vecFsz,
		VectorXd vecFsKappa1, 	VectorXd vecFsKappa2, 	VectorXd vecFsKappa3, VectorXd vecFsKappa4,
		VectorXd vecLs1, 	VectorXd vecLs2, 	VectorXd vecLs3,
		VectorXd vecFx, 	VectorXd vecFy, 	VectorXd vecFz,
		VectorXd vecFKappa1, 	VectorXd vecFKappa2, 	VectorXd vecFKappa3, VectorXd vecFKappa4,
		VectorXd vecL1, 	VectorXd vecL2, 	VectorXd vecL3,
		VectorXd vecFsPsi, 	VectorXd vecFPsi)
{
	// Tree
	fOut = fopen("data/Tree.txt","wt");
	SaveMatData(fOut, matTree);
	fclose(fOut);
	
	// FinalPath
	fOut = fopen("data/FinalPath.txt","wt");
	SaveMatData(fOut, matPathFinal);
	fclose(fOut);
	
	// FinalPoint
	fOut = fopen("data/FinalPoint.txt","wt");
	SaveMatData(fOut, matFinalPoint);
	fclose(fOut);
	
	// t
	fOut = fopen("data/t.txt","wt");
	SaveVecData(fOut, vecT);
	fclose(fOut);
	
	// [Fs_x; Fs_y; Fs_z]
	fOut = fopen("data/Fs_xyz.txt","wt");
	SaveVecData(fOut, vecFsx);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFsy);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFsz);  fprintf(fOut,"\n");
	fclose(fOut);
	
	// [Fs_Kappa1; Fs_Kappa2; Fs_Kappa3; Fs_Kappa4]
	fOut = fopen("data/Fs_Kappa.txt","wt");
	SaveVecData(fOut, vecFsKappa1);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFsKappa2);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFsKappa3);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFsKappa4);  fprintf(fOut,"\n");
	fclose(fOut);
	
	// [Ls1; Ls2; Ls3]
	fOut = fopen("data/Ls.txt","wt");
	SaveVecData(fOut, vecLs1);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecLs2);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecLs3);  fprintf(fOut,"\n");
	fclose(fOut);

	// [F_x; F_y; F_z]
	fOut = fopen("data/F_xyz.txt","wt");
	SaveVecData(fOut, vecFx);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFy);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFz);  fprintf(fOut,"\n");
	fclose(fOut);
	
	// [F_Kappa1; F_Kappa2; F_Kappa3; F_Kappa4]
	fOut = fopen("data/F_Kappa.txt","wt");
	SaveVecData(fOut, vecFKappa1);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFKappa2);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFKappa3);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFKappa4);  fprintf(fOut,"\n");
	fclose(fOut);
	
	// [L1; L2; L3]
	fOut = fopen("data/L.txt","wt");
	SaveVecData(fOut, vecL1);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecL2);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecL3);  fprintf(fOut,"\n");
	fclose(fOut);
	
	// Fs_Psi
	fOut = fopen("data/Fs_Psi.txt","wt");
	SaveVecData(fOut, vecFsPsi);
	fclose(fOut);
	
	// F_Psi
	fOut = fopen("data/F_Psi.txt","wt");
	SaveVecData(fOut, vecFPsi);
	fclose(fOut);
}

void SaveResult(FILE *fOut,
		VectorXd vecFsxs, 	VectorXd vecFsys, 	VectorXd vecFszs, 	VectorXd vecFsPsis,
		VectorXd vecFsKappa1s, 	VectorXd vecFsKappa2s, 	VectorXd vecFsKappa3s, 	VectorXd vecFsKappa4s,
		VectorXd vecFsLs, 	VectorXd vecLss1, 	VectorXd vecLss2, 	VectorXd vecLss3)
{
	// [F_x; F_y; F_z]
	fOut = fopen("data/Fs_xyzs.txt","wt");
	SaveVecData(fOut, vecFsxs);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFsys);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFszs);  fprintf(fOut,"\n");
	fclose(fOut);

	// F_Psi
	fOut = fopen("data/Fs_Psis.txt","wt");
	SaveVecData(fOut, vecFsPsis);
	fclose(fOut);

	// [F_Kappa1; F_Kappa2; F_Kappa3; F_Kappa4]
	fOut = fopen("data/Fs_Kappas.txt","wt");
	SaveVecData(fOut, vecFsKappa1s);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFsKappa2s);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFsKappa3s);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecFsKappa4s);  fprintf(fOut,"\n");
	fclose(fOut);
	
	// F_Psi
	fOut = fopen("data/Fs_Ls.txt","wt");
	SaveVecData(fOut, vecFsLs);
	fclose(fOut);
	
	// [L1; L2; L3]
	fOut = fopen("data/Lss.txt","wt");
	SaveVecData(fOut, vecLss1);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecLss2);  fprintf(fOut,"\n");
	SaveVecData(fOut, vecLss3);  fprintf(fOut,"\n");
	fclose(fOut);
}



void SaveParameter(FILE* fOut, double d, double SZR, double SZH, int nns, int nn)
{
	fOut = fopen("data/Parameters.txt","wt");
	fprintf(fOut,"%f\t%f\t%f\t%d\t%d\t", d, SZR, SZH, nns, nn);
	fclose(fOut);
}


void SaveWorld(FILE* fOut, world_t world)
{
	// Save world setting
	fOut = fopen("data/World_setting.txt","wt");
	fprintf(fOut,"%d\t",world.iNumObstacles);
	SaveVecData(fOut, world.vecNEcorner);
	SaveVecData(fOut, world.vecSWcorner);
	fclose(fOut);
	
	// Save world data [cn; ce; radius; zl; zh]
	fOut = fopen("data/World_data.txt","wt");
	SaveVecData(fOut, world.vecCN);	fprintf(fOut,"\n");
	SaveVecData(fOut, world.vecCE);	fprintf(fOut,"\n");
	SaveVecData(fOut, world.vecRadius);	fprintf(fOut,"\n");
	SaveVecData(fOut, world.vecZL);	fprintf(fOut,"\n");
	SaveVecData(fOut, world.vecZH);	fprintf(fOut,"\n");
	fclose(fOut);
}


void SaveWorld(FILE* fOut, worldRect_t world)
{
	// Save world setting
	fOut = fopen("data/World_setting.txt","wt");
	fprintf(fOut,"%d\t",world.iNumObstacles);
	SaveVecData(fOut, world.vecNEcorner);
	SaveVecData(fOut, world.vecSWcorner);
	fclose(fOut);
	
	// Save world data [cn; ce; radius; zl; zh]
	fOut = fopen("data/World_data.txt","wt");
	SaveVecData(fOut, world.vecCN);		fprintf(fOut,"\n");
	SaveVecData(fOut, world.vecCE);		fprintf(fOut,"\n");
	SaveVecData(fOut, world.vecHorizontal);	fprintf(fOut,"\n");
	SaveVecData(fOut, world.vecVertical);		fprintf(fOut,"\n");
	SaveVecData(fOut, world.vecZ);		fprintf(fOut,"\n");
	SaveVecData(fOut, world.vecRot);		fprintf(fOut,"\n");
	fclose(fOut);
}



void SaveWorld(FILE* fOut, worldLine_t world, int num)
// void SaveWorld(FILE* fOut, worldLine_t world)
{
	char buffer [50];
	
	// Save world setting  
	sprintf(buffer,"data/World_setting%d.txt",num);
	fOut = fopen(buffer,"wt");
	
	fprintf(fOut,"%d\t",world.iNumObstacles);
//  SaveVecData(fOut, world.vecNEcorner);
//  SaveVecData(fOut, world.vecSWcorner);
	fclose(fOut);
	
	// Save world data [cn; ce; radius; zl; zh]
	sprintf(buffer,"data/World_data%d.txt",num);
	fOut = fopen(buffer,"wt");
	
	for (int i=0; i<world.lineSeg.size(); i++)
	{
		fprintf(fOut, "%f\t", world.lineSeg.at(i).startPnt.x);
		fprintf(fOut, "%f\t", world.lineSeg.at(i).startPnt.y);
		fprintf(fOut, "%f\t", world.lineSeg.at(i).endPnt.x);
		fprintf(fOut, "%f\t", world.lineSeg.at(i).endPnt.y);
		
		fprintf(fOut,"\n");
	}
	
	fclose(fOut);
}

void SaveKnownWorld(FILE* fOut, worldLine_t world, int num)
// void SaveKnownWorld(FILE* fOut, worldLine_t world)
{
	char buffer [50];
	
	// Save world setting
	sprintf(buffer,"data/WorldKnown_setting%d.txt",num);
	fOut = fopen(buffer,"wt");
	
	fprintf(fOut,"%d\t",world.iNumObstacles);
//  SaveVecData(fOut, world.vecNEcorner);
//  SaveVecData(fOut, world.vecSWcorner);
	fclose(fOut);
	
	// Save world data [cn; ce; radius; zl; zh]
	sprintf(buffer,"data/WorldKnown_data%d.txt",num);
	fOut = fopen(buffer,"wt");  
	
	for (int i=0; i<world.lineSeg.size(); i++)
	{
		fprintf(fOut, "%f\t", world.lineSeg.at(i).startPnt.x);
		fprintf(fOut, "%f\t", world.lineSeg.at(i).startPnt.y);
		fprintf(fOut, "%f\t", world.lineSeg.at(i).endPnt.x);
		fprintf(fOut, "%f\t", world.lineSeg.at(i).endPnt.y);
		
		fprintf(fOut,"\n");
	}
	
	fclose(fOut);
}
