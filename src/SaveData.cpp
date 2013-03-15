#include "SaveData.h"
#include <iostream>

void SaveMatData(FILE* fOut, MatrixXd matIn)
{
	for (int i=0; i<matIn.rows(); i++) {
		for (int j=0; j<matIn.cols(); j++)
			fprintf(fOut, "%f\t", matIn(i, j));

		fprintf(fOut, "\n");
	}
}

void SaveVecData(FILE* fOut, VectorXd vecIn)
{
	for (int idx=0; idx<vecIn.size(); idx++)
		fprintf(fOut, "%lf\t", vecIn(idx));
}

void SaveWorld(FILE* fOut, worldLine_t world, int num)
// void SaveWorld(FILE* fOut, worldLine_t world)
{
	char buffer [50];
	
	// Save world setting  
	sprintf(buffer,"data/World_setting%d.txt",num);
	fOut = fopen(buffer,"wt");
	
	fprintf(fOut,"%d\t",world.iNumObstacles);
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
