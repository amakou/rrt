#include "Collision.h"
#include "Struct.h"
#include "Functions.h"

#include <math.h>
#include <iostream>
#include "SaveData.h"

int Collision(VectorXd vecNode, VectorXd vecParent, worldLine_t world, double dSZR, double dSZH, int precision)
{
	VectorXd dt;
	double dSigma;
	CURB_Point_2D p;
	
	if ( precision > 1 )
		dt = linspace(0, 1, precision);
	else
		dt = linspace(0, 1, ceil((vecNode-vecParent).norm()/0.5));

	// check each obstacle
	for (int j=0; j < world.iNumObstacles; j++) {
		for (int i=0; i < dt.size(); i++) {
			dSigma = dt(i);
			p.x = dSigma*vecNode(0)+(1-dSigma)*vecParent(0);
			p.y = dSigma*vecNode(1)+(1-dSigma)*vecParent(1);
			
			if (world.lineSeg.at(j).GetDistance(p) < dSZR)
				return 1;
		}
	}
	return 0;
}

int CollisionSmooth(VectorXd vecX, VectorXd vecY, VectorXd vecZ, worldLine_t world, double dSZR, double dSZH)
{
	// check to see if a node is in collision with obstacles 10 times
	// general collision function check just 5 times
	VectorXd vecP;
	CURB_Point_2D p;

	/// check each obstacle
	for (int i=0; i < world.iNumObstacles; i++)	{
		for (double t=0; t < 1+0.05; t+=0.05) {
			CurvePointCalculation(vecX, vecY, vecZ, t, vecP);
			p.x = vecP(0);
			p.y = vecP(1);

			if (world.lineSeg.at(i).GetDistance(p) < dSZR)
				return 1;
		}
	}
	return 0;
}
