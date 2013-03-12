#include "Collision.h"
#include "Struct.h"
#include "Functions.h"

#include <math.h>

#include <iostream>
#include "SaveData.h"

int Collision(VectorXd vecNode, VectorXd vecParent, world_t world, double dSZR, double dSZH)
{
	VectorXd dt;
	Vector3d p;
	int collision_flag, n, i, j;
	double dLEN, dSigma, dNorm;
	
	collision_flag = 0;
	
	if ( 	(vecNode(0)>world.vecNEcorner(0))|(vecNode(0)<world.vecSWcorner(0)) |
		(vecNode(1)>world.vecNEcorner(1))|(vecNode(1)<world.vecSWcorner(1)) |
		(vecNode(2)>world.vecNEcorner(2))|(vecNode(2)<world.vecSWcorner(2)) )
		collision_flag = 1;
	else
	{
		dLEN = (vecNode-vecParent).norm();
		
		dt = linspace(0,1,ceil(dLEN/0.5));
		n = dt.size();
		
		for (i=0; i<n; i++)
		{
			dSigma = dt(i);
			
			p <<  dSigma*vecNode(0)+(1-dSigma)*vecParent(0),
				dSigma*vecNode(1)+(1-dSigma)*vecParent(1),
				dSigma*vecNode(2)+(1-dSigma)*vecParent(2);
			
			
			// check each obstacle
			for (j=0; j<world.iNumObstacles; j++)
			{
				dNorm = sqrt( (p(0)-world.vecCN(j))*(p(0)-world.vecCN(j))+
					(p(1)-world.vecCE(j))*(p(1)-world.vecCE(j)) );
				
				if ( (dNorm<=world.vecRadius(j)+dSZR) && (p(2)<=world.vecZH(j)+dSZH) )
				{
					collision_flag = 1;
					
					return collision_flag;
				}
			}
		}
	}
  
	return collision_flag;
}



int Collision(VectorXd vecNode, VectorXd vecParent, worldRect_t world, double dSZR, double dSZH)
{
	VectorXd dt;
	Vector3d p, pTrans, center;
	MatrixXd rot(3,3);
	int collision_flag, n, i, j;
	double dLEN, dSigma, dTh;
	
	collision_flag = 0;
	
	if ( 	(vecNode(0)>world.vecNEcorner(0))|(vecNode(0)<world.vecSWcorner(0)) |
		(vecNode(1)>world.vecNEcorner(1))|(vecNode(1)<world.vecSWcorner(1)) |
		(vecNode(2)>world.vecNEcorner(2))|(vecNode(2)<world.vecSWcorner(2)) )
		collision_flag = 1;
	else
	{
		dLEN = (vecNode-vecParent).norm();
		dt = linspace(0,1,ceil(dLEN/0.5));
		n = dt.size();
		
		// check each obstacle
		for (j=0; j<world.iNumObstacles; j++)
		{
			dTh = world.vecRot(j);
			
			rot << cos(-dTh), -sin(-dTh), 0,
				sin(-dTh), cos(-dTh), 0,
				0, 0, 1;
			
			center << world.vecCN(j), world.vecCE(j), 0;
			
			for (i=0; i<n; i++)
			{
				dSigma = dt(i);
				
				p <<  dSigma*vecNode(0)+(1-dSigma)*vecParent(0),
				dSigma*vecNode(1)+(1-dSigma)*vecParent(1),
				dSigma*vecNode(2)+(1-dSigma)*vecParent(2);
				
				pTrans = p - center;
				p = rot*pTrans + center;	
				
				if ( ( abs(p(0)-world.vecCN(j)) <= world.vecHorizontal(j) + dSZR ) &&
					( abs(p(1)-world.vecCE(j)) <= world.vecVertical(j) + dSZR ) &&
					( p(2)<world.vecZ(j) + dSZH ) )
				{
					collision_flag = 1;
					
					return collision_flag;
				}
			}
		}
	}
	
	return collision_flag;
}



int Collision(VectorXd vecNode, VectorXd vecParent, worldLine_t world, double dSZR, double dSZH)
{
	VectorXd dt;
	int collision_flag, n, i, j;
	double dLEN, dSigma;
	
	CURB_Point_2D p;
	
	collision_flag = 0;
	
	double d1, d2, d3;
	d1 = vecNode(0);
	d2 = vecNode(1);
	d3 = vecNode(2);
	
	
	dLEN = (vecNode-vecParent).norm();
	dt = linspace(0,1,ceil(dLEN/0.5));
	n = dt.size();
	
	// check each obstacle
	for (j=0; j<world.iNumObstacles; j++)
	{
		for (i=0; i<n; i++)
		{
			dSigma = dt(i);
			
			p.x = dSigma*vecNode(0)+(1-dSigma)*vecParent(0);
			p.y = dSigma*vecNode(1)+(1-dSigma)*vecParent(1);
			
			if (world.lineSeg.at(j).GetDistance(p) < dSZR)
			{
				collision_flag = 1;
				
				return collision_flag;
			}
		}
	}
	
	return collision_flag;
}



int Collision10(VectorXd vecNode, VectorXd vecParent, world_t world, double dSZR, double dSZH)
{
	VectorXd dt;
	Vector3d p;
	int collision_flag, n, i, j;
	double dLEN, dSigma, dNorm;

	collision_flag = 0;

	if ( 	(vecNode(0)>world.vecNEcorner(0))|(vecNode(0)<world.vecSWcorner(0)) |
		(vecNode(1)>world.vecNEcorner(1))|(vecNode(1)<world.vecSWcorner(1)) |
		(vecNode(2)>world.vecNEcorner(2))|(vecNode(2)<world.vecSWcorner(2)) )
		collision_flag = 1;
	else
	{
		dLEN = (vecNode-vecParent).norm();
		dt = linspace(0,1,ceil(dLEN/0.2));
		n = dt.size();
		
		for (i=0; i<n; i++)
		{
			dSigma = dt(i);
			
			p <<  dSigma*vecNode(0)+(1-dSigma)*vecParent(0),
				dSigma*vecNode(1)+(1-dSigma)*vecParent(1),
				dSigma*vecNode(2)+(1-dSigma)*vecParent(2);
			
			
			// check each obstacle
			for (j=0; j<world.iNumObstacles; j++)
			{
				dNorm = sqrt( (p(0)-world.vecCN(j))*(p(0)-world.vecCN(j))+
					(p(1)-world.vecCE(j))*(p(1)-world.vecCE(j)) );
				
				if ( (dNorm<=world.vecRadius(j)+dSZR) && (p(2)<=world.vecZH(j)+dSZH) )
				{
					collision_flag = 1;
					
					return collision_flag;
				}
			}
		}
	}
	
	return collision_flag;
}



int Collision10(VectorXd vecNode, VectorXd vecParent, worldRect_t world, double dSZR, double dSZH)
{
	VectorXd dt;
	Vector3d p, pTrans, center;
	MatrixXd rot(3,3);
	int collision_flag, n, i, j;
	double dLEN, dSigma, dTh;

	collision_flag = 0;

	if ( 	(vecNode(0)>world.vecNEcorner(0))|(vecNode(0)<world.vecSWcorner(0)) |
		(vecNode(1)>world.vecNEcorner(1))|(vecNode(1)<world.vecSWcorner(1)) |
		(vecNode(2)>world.vecNEcorner(2))|(vecNode(2)<world.vecSWcorner(2)) )
		collision_flag = 1;
	else
	{
		dLEN = (vecNode-vecParent).norm();
		dt = linspace(0,1,ceil(dLEN/0.2));
		n = dt.size();
		
		// check each obstacle
		for (j=0; j<world.iNumObstacles; j++)
		{
			dTh = world.vecRot(j);
			
			rot << cos(-dTh), -sin(-dTh), 0,
				sin(-dTh), cos(-dTh), 0,
				0, 0, 1;
			
			center << world.vecCN(j), world.vecCE(j), 0;
			
			for (i=0; i<n; i++)
			{
				dSigma = dt(i);
				
				p <<  dSigma*vecNode(0)+(1-dSigma)*vecParent(0),
					dSigma*vecNode(1)+(1-dSigma)*vecParent(1),
					dSigma*vecNode(2)+(1-dSigma)*vecParent(2);
				
				pTrans = p - center;
				p = rot*pTrans + center;

				if ( ( abs(p(0)-world.vecCN(j)) <= world.vecHorizontal(j) + dSZR ) &&
					( abs(p(1)-world.vecCE(j)) <= world.vecVertical(j) + dSZR ) &&
					( p(2)<world.vecZ(j) + dSZH ) )
				{
					collision_flag = 1;
					
					return collision_flag;
				}
			}
		}
	}
	
	return collision_flag;
}



int Collision10(VectorXd vecNode, VectorXd vecParent, worldLine_t world, double dSZR, double dSZH)
{
	VectorXd dt;
	int collision_flag, n, i, j;
	double dLEN, dSigma;

	CURB_Point_2D p;
	
	collision_flag = 0;
	
	dLEN = (vecNode-vecParent).norm();
	//   dt = linspace(0,1,ceil(dLEN/0.2));
	dt = linspace(0,1,10);
	n = dt.size();
	
	// check each obstacle
	for (j=0; j<world.iNumObstacles; j++)
	{
		for (i=0; i<n; i++)
		{
			dSigma = dt(i);
			
			p.x = dSigma*vecNode(0)+(1-dSigma)*vecParent(0);
			p.y = dSigma*vecNode(1)+(1-dSigma)*vecParent(1);
			
			if (world.lineSeg.at(j).GetDistance(p) < dSZR)
			{
				collision_flag = 1;

				return collision_flag;
			}
		}
	}
	
	return collision_flag;
}



int CollisionPoint(VectorXd vecNode, world_t world, double dSZR, double dSZH)
{
	int collision_flag, i;
	double dNorm;
	
	collision_flag = 0;
	
	for (i=0; i<world.iNumObstacles; i++)
	{
		dNorm = sqrt( (vecNode(0)-world.vecCN(i))*(vecNode(0)-world.vecCN(i))+
				(vecNode(1)-world.vecCE(i))*(vecNode(1)-world.vecCE(i)) );
		
		if ( (dNorm<=world.vecRadius(i)+dSZR) && (vecNode(2)<=world.vecZH(i)+dSZH) )
		{
			collision_flag = 1;
			
			return collision_flag;
		}
	}
	
	return collision_flag;
}



int CollisionSmooth(VectorXd vecX, VectorXd vecY, VectorXd vecZ, world_t world, double dSZR, double dSZH)
{
	/// check to see if a node is in collision with obstacles 10 times
	/// general collision function check just 5 times
	int collision_flag, i;
	double t;
	VectorXd vecP;
	
	VectorXd vecTemp;
	
	collision_flag = 0;
	
	vecTemp.resize(2);
  	
	for (t=0; t<1+0.05; t=t+0.05)
	{
		CurvePointCalculation(vecX, vecY, vecZ, t, vecP);
		
		/// check each obstacle
		for (i=0; i<world.iNumObstacles; i++)
		{
			vecTemp << vecP(1)-world.vecCN(i), vecP(2)-world.vecCE(i);
			
			if ( (vecTemp.norm() <= (world.vecRadius(i)+dSZR)) & (vecP(3) < world.vecZH(i) + dSZH) )
			{
				collision_flag = 1;
				
				return collision_flag;
			}
		}
	}
	
	return collision_flag;
}



int CollisionSmooth(VectorXd vecX, VectorXd vecY, VectorXd vecZ, worldRect_t world, double dSZR, double dSZH)
{
	/// check to see if a node is in collision with obstacles 10 times
	/// general collision function check just 5 times
	int collision_flag, i;
	double t, dTh;
	MatrixXd rot(3,3);
	VectorXd vecP;
	Vector3d pTrans, center;
	
	collision_flag = 0;
	
	/// check each obstacle
	for (i=0; i<world.iNumObstacles; i++)
	{
		dTh = world.vecRot(i);
		
		rot << cos(-dTh), -sin(-dTh), 0,
			sin(-dTh), cos(-dTh), 0,
			0, 0, 1;
			
		for (t=0; t<1+0.05; t=t+0.05)
		{
			CurvePointCalculation(vecX, vecY, vecZ, t, vecP);
			
			pTrans = vecP - center;
			vecP = rot*pTrans + center;
			
			if ( ( abs(vecP(1) - world.vecCN(i)) <= world.vecHorizontal(i)+dSZR ) &&
				( abs(vecP(2) - world.vecCE(i)) <= world.vecVertical(i)+dSZR ) &&
				( vecP(3) <= world.vecZ(i)+dSZH ) )
			{
				collision_flag = 1;
				
				return collision_flag;
			}
		}
	}
	
	return collision_flag;
}



int CollisionSmooth(VectorXd vecX, VectorXd vecY, VectorXd vecZ, worldLine_t world, double dSZR, double dSZH)
{
	/// check to see if a node is in collision with obstacles 10 times
	/// general collision function check just 5 times
	int collision_flag, i;
	double t, dTh;
	MatrixXd rot(3,3);
	VectorXd vecP;
	
	CURB_Point_2D p;
	
	collision_flag = 0;
	
	/// check each obstacle
	for (i=0; i<world.iNumObstacles; i++)
	{
		for (t=0; t<1+0.05; t=t+0.05)
		{
			CurvePointCalculation(vecX, vecY, vecZ, t, vecP);
			
			p.x = vecP(0);
			p.y = vecP(1);
			
			if (world.lineSeg.at(i).GetDistance(p) < dSZR)
			{
				collision_flag = 1;
				
				return collision_flag;
			}
		}
	}
	
	return collision_flag;
}
