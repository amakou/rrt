#include "CreateWorld.h"
#include "Functions.h"

world_t CreateWorld_TRO(int iNumObstacles, Vector3d vecNEcorner, Vector3d vecSWcorner)
{
  /// create random world with obstacles
  /// the first element is the north coordinate
  /// the second element is the south coordinate
  
  world_t world;
  
  // Obstacle Number
  world.iNumObstacles = iNumObstacles;
  
  // Size of the world
  world.vecNEcorner = vecNEcorner;
  world.vecSWcorner = vecSWcorner;
  
  world.vecCN.resize(iNumObstacles);
  world.vecCE.resize(iNumObstacles);
  world.vecRadius.resize(iNumObstacles);
  world.vecZL.resize(iNumObstacles);
  world.vecZH.resize(iNumObstacles);
  
  
  // center of x direction
  world.vecCN << 	20,	65,	150,	40,	100,	170,	40,	114,	140,	200,
			89,	180,	230,	220,	130,	280,	200,	250,	150,	250,
			45,	70,	220,	280,	270;
  // center of y direction
  world.vecCE <<	100,	110,	35,	40,	50,	160,	160,	140,	100,	75,
			221,	210,	200,	140,	200,	210,	270,	250,	250,	100,
			210,	260,	30,	150,	50;
  // radius of circle
  world.vecRadius << 	8,	15,	8,	15,	9,	14,	11,	13,	15,	12,
			12,	11,	12,	14,	15,	10,	8,	15,	8,	8,
			8,	8,	8,	8,	5;
  // height of building : botton
  world.vecZL <<	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
			0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
			0,	0,	0,	0,	0;
			
  world.vecZH <<	20,	20,	24,	25,	22,	22,	20,	22,	27,	28,
			26,	25,	28,	20,	25,	28,	28,	20,	20,	22,
			20,	20,	20,	25,	21;

  return world;
}



world_t CreateWorld_Obstacles(int iNumObstacles, Vector3d vecNEcorner, Vector3d vecSWcorner)
{
  /// create random world with obstacles
  /// the first element is the north coordinate
  /// the second element is the south coordinate
  
  world_t world;
  int scale = 1;
  
  // Obstacle Number
  world.iNumObstacles = iNumObstacles;
  
  // Size of the world
  world.vecNEcorner = vecNEcorner;
  world.vecSWcorner = vecSWcorner;
  
  world.vecCN.resize(iNumObstacles);
  world.vecCE.resize(iNumObstacles);
  world.vecRadius.resize(iNumObstacles);
  world.vecZL.resize(iNumObstacles);
  world.vecZH.resize(iNumObstacles);
  
  
  // center of x directioni
  world.vecCN <<	20,	80,	150,	40,	100,	185,	40,	114,	140,	200,
			82,	180,	230,	220,	130,	280,	200,	250,	150,	250,
			80,	70,	220,	280,	270,	30,	80,	65,	125,	190;
  // center of y direction
  world.vecCE <<	100,	85,	35,	40,	50,	170,	160,	140,	100,	75,
			180,	210,	200,	140,	200,	210,	270,	250,	250,	100,
			225,	275,	30,	150,	50,	200,	20,	125,	286,	115;
  // radius of circle
  world.vecRadius << 	14,	15,	16,	15,	12,	14,	12,	13,	15,	12,
			12,	11,	12,	14,	15,	13,	12,	15,	15,	12,
			18,	12,	16,	12,	12,	10,	10,	14,	9,	8;
  // height of building : botton
  world.vecZL <<	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
			0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
			0,	0,	0,	0,	0,	0,	0,	0,	0,	0;
			
  world.vecZH <<	20,	20,	24,	25,	22,	22,	20,	22,	27,	28,
			26,	25,	28,	20,	25,	28,	28,	20,	20,	22,
			20,	20,	20,	25,	21,	22,	20,	20,	20,	25;

  world.vecCN = world.vecCN*scale;
  world.vecCE = world.vecCE*scale;
  world.vecRadius = world.vecRadius*scale;
  world.vecZL = world.vecZL*scale;
  world.vecZH = world.vecZH*scale;
  
  return world;
}


worldRect_t CreateWorld_Rect(int iNumObstacles, Vector3d vecNEcorner, Vector3d vecSWcorner)
{
  /// create random world with obstacles
  /// the first element is the north coordinate
  /// the second element is the south coordinate
  
  worldRect_t world;
  
  int i, j;
  
  // Obstacle Number
  world.iNumObstacles = iNumObstacles;
  
  // Size of the world
  world.vecNEcorner = vecNEcorner;
  world.vecSWcorner = vecSWcorner;
  
  world.vecCN.resize(iNumObstacles);
  world.vecCE.resize(iNumObstacles);
  world.vecHorizontal.resize(iNumObstacles);
  world.vecVertical.resize(iNumObstacles);
  world.vecZ.resize(iNumObstacles);
  world.vecRot.resize(iNumObstacles);
 
  for (i=0; i<7; i++)
  {
    for (j=0; j<7; j++)
    {
      world.vecCN(7*i+j) = (i+1)*40;
      world.vecCE(7*i+j) = (j+1)*40;
      world.vecHorizontal(7*i+j) = 7;
      world.vecVertical(7*i+j) = 7;
      world.vecZ(7*i+j) = 30;
      world.vecRot(7*i+j) = 100*rand();
    }
  }
  
  return world;
}


worldLine_t CreateWorld_Line(int iNumObstacles)
{
  /// create random world with obstacles
  /// the first element is the north coordinate
  /// the second element is the south coordinate
  int i;
  
  worldLine_t world;
  vector<CURB_Segment> lineSeq;
  
  
  // Obstacle Number
  world.iNumObstacles = iNumObstacles;
  
  lineSeq.reserve(iNumObstacles);
  
  world.lineSeg = lineSeq;
  
  return world;
}

