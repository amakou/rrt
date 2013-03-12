#ifndef CREATE_WORLD_H_
#define CREATE_WORLD_H_

#include "Struct.h"

using namespace Eigen;

world_t CreateWorld_TRO(int iNumObstacles, Vector3d vecNEcorner, Vector3d vecSWcorner);

world_t CreateWorld_Obstacles(int iNumObstacles, Vector3d vecNEcorner, Vector3d vecSWcorner);

worldRect_t CreateWorld_Rect(int iNumObstacles, Vector3d vecNEcorner, Vector3d vecSWcorner);

worldLine_t CreateWorld_Line(int iNumObstacles);

#endif /* CREATE_WORLD_H_ */
