#include "CreateWorld.h"

worldLine_t CreateWorld_Line(int iNumObstacles)
{
  worldLine_t world;
  vector<CURB_Segment> lineSeq;

  lineSeq.reserve(iNumObstacles);
  
  world.iNumObstacles = iNumObstacles;
  world.lineSeg = lineSeq;
  
  return world;
}
