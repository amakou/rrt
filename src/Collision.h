#ifndef COLLISION_H_
#define COLLISION_H_

#include "Struct.h"
#include "CurveGeneration.h"

int Collision(VectorXd vecNode, VectorXd vecParent, worldLine_t world, double dSZR, double dSZH, int precision);

int CollisionSmooth(VectorXd vecX, VectorXd vecY, VectorXd vecZ, worldLine_t world, double dSZR, double dSZH);

#endif /* COLLISION_H_ */
