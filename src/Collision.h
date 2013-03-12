#ifndef COLLISION_H_
#define COLLISION_H_

#include "Struct.h"
#include "CurveGeneration.h"

int Collision(VectorXd vecNode, VectorXd vecParent, world_t world, double dSZR, double dSZH);

int Collision(VectorXd vecNode, VectorXd vecParent, worldRect_t world, double dSZR, double dSZH);

int Collision(VectorXd vecNode, VectorXd vecParent, worldLine_t world, double dSZR, double dSZH);

int Collision10(VectorXd vecNode, VectorXd vecParent, world_t world, double dSZR, double dSZH);

int Collision10(VectorXd vecNode, VectorXd vecParent, worldRect_t world, double dSZR, double dSZH);

int Collision10(VectorXd vecNode, VectorXd vecParent, worldLine_t world, double dSZR, double dSZH);

int CollisionPoint(VectorXd vecNode, world_t world, double dSZR, double dSZH);

int CollisionSmooth(VectorXd vecX, VectorXd vecY, VectorXd vecZ, world_t world, double dSZR, double dSZH);

int CollisionSmooth(VectorXd vecX, VectorXd vecY, VectorXd vecZ, worldRect_t world, double dSZR, double dSZH);

int CollisionSmooth(VectorXd vecX, VectorXd vecY, VectorXd vecZ, worldLine_t world, double dSZR, double dSZH);

#endif /* COLLISION_H_ */
