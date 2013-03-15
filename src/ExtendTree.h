#ifndef EXTENDTREE_H_
#define EXTENDTREE_H_

#include "Struct.h"

#include "Kappa_Max_Calculation.h"
#include "Collision.h"
#include "Functions.h"

#include <math.h>
#include <iostream>

#include <ros/ros.h>

extendTreeR_t ExtendTree_NHC_Sort_OnlyGS_TermCond_Heading(MatrixXd tree, VectorXd vecEndNode, worldLine_t world, setting_t P,
				double dKmax, double c4, double dMax, double dMaxBeta, sigma_t sigma, int iMaxIte, int iINDC);

#endif /* EXTENDTREE_H_ */
