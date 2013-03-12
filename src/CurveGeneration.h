#ifndef CURVEGENERATION_H_
#define CURVEGENERATION_H_

#include "Struct.h"

void One_Spiral_Curve_Segmentation_Smooth_Sub_Bisect_Lr_Apply(VectorXd vecXP, VectorXd vecYP,double dGamma, setting_t P,
							      double dLmax, VectorXd &vecDataX, VectorXd &vecDataY);

dataProcessRet_t DataProcess1(VectorXd vecData, VectorXd vecDataX, VectorXd vecDataY, MatrixXd matPnew, MatrixXd matTM);

void CurvePointCalculation(VectorXd vecXP1, VectorXd vecYP1, VectorXd vecZP1, double t, VectorXd &vecP);

xbeCalRet_t XBE_Calculation(VectorXd vecXP, double d, double cg, double sg, double cb, double sb, MatrixXd matPnew, MatrixXd matTM);

void PathLineSegmentation(double XP1, double XP2, double YP1, double YP2, double ZP1, double ZP2,
			  VectorXd &vecXS, VectorXd &vecYS, VectorXd &vecZS);

curveGenRet_t CurveGeneration(VectorXd vecXP1, VectorXd vecYP1, VectorXd vecZP1, VectorXd vecT);

double CurveLength(VectorXd vecXP, VectorXd vecYP);

#endif /* CURVEGENERATION_H_ */
