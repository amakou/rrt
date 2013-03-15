#ifndef STRUCT_H_
#define STRUCT_H_

//#include "../Eigen/Dense"
#include <Eigen/Dense>
#include "URB_Line.h"

#include <vector>

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using namespace std;

typedef struct worldLine {
	// iNumObstacles ; number of obstacles
	int iNumObstacles;
	
	// vecNEcorner : NE corner, vecSWcorner : SW corner
	Vector3d vecNEcorner, vecSWcorner;
	
	vector<CURB_Segment> lineSeg;
} worldLine_t;

// Setting Properties
typedef struct setting {
	// L : RRT tree extensin length
	// SZR : Safety Zone of Radius	SZH : Safety Zone of Height
	// d : smoothing Length of Cubic Bezier Spiral
	// ds : smoothing Length of Simple Cubic Bezier Curve
	// kappa : maximum curvature of the path
	// INC : Increment of d			Ind : Indicator
	double L, SZR, SZH, d, ds, kappa, INC, Ind;
} setting_t;

// Sigma Structure
typedef struct sigma {
	double r, theta0, theta;
	Vector3d sp; // Start point
} sigma_t;


// ExtendTree Return
typedef struct extendTreeR {
  MatrixXd newTree;
  int flag;
  int INDC;
  sigma_t sigma;
  int maxIte;
  int ii;
  int jj;
} extendTreeR_t;


// Info
typedef struct info {
  VectorXd L;
  MatrixXd d_kappa;
  VectorXd Lmax;
  vector<MatrixXd> TM, Pnew;
  VectorXd gamma, beta, SC;
} info_t;


// FullSpiralCurve Return
typedef struct sprialCurveRet {
  MatrixXd PathSegX, PathSegY, PathSegZ;
  VectorXd P_x, P_y, P_z, Psi, Kappa1, Kappa2, Kappa3, Kappa4, L;
  int nn;
} spiralCurveRet_t;


// FullSpiralSegmentation Return
typedef struct spiralSegRet {
  MatrixXd PathSegX, PathSegY, PathSegZ;
} spiralSegRet_t;


// Data Process Return
typedef struct dataProcessRet {
  VectorXd xp_b1, yp_b1, zp_b1;
  VectorXd xp_e1, yp_e1, zp_e1;
  VectorXd xp_b2, yp_b2, zp_b2;
  VectorXd xp_e2, yp_e2, zp_e2;
} dataProcessRet_t;


// XBE Calculation Return
typedef struct xbeCalRet {
  VectorXd xp_b, yp_b, zp_b;
  VectorXd xp_e, yp_e, zp_e;
} xbeCalRet_t;


// Curve Generation Return
typedef struct curveGenRet {
  VectorXd P_x, P_y, P_z, Psi, kappa1, kappa2, kappa3, kappa4;
} curveGenRet_t;
#endif /* STRUCT_H_ */
