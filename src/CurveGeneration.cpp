#include "CurveGeneration.h"

void One_Spiral_Curve_Segmentation_Smooth_Sub_Bisect_Lr_Apply(VectorXd vecXP, VectorXd vecYP, double dGamma, setting_t P,
							      double dLmax, VectorXd& vecDataX, VectorXd& vecDataY)
{
  VectorXd vecP1, vecP2, vecP3, vecDp1, vecDp2;
  double dN1;
  
  
  
  ///*****************///
  /// Initial setting ///
  ///*****************///
  
  vecP1.resize(2);	vecP2.resize(2);	vecP3.resize(2);
  
  vecP1 << vecXP(0), vecYP(0);
  vecP2 << vecXP(1), vecYP(1);
  vecP3 << vecXP(2), vecYP(2);
  
  // vector between two points
  vecDp1 = vecP2 - vecP1;
  vecDp2 = vecP2 - vecP3;
  
  dN1 = vecDp1.norm();
  
  
  
  ///*************************///
  /// First Curve Calculation ///
  ///*************************///
  
  double gamma, dBeta, sb, cb, sg, cg, dDkappa, dTx1;
  
  // 1. theta
  gamma = 0.5*dGamma;
    
  // 2. G2 condition
  if (dGamma > 0)
    dBeta = 0.25*dGamma;
  else if (dGamma < 0)
    dBeta = M_PI+0.25*dGamma;
  
  sb = sin(dBeta);	cb = cos(dBeta);
  sg = sin(gamma);	cg = cos(gamma);
  
  // 3. Nonholonomic Constraint
  dDkappa = dLmax/(1+1/cg);
  
  // 5. Translate the curve  : L + d
  dTx1 = dN1 - dLmax;
  
  // Middle Points //
  
  // bezier cubic spiral points
  vecXP(0) = 0;			vecYP(0) = 0;
  vecXP(1) = dDkappa;		vecYP(1) = 0;
  vecXP(2) = dDkappa*(1+cg);	vecYP(2) = dDkappa*sg;
  
  
  
  ///*********************************///
  /// Cubic Bezier Spiral Calculation ///
  ///*********************************///
  
  double h_b, h_e, g_b, g_e, k_b, k_e;
  
  h_b = 0.346*dDkappa;	h_e = h_b;
  g_b = 0.58*h_b;	g_e = g_b;
  k_b = 1.31*h_b*cb;	k_e = k_b;
  
  
  
  ///***************///
  /// Assign Values ///
  ///***************///
  
  double g_bx0, g_by0, g_ex0, g_ey0;
  VectorXd vecXP_b, vecYP_b, vecXP_e, vecYP_e;
  
  // assign values
  g_bx0 = vecXP(0);	g_by0 = vecYP(0);
  g_ex0 = vecXP(2);	g_ey0 = vecYP(2);
  
  // assign xp, yp values
  vecXP_b.resize(4);	vecYP_b.resize(4);
  vecXP_e.resize(4);	vecYP_e.resize(4);
  
  vecXP_b << g_bx0, g_bx0+g_b, g_bx0+g_b+h_b, g_bx0+g_b+h_b+k_b*cb;
  vecYP_b << g_by0, g_by0, g_by0, g_by0+k_b*sb;
  
  vecXP_e << g_ex0, g_ex0-g_e*cg, g_ex0-(g_e+h_e)*cg, g_ex0-(g_e+h_e)*cg-k_e*cb;
  vecYP_e << g_ey0, g_ey0-g_e*sg, g_ey0-(g_e+h_e)*sg, g_ey0-(g_e+h_e)*sg-k_e*cb;
  
  
  
  ///***************************///
  /// Move to original position ///
  ///***************************///
  
  // 5. Inverse Translate the curve : Tx2
  VectorXd vecTemp;
  vecTemp.resizeLike(vecXP_b);
  vecTemp.setConstant(dTx1);
  vecXP_b = vecXP_b + vecTemp;
  vecXP_e = vecXP_e + vecTemp;
  
  
  
  ///***********///
  /// Save Data ///
  ///***********///
  
  VectorXd vecData1x, vecData1y;
  
  vecData1x.resize(vecXP_b.rows()+vecXP_e.rows());
  vecData1y.resize(vecYP_b.rows()+vecYP_e.rows());
  
  vecData1x.segment(0,vecXP_b.rows()) = vecXP_b;
  vecData1x.segment(vecXP_b.rows()-1,vecXP_e.rows()) = vecXP_e;
  
  vecData1y.segment(0,vecYP_b.rows()) = vecYP_b;
  vecData1y.segment(vecYP_b.rows()-1,vecYP_e.rows()) = vecYP_e;
  
  
  
  ///**************************///
  /// Second Curve Calculation ///
  ///**************************///
  
  VectorXd vecData2x, vecData2y;
  MatrixXd matK, matTmp;
  
  /// Translation 1: move to (0,0)
  vecData2x.resizeLike(vecData1x);
  vecData2y.resizeLike(vecData1y);
  
  vecTemp.resizeLike(vecData1x);
  vecTemp.setConstant(vecXP_b(0));
  vecData2x = vecData1x - vecTemp;
  vecData2y = vecData1y;
  
  /// Rotation Matrix :  rotate to allign second curve
  MatrixXd matRM;
  
  matRM.resize(2,2);
  matRM << cg, -sg, sg, cg;
  
  // Rotation : 2
  matTmp.resize(2, vecData2x.rows());
  matTmp.topRows(1) = vecData2x.transpose();
  matTmp.bottomRows(1) = vecData2y.transpose();
  
  matK.resize(matRM.rows(), matTmp.cols());
  matK = matRM*matTmp;
  
  // x, y value
  vecData2x = matK.row(0).transpose();
  vecData2y = matK.row(1).transpose();
  
  /// Translation 2 : move to {vecXP_e(0), vecYP_e(0)}
  vecData2x = vecData2x*vecXP_e(0);
  vecData2y = vecData2y*vecYP_e(0);
  
  
  
  ///************///
  /// Final Data ///
  ///************///
  
  vecDataX.resize(vecData1x.rows()+vecData2x.rows());
  vecDataY.resize(vecData1y.rows()+vecData2y.rows());
  
  vecDataX.segment(0,vecData1x.rows()) = vecData1x;
  vecDataX.segment(vecData1x.rows()-1,vecData2x.rows()) = vecData2x;
  
  vecDataY.segment(0,vecData1y.rows()) = vecData1y;
  vecDataY.segment(vecData1y.rows()-1,vecData2y.rows()) = vecData2y;
}


dataProcessRet_t DataProcess1(VectorXd vecData, VectorXd vecDataX, VectorXd vecDataY, MatrixXd matPnew, MatrixXd matTM)
{
  dataProcessRet_t funcRet;
  
  VectorXd vecOutput, vecTemp;
  VectorXd vecXP_b1, vecXP_e1, vecXP_b2, vecXP_e2, vecXP_b3, vecXP_e3;
  VectorXd vecYP_b1, vecYP_e1, vecYP_b2, vecYP_e2, vecYP_b3, vecYP_e3;
  VectorXd vecZP_b1, vecZP_e1, vecZP_b2, vecZP_e2, vecZP_b3, vecZP_e3;
  VectorXd vecXP_eTemp1, vecXP_eTemp2, vecXP_eTemp3, vecYP_eTemp1, vecYP_eTemp2, vecYP_eTemp3, vecZP_eTemp1, vecZP_eTemp2, vecZP_eTemp3;
  
  MatrixXd matPafter1, matPbefore1;
  
  int i;
  
  matPafter1.resize(4, vecDataX.rows());
  
  if ( vecDataX.cols() == 0 )
  {
    
  }
  else if ( vecDataX.cols() == 16 )
  {
    matPafter1.resize(4, vecDataX.rows());
    vecTemp.resize(16);
    vecTemp.setConstant(1);
    
    matPafter1.row(0) = vecDataX.transpose();
    matPafter1.row(1) = vecDataY.transpose();
    matPafter1.row(2) = matPnew(2,0)*vecTemp.transpose();
    matPafter1.row(3) = vecTemp.transpose();
    
    matPbefore1.resize(matTM.rows(), matPafter1.cols());
    matPbefore1 = matTM*matPafter1;
    
    vecXP_b1.resize(4);	vecXP_e1.resize(4);
    vecXP_b2.resize(4);	vecXP_e2.resize(4);
    vecXP_b1 = matPbefore1.block(0,0,1,4).transpose();
    vecXP_e1 = matPbefore1.block(0,4,1,4).transpose();
    vecXP_b2 = matPbefore1.block(0,8,1,4).transpose();
    vecXP_e2 = matPbefore1.block(0,12,1,4).transpose();
    
    vecYP_b1.resize(4);	vecYP_e1.resize(4);
    vecYP_b2.resize(4);	vecYP_e2.resize(4);
    vecYP_b1 = matPbefore1.block(1,0,1,4).transpose();
    vecYP_e1 = matPbefore1.block(1,4,1,4).transpose();
    vecYP_b2 = matPbefore1.block(1,8,1,4).transpose();
    vecYP_e2 = matPbefore1.block(1,12,1,4).transpose();
    
    vecZP_b1.resize(4);	vecZP_e1.resize(4);
    vecZP_b2.resize(4);	vecZP_e2.resize(4);
    vecZP_b1 = matPbefore1.block(2,0,1,4).transpose();
    vecZP_e1 = matPbefore1.block(2,4,1,4).transpose();
    vecZP_b2 = matPbefore1.block(2,8,1,4).transpose();
    vecZP_e2 = matPbefore1.block(2,12,1,4).transpose();
    
    // Change the Sequence
    vecXP_eTemp1.resize(4);	vecXP_eTemp2.resize(4);
    vecYP_eTemp1.resize(4);	vecYP_eTemp2.resize(4);
    vecZP_eTemp1.resize(4);	vecZP_eTemp2.resize(4);
    
    for (i=3; i>-1; i--)
    {
      vecXP_eTemp1(3-i) = vecXP_e1(i);
      vecXP_eTemp2(3-i) = vecXP_e2(i);
      vecYP_eTemp1(3-i) = vecYP_e1(i);
      vecYP_eTemp2(3-i) = vecYP_e2(i);
      vecZP_eTemp1(3-i) = vecZP_e1(i);
      vecZP_eTemp2(3-i) = vecZP_e2(i);
    }
    
    vecXP_e1 = vecXP_eTemp1;
    vecXP_e2 = vecXP_eTemp2;
    vecYP_e1 = vecYP_eTemp1;
    vecYP_e2 = vecYP_eTemp2;
    vecZP_e1 = vecZP_eTemp1;
    vecZP_e2 = vecZP_eTemp2;
  }
  else if (vecDataX.rows()==24)
  {
    matPafter1.resize(4, vecDataX.rows());
    vecTemp.resize(24);
    vecTemp.setConstant(1);
    
    matPafter1.row(0) = vecDataX.transpose();
    matPafter1.row(1) = vecDataY.transpose();
    matPafter1.row(2) = matPnew(2,0)*vecTemp.transpose();
    matPafter1.row(3) = vecTemp.transpose();
    
    matPbefore1.resize(matTM.rows(), matPafter1.cols());
    matPbefore1 = matTM*matPafter1;
    
    vecXP_b1.resize(4);	vecXP_e1.resize(4);
    vecXP_b2.resize(4);	vecXP_e2.resize(4);
    vecXP_b3.resize(4);	vecXP_e3.resize(4);
    vecXP_b1 = matPbefore1.block(0,0,1,4).transpose();
    vecXP_e1 = matPbefore1.block(0,4,1,4).transpose();
    vecXP_b2 = matPbefore1.block(0,8,1,4).transpose();
    vecXP_e2 = matPbefore1.block(0,12,1,4).transpose();
    vecXP_b3 = matPbefore1.block(0,16,1,4).transpose();
    vecXP_e3 = matPbefore1.block(0,20,1,4).transpose();
    
    vecYP_b1.resize(4);	vecYP_e1.resize(4);
    vecYP_b2.resize(4);	vecYP_e2.resize(4);
    vecYP_b3.resize(4);	vecYP_e3.resize(4);
    vecYP_b1 = matPbefore1.block(1,0,1,4).transpose();
    vecYP_e1 = matPbefore1.block(1,4,1,4).transpose();
    vecYP_b2 = matPbefore1.block(1,8,1,4).transpose();
    vecYP_e2 = matPbefore1.block(1,12,1,4).transpose();
    vecYP_b3 = matPbefore1.block(1,16,1,4).transpose();
    vecYP_e3 = matPbefore1.block(1,20,1,4).transpose();
    
    vecZP_b1.resize(4);	vecZP_e1.resize(4);
    vecZP_b2.resize(4);	vecZP_e2.resize(4);
    vecZP_b3.resize(4);	vecZP_e3.resize(4);
    vecZP_b1 = matPbefore1.block(2,0,1,4).transpose();
    vecZP_e1 = matPbefore1.block(2,4,1,4).transpose();
    vecZP_b2 = matPbefore1.block(2,8,1,4).transpose();
    vecZP_e2 = matPbefore1.block(2,12,1,4).transpose();
    vecZP_b3 = matPbefore1.block(2,16,1,4).transpose();
    vecZP_e3 = matPbefore1.block(2,24,1,4).transpose();
    
    // Change the Sequence
    vecXP_eTemp1.resize(4);	vecXP_eTemp2.resize(4);	vecXP_eTemp3.resize(4);
    vecYP_eTemp1.resize(4);	vecYP_eTemp2.resize(4);	vecYP_eTemp3.resize(4);
    vecZP_eTemp1.resize(4);	vecZP_eTemp2.resize(4);	vecZP_eTemp3.resize(4);
    
    for (i=3; i>-1; i--)
    {
      vecXP_eTemp1(3-i) = vecXP_e1(i);
      vecXP_eTemp2(3-i) = vecXP_e2(i);
      vecXP_eTemp3(3-i) = vecXP_e3(i);
      vecYP_eTemp1(3-i) = vecYP_e1(i);
      vecYP_eTemp2(3-i) = vecYP_e2(i);
      vecYP_eTemp3(3-i) = vecYP_e3(i);
      vecZP_eTemp1(3-i) = vecZP_e1(i);
      vecZP_eTemp2(3-i) = vecZP_e2(i);
      vecZP_eTemp3(3-i) = vecZP_e3(i);
    }
    
    vecXP_e1 = vecXP_eTemp1;
    vecXP_e2 = vecXP_eTemp2;
    vecXP_e3 = vecXP_eTemp3;
    vecYP_e1 = vecYP_eTemp1;
    vecYP_e2 = vecYP_eTemp2;
    vecYP_e3 = vecYP_eTemp3;
    vecZP_e1 = vecZP_eTemp1;
    vecZP_e2 = vecZP_eTemp2;
    vecZP_e3 = vecZP_eTemp3;
  }// end if ( vecDataX.cols() == 0 )
  
  funcRet.xp_b1 = vecXP_b1;	funcRet.xp_b2 = vecXP_b2;
  funcRet.xp_e1 = vecXP_e1;	funcRet.xp_e2 = vecXP_e2;
  
  funcRet.yp_b1 = vecYP_b1;	funcRet.yp_b2 = vecYP_b2;
  funcRet.yp_e1 = vecYP_e1;	funcRet.yp_e2 = vecYP_e2;
  
  funcRet.zp_b1 = vecZP_b1;	funcRet.zp_b2 = vecZP_b2;
  funcRet.zp_e1 = vecZP_e1;	funcRet.zp_e2 = vecZP_e2;
  
  return funcRet;
}



void CurvePointCalculation(VectorXd vecXP1, VectorXd vecYP1, VectorXd vecZP1, double t, VectorXd &vecP)
{
  /// Curve Point Calculation
  double ax1, bx1, cx1, dx1;
  double ay1, by1, cy1, dy1;
  double az1, bz1, cz1, dz1;
  double Px, Py, Pz;
  
  
  
  ///*******************************///
  /// Cubic Bezier Curve Generation ///
  ///*******************************///
  
  // Coefficient of Cubic Bezier Polynomials 1
  ax1 = (-vecXP1(0)+3*vecXP1(1)-3*vecXP1(2)+vecXP1(3));
  bx1 = (3*vecXP1(0)-6*vecXP1(1)+3*vecXP1(2));
  cx1 = (-3*vecXP1(0)+3*vecXP1(1));
  dx1 = vecXP1(0);
  
  ay1 = (-vecYP1(0)+3*vecYP1(1)-3*vecYP1(2)+vecYP1(3));
  by1 = (3*vecYP1(0)-6*vecYP1(1)+3*vecYP1(2));
  cy1 = (-3*vecYP1(0)+3*vecYP1(1));
  dy1 = vecYP1(0);
  
  az1 = (-vecZP1(0)+3*vecZP1(1)-3*vecZP1(2)+vecZP1(3));
  bz1 = (3*vecZP1(0)-6*vecZP1(1)+3*vecZP1(2));
  cz1 = (-3*vecZP1(0)+3*vecZP1(1));
  dz1 = vecZP1(0);
  
  // Parametric Polynomial : Cubic Bezier Curve 1
  Px = ax1*t*t*t + bx1*t*t + cx1*t+dx1;
  Py = ay1*t*t*t + by1*t*t + cy1*t+dy1;
  Pz = az1*t*t*t + bz1*t*t + cz1*t+dz1;
  
  vecP.resize(3);
  vecP << Px, Py, Pz;
}


xbeCalRet_t XBE_Calculation(VectorXd vecXP, double d, double cg, double sg, double cb, double sb, MatrixXd matPnew, MatrixXd matTM)
{
  xbeCalRet funcRet;
  double dTx2;
  
  // Translate the curve : Tx2
  dTx2 = vecXP(1)-d;
  
  // 3 points for G2CBS
  VectorXd vecxp, vecyp, veczp;
  
  vecxp.resize(3); vecyp.resize(3); veczp.resize(3);
  
  vecxp << 0, d, d*(1+cg);
  vecyp << 0, 0, d*sg;
  
  
  
  ///*********************************///
  /// Cubic Bezier Spiral Calculation ///
  ///*********************************///
  
  double h_b, h_e, g_b, g_e, k_b, k_e;
  
  h_b=0.346*d;
  h_e=h_b;
  g_b=0.58*h_b;
  g_e=g_b;
  k_b=1.31*h_b*cb;
  k_e=k_b;
  
  
  
  ///***************///
  /// Assign Values ///
  ///***************///
  
  double g_bx0, g_by0, g_ex0, g_ey0;
  VectorXd vecXP_b, vecYP_b, vecXP_e, vecYP_e;
  
  // assign values
  g_bx0=vecxp(0);
  g_by0=vecyp(0);
  
  g_ex0=vecxp(2);
  g_ey0=vecyp(2);
  
  // assign xp, yp value
  vecXP_b.resize(4); vecYP_b.resize(4);
  vecXP_e.resize(4); vecYP_e.resize(4);
  
  vecXP_b << g_bx0, g_bx0+g_b, g_bx0+g_b+h_b, g_bx0+g_b+h_b+k_b*cb;
  vecYP_b << g_by0, g_by0, g_by0, g_by0+k_b*sb;

  vecXP_e << g_ex0, g_ex0-g_e*cg, g_ex0-(g_e+h_e)*cg, g_ex0-(g_e+h_e)*cg-k_e*cb;
  vecYP_e << g_ey0, g_ey0-g_e*sg, g_ey0-(g_e+h_e)*sg, g_ey0-(g_e+h_e)*sg-k_e*sb;
  
  
  
  ///****************************///
  /// Move to original positioin ///
  ///****************************///
  
  VectorXd vecTemp;
  
  vecTemp.resize(vecXP_b.rows());
  vecTemp.setConstant(dTx2);
  
  vecXP_b = vecXP_b+vecTemp;
  vecXP_e = vecXP_e+vecTemp;
  
  
  // 7. Original Position
  MatrixXd matPafter1, matPbefore;
  
  vecTemp.resize(8);
  vecTemp.setConstant(1);
  
  matPafter1.resize(4,8);
  
  matPafter1.row(0) << vecXP_b.transpose(), vecXP_e.transpose();
  matPafter1.row(1) << vecYP_b.transpose(), vecYP_e.transpose();  
  
  matPafter1.row(2) = matPnew(2,0)*vecTemp.transpose();
  matPafter1.row(3) = vecTemp.transpose();
  
  matPbefore = matTM*matPafter1;
  
  funcRet.xp_b = matPbefore.block(0,0,1,4).transpose();
  funcRet.xp_e = matPbefore.block(0,4,1,4).transpose();
  funcRet.yp_b = matPbefore.block(1,0,1,4).transpose();
  funcRet.yp_e = matPbefore.block(1,4,1,4).transpose();
  funcRet.zp_b = matPbefore.block(2,0,1,4).transpose();
  funcRet.zp_e = matPbefore.block(2,4,1,4).transpose();
  
  return funcRet;
}


void PathLineSegmentation(double XP1, double XP2, double YP1, double YP2, double ZP1, double ZP2, VectorXd& vecXS, VectorXd& vecYS, VectorXd& vecZS)
{
  /// Path Line Segmentation
  /// Objective : To generate segment of smooth curve
  /// Copywrite Kwang Jin Yang 18 April 2007 Version 1.0
  
  double x1, x2, x3, x4;
  double y1, y2, y3, y4;
  double z1, z2, z3, z4;
  
  // data point for generating bezier curve
  x1= XP1;	x2= XP1+(XP2-XP1)/3;	x3= XP1+2*(XP2-XP1)/3;	x4= XP2;
  y1= YP1;	y2= YP1+(YP2-YP1)/3;	y3= YP1+2*(YP2-YP1)/3;	y4= YP2; 
  z1= ZP1;	z2= ZP1+(ZP2-ZP1)/3;	z3= ZP1+2*(ZP2-ZP1)/3;	z4= ZP2;

  vecXS.resize(4);	vecYS.resize(4);	vecZS.resize(4);
  
  vecXS << x1, x2, x3, x4;
  vecYS << y1, y2, y3, y4;
  vecZS << z1, z2, z3, z4;
}


curveGenRet_t CurveGeneration(VectorXd vecXP1, VectorXd vecYP1, VectorXd vecZP1, VectorXd vecT)
{
  ///*******************************///
  /// Cubic Bezier Curve Generation ///
  ///*******************************///
  
  curveGenRet_t funcRet;
  int i;
  int len = vecT.rows();
  double a_x1, b_x1, c_x1, d_x1;
  double a_y1, b_y1, c_y1, d_y1;
  double a_z1, b_z1, c_z1, d_z1;
  
  // Coefficient of Cubic Bezier Polynomials 1
  a_x1 = (-vecXP1(0)+3*vecXP1(1)-3*vecXP1(2)+vecXP1(3));
  b_x1 = (3*vecXP1(0)-6*vecXP1(1)+3*vecXP1(2));
  c_x1 = (-3*vecXP1(0)+3*vecXP1(1));
  d_x1 = vecXP1(0);
  
  a_y1 = (-vecYP1(0)+3*vecYP1(1)-3*vecYP1(2)+vecYP1(3));
  b_y1 = (3*vecYP1(0)-6*vecYP1(1)+3*vecYP1(2));
  c_y1 = (-3*vecYP1(0)+3*vecYP1(1));
  d_y1 = vecYP1(0);
  
  a_z1 = (-vecZP1(0)+3*vecZP1(1)-3*vecZP1(2)+vecZP1(3));
  b_z1 = (3*vecZP1(0)-6*vecZP1(1)+3*vecZP1(2));
  c_z1 = (-3*vecZP1(0)+3*vecZP1(1));
  d_z1 = vecZP1(0);
  
  // Parametric Polynomial : Cubic Bezier Curve 1
  VectorXd vecPx(len), vecPy(len), vecPz(len);
  
  for (i=0; i<len; i++)
  {
    vecPx(i) = a_x1*vecT(i)*vecT(i)*vecT(i)+b_x1*vecT(i)*vecT(i)+c_x1*vecT(i)+d_x1;
    vecPy(i) = a_y1*vecT(i)*vecT(i)*vecT(i)+b_y1*vecT(i)*vecT(i)+c_y1*vecT(i)+d_y1;
    vecPz(i) = a_z1*vecT(i)*vecT(i)*vecT(i)+b_z1*vecT(i)*vecT(i)+c_z1*vecT(i)+d_z1;
  }
  
  
  
  ///*************************///
  /// Derivative of the Curve ///
  ///*************************///
  
  VectorXd vecDPx1(len), vecDPy1(len), vecDPz1(len);
  VectorXd vecDDPx1(len), vecDDPy1(len), vecDDPz1(len);
  VectorXd vecPsi(len);
    
  for (i=0; i<len; i++)
  {
    // First derivative of P(t) : Curve 1
    vecDPx1(i) = 3*a_x1*vecT(i)*vecT(i)+2*b_x1*vecT(i)+c_x1;
    vecDPy1(i) = 3*a_y1*vecT(i)*vecT(i)+2*b_y1*vecT(i)+c_y1;
    vecDPz1(i) = 3*a_z1*vecT(i)*vecT(i)+2*b_z1*vecT(i)+c_z1;
    
    // Second derivative of P(t) : Curve 1
    vecDDPx1(i) = 6*a_x1*vecT(i)+2*b_x1;
    vecDDPy1(i) = 6*a_y1*vecT(i)+2*b_y1;
    vecDDPz1(i) = 6*a_z1*vecT(i)+2*b_z1;
    
    // Heading of the Curve
    vecPsi(i) = atan2(vecDPy1(i), vecDPx1(i));
  }
  
  
  
  ///***********************///
  /// Curvature Calculation ///
  ///***********************///
  
  VectorXd vecA1(len), vecB1(len), vecKappa1(len);
  VectorXd vecA2(len), vecB2(len), vecKappa2(len);
  VectorXd vecA3(len), vecB3(len), vecKappa3(len);
  
  /// Plane curvature calculation
  for (i=0; i<len; i++)
  {
    // x-y plane
    vecA1(i) = vecDPx1(i)*vecDPx1(i)+vecDPy1(i)*vecDPy1(i);
    vecB1(i) = vecDPx1(i)*vecDDPy1(i)-vecDPy1(i)*vecDDPx1(i);
    vecKappa1(i) = vecB1(i) / sqrt(vecA1(i)*vecA1(i)*vecA1(i));
    
    // y-z plane
    vecA2(i) = vecDPy1(i)*vecDPy1(i)+vecDPz1(i)*vecDPz1(i);
    vecB2(i) = vecDPy1(i)*vecDDPz1(i)-vecDPz1(i)*vecDDPy1(i);
    vecKappa2(i) = vecB2(i) / sqrt(vecA2(i)*vecA2(i)*vecA2(i));
    
    // z-x plane
    vecA3(i) = vecDPz1(i)*vecDPz1(i)+vecDPx1(i)*vecDPx1(i);
    vecB3(i) = vecDPz1(i)*vecDDPx1(i)-vecDPx1(i)*vecDDPz1(i);
    vecKappa3(i) = vecB3(i) / sqrt(vecA3(i)*vecA3(i)*vecA3(i));
  }
  
  
  
  /// Plane curvature calculation2
  VectorXd vecKappa4(len);
  double dNum, dDen;
  
  for (i=0; i<len; i++)
  {
    dNum = sqrt((vecDPx1(i)*vecDDPy1(i)-vecDPy1(i)*vecDDPx1(i))*(vecDPx1(i)*vecDDPy1(i)-vecDPy1(i)*vecDDPx1(i)) +
	        (vecDPy1(i)*vecDDPz1(i)-vecDPz1(i)*vecDDPy1(i))*(vecDPy1(i)*vecDDPz1(i)-vecDPz1(i)*vecDDPy1(i)) +
		(vecDPz1(i)*vecDDPx1(i)-vecDPx1(i)*vecDDPz1(i))*(vecDPz1(i)*vecDDPx1(i)-vecDPx1(i)*vecDDPz1(i)) );
    
    dDen = sqrt(vecDPx1(i)*vecDPx1(i)+vecDPy1(i)*vecDPy1(i)+vecDPz1(i)*vecDPz1(i));
    
    if( vecKappa1(i) > 0)
      vecKappa4(i) = dNum/(dDen*dDen*dDen);
    else
      vecKappa4(i) = -dNum/(dDen*dDen*dDen);
  }
  
  
  
  funcRet.kappa1 = vecKappa1;
  funcRet.kappa2 = vecKappa2;
  funcRet.kappa3 = vecKappa3;
  funcRet.kappa4 = vecKappa4;
  funcRet.P_x = vecPx;
  funcRet.P_y = vecPy;
  funcRet.P_z = vecPz;
  funcRet.Psi = vecPsi;
  
  return funcRet;
}

double CurveLength(VectorXd vecXP, VectorXd vecYP)
{
  /// Objective : Calculate the maximum curvature and arc length of bezier curve  
  /// Copywrite Kwang Jin YANG 7 Feb 2007 Version 1.0
  
  /// Paper : Adaptive subdivision and the length and energy of bezier curve(1997)
  
  int d = 3;
  double b, b1, b2, b3;
  
  b = sqrt( (vecXP(3)-vecXP(0))*(vecXP(3)-vecXP(0)) + (vecYP(3)-vecYP(0))*(vecYP(3)-vecYP(0)) );
  b1 = sqrt( (vecXP(1)-vecXP(0))*(vecXP(1)-vecXP(0)) + (vecYP(1)-vecYP(0))*(vecYP(1)-vecYP(0)) );
  b2 = sqrt( (vecXP(2)-vecXP(1))*(vecXP(2)-vecXP(1)) + (vecYP(2)-vecYP(1))*(vecYP(2)-vecYP(1)) );
  b3 = sqrt( (vecXP(3)-vecXP(2))*(vecXP(3)-vecXP(2)) + (vecYP(3)-vecYP(2))*(vecYP(3)-vecYP(2)) );
  
  // Chord length
  double dLc;
  
  dLc = b;
  
  // Total polygon length
  double dLp;
  
  dLp = b1+b2+b3;
  
  // Arc length of bezier curve
  double dL_app;
  
  dL_app = (2*dLc+(d-1)*dLp)/(d+1);
  
  return dL_app;
}



