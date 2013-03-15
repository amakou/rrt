#ifndef CURVEGENERATIONTEMPLATE_H_
#define CURVEGENERATIONTEMPLATE_H_

#include "Struct.h"
#include "CurveGeneration.h"
#include "Collision.h"

template <typename T>
spiralCurveRet_t Full_Spiral_Curve_Path_L(T world, MatrixXd matFinalPoint, VectorXd vecT, setting_t P, info_t INFO)
{
	spiralCurveRet_t funcRet;
	
	/// Full Spiral Curve Path  
	MatrixXd matPathSegX, matPathSegY, matPathSegZ;
	
	spiralSegRet_t fullSpiralSegRet;
	
	
	/// Control points of full path
	fullSpiralSegRet = Full_Spiral_Segmentation_L(world, matFinalPoint, vecT, P, INFO);  
	
	matPathSegX = fullSpiralSegRet.PathSegX;
	matPathSegY = fullSpiralSegRet.PathSegY;
	matPathSegZ = fullSpiralSegRet.PathSegZ;
	
	
	
	/// ******************************** ///
	///  Remove the which has same data  ///
	/// ******************************** ///
	
	/// Detect the index having same data
	int i, n, ind;
	VectorXd vecIND;
	
	n = matPathSegX.rows();
	vecIND.resize(0);
	
	for (i=0; i<n; i++)
	{
		if ( (abs(matPathSegX(i,0)-matPathSegX(i,1))>1e-6) || (abs(matPathSegX(i,1)-matPathSegX(i,2))>1e-6) ||
				 (abs(matPathSegX(i,2)-matPathSegX(i,3))>1e-6) ||
				 (abs(matPathSegY(i,0)-matPathSegY(i,1))>1e-6) || (abs(matPathSegY(i,1)-matPathSegY(i,2))>1e-6) ||
				 (abs(matPathSegY(i,2)-matPathSegY(i,3))>1e-6) )
		{
			ind = i;
			
			vecIND.conservativeResize(vecIND.rows()+1);
			
			vecIND(vecIND.rows()-1) = ind;
		}
	}
	
	/// new matrix removing same data
	MatrixXd matPathSegXX, matPathSegYY, matPathSegZZ;
	int nn;
	
	nn = vecIND.rows();
	
	matPathSegXX.resize(nn, matPathSegX.cols());
	matPathSegYY.resize(nn, matPathSegY.cols());
	matPathSegZZ.resize(nn, matPathSegZ.cols());
	
	for ( i=0; i<nn; i++)
		matPathSegXX.row(i) = matPathSegX.row(vecIND(i));
	
	for ( i=0; i<nn; i++)
		matPathSegYY.row(i) = matPathSegY.row(vecIND(i));
	
	for ( i=0; i<nn; i++)
		matPathSegZZ.row(i) = matPathSegZ.row(vecIND(i));
	
	matPathSegX = matPathSegXX;
	matPathSegY = matPathSegYY;
	matPathSegZ = matPathSegZZ;
	
	

	/// ********************** ///
	///  All of the Path Data  ///
	/// ********************** ///
	
	curveGenRet_t curvGenRet;
	VectorXd vecPx(0), vecPy(0), vecPz(0), vecPsi(0), vecKappa1(0), vecKappa2(0), vecKappa3(0), vecKappa4(0);
	int k;
	
	nn = matPathSegX.rows();
	
	for ( k=0; k<nn; k++ )
	{
		curvGenRet = CurveGeneration(matPathSegX.row(k), matPathSegY.row(k), matPathSegZ.row(k), vecT);
		
		vecPx.conservativeResize(vecPx.rows()+curvGenRet.P_x.rows());
		vecPx.tail(curvGenRet.P_x.rows()) = curvGenRet.P_x;
		
		vecPy.conservativeResize(vecPy.rows()+curvGenRet.P_y.rows());
		vecPy.tail(curvGenRet.P_y.rows()) = curvGenRet.P_y;
		
		vecPz.conservativeResize(vecPz.rows()+curvGenRet.P_z.rows());
		vecPz.tail(curvGenRet.P_z.rows()) = curvGenRet.P_z;
		
		vecPsi.conservativeResize(vecPsi.rows()+curvGenRet.Psi.rows());
		vecPsi.tail(curvGenRet.Psi.rows()) = curvGenRet.Psi;
		
		vecKappa1.conservativeResize(vecKappa1.rows()+curvGenRet.kappa1.rows());
		vecKappa1.tail(curvGenRet.kappa1.rows()) = curvGenRet.kappa1;
		
		vecKappa2.conservativeResize(vecKappa2.rows()+curvGenRet.kappa2.rows());
		vecKappa2.tail(curvGenRet.kappa2.rows()) = curvGenRet.kappa2;
		
		vecKappa3.conservativeResize(vecKappa3.rows()+curvGenRet.kappa3.rows());
		vecKappa3.tail(curvGenRet.kappa3.rows()) = curvGenRet.kappa3;
		
		vecKappa4.conservativeResize(vecKappa4.rows()+curvGenRet.kappa4.rows());
		vecKappa4.tail(curvGenRet.kappa4.rows()) = curvGenRet.kappa4;    
	}
	
	
	
	/// ********************* ///
	///  Length of the Curve  ///
	/// ********************* ///
	
	VectorXd vecL(0);
	double dL_a;
	
	for (i=0; i<nn; i++)
	{
		dL_a = CurveLength(matPathSegX.row(i), matPathSegY.row(i) );
		
		vecL.conservativeResize(vecL.rows()+1);
		vecL(vecL.rows()-1) = dL_a;
	}
	
	funcRet.Kappa1 = vecKappa1;
	funcRet.Kappa2 = vecKappa2;
	funcRet.Kappa3 = vecKappa3;
	funcRet.Kappa4 = vecKappa4;
	funcRet.L = vecL;
	funcRet.nn = nn;
	funcRet.P_x = vecPx;
	funcRet.P_y = vecPy;
	funcRet.P_z = vecPz;
	funcRet.PathSegX = matPathSegX;
	funcRet.PathSegY = matPathSegY;
	funcRet.PathSegZ = matPathSegZ;
	funcRet.Psi = vecPsi;
	
	return funcRet;
}


template <typename T>
spiralSegRet_t Full_Spiral_Segmentation_L(T world, MatrixXd matFinalPoint, VectorXd vecT, setting_t P, info_t INFO)
{
	spiralSegRet_t funcRet;
	MatrixXd matData;



	/// ******************************************** ///
	///  All of the Cubic Bezier Curve Segmentation  ///
	/// ******************************************** ///

	matData = Partial_Spiral_Path_Segmentation_L(world, matFinalPoint, vecT, P, INFO);



	/// ************************* ///
	///  First line segmentation  ///
	/// ************************* ///

	double dx1s, dx2s, dy1s, dy2s, dz1s, dz2s;
	VectorXd vecX1s, vecY1s, vecZ1s;

	dx1s = matFinalPoint(0,0);
	dx2s = matData(0,0);
	dy1s = matFinalPoint(0,1);
	dy2s = matData(0,4);
	dz1s = matFinalPoint(0,2);
	dz2s = matData(0,8);

	PathLineSegmentation(dx1s, dx2s, dy1s, dy2s, dz1s, dz2s, vecX1s, vecY1s, vecZ1s);



	/// ************************ ///
	///  Last line segmentation  ///
	/// ************************ ///

	double dx11s, dx22s, dy11s, dy22s, dz11s, dz22s;
	VectorXd vecX1e, vecY1e, vecZ1e;
	int end, end2;

	end = matData.rows()-1;	end2 = matFinalPoint.rows()-1;

	dx11s = matData(end, 3);
	dx22s = matFinalPoint(end2, 0);
	dy11s = matData(end, 7);
	dy22s = matFinalPoint(end2, 1);
	dz11s = matData(end, 11);
	dz22s = matFinalPoint(end2, 2);

	PathLineSegmentation(dx11s, dx22s, dy11s, dy22s, dz11s, dz22s, vecX1e, vecY1e, vecZ1e);



	/// ********************************************** ///
	///  Line Segmentation between Cubic Bezier Curve  ///
	/// ********************************************** ///

	int n, i;
	double dXP1, dXP2, dYP1, dYP2, dZP1, dZP2;

	VectorXd vecxs, vecys, veczs;
	MatrixXd matXs, matYs, matZs, matTempData;

	n = matData.rows();

	matXs.resize(n-1, 4);
	matYs.resize(n-1, 4);
	matZs.resize(n-1, 4);
	matTempData.resize(0,0);

	for (i=0; i<n-1; i++)
	{
		dXP1 = matData(i, 3);
		dXP2 = matData(i+1, 0);
		dYP1 = matData(i, 7);
		dYP2 = matData(i+1, 4);
		dZP1 = matData(i, 11);
		dZP2 = matData(i+1, 8);

		if ( abs(matData(i,3)-matData(i+1,0)) > 1e-1 )
		{
			PathLineSegmentation(dXP1, dXP2, dYP1, dYP2, dZP1, dZP2, vecxs, vecys, veczs);

			matXs.row(i) = vecxs.transpose();
			matYs.row(i) = vecys.transpose();
			matZs.row(i) = veczs.transpose();

			matTempData.conservativeResize(matTempData.rows()+2, 12);

			matTempData.row(matTempData.rows()-2) = matData.block(i,0,1,12);
			matTempData.block(matTempData.rows()-1, 0, 1, 4) = matXs.row(i);
			matTempData.block(matTempData.rows()-1, 4, 1, 4) = matYs.row(i);
			matTempData.block(matTempData.rows()-1, 8, 1, 4) = matZs.row(i);
		}
		else
		{
			matTempData.conservativeResize(matTempData.rows()+1, 12);
			matTempData.bottomRows(1) = matData.block(i,0,1,12);
		}
	}// end for

	MatrixXd matFinData;

	matFinData.resize(matTempData.rows()+3, 12);

	matFinData.topRows(1) << vecX1s.transpose(), vecY1s.transpose(), vecZ1s.transpose();
	matFinData.block(1, 0, matTempData.rows(), 12) = matTempData;
	matFinData.bottomRows(2) << matData.block(matData.rows()-1, 0, 1, 12),
	vecX1e.transpose(), vecY1e.transpose(), vecZ1e.transpose();


	int ns = matFinData.rows();

	funcRet.PathSegX.resize(ns,4);
	funcRet.PathSegY.resize(ns,4);
	funcRet.PathSegZ.resize(ns,4);

	for (i=0; i<ns; i++)
	{
		funcRet.PathSegX.row(i) = matFinData.block(i,0,1,4);
		funcRet.PathSegY.row(i) = matFinData.block(i,4,1,4);
		funcRet.PathSegZ.row(i) = matFinData.block(i,8,1,4);
	}

	return funcRet;
}


template <typename T>
MatrixXd Partial_Spiral_Path_Segmentation_L(T world, MatrixXd matPath, VectorXd vecT, setting_t P, info_t INFO)
{
	MatrixXd matData, matRetData;
	VectorXd vecXP, vecYP, vecZP;

	vecXP.resize(matPath.rows());	vecYP.resize(matPath.rows());	vecZP.resize(matPath.rows());
	vecXP = matPath.col(0);	vecYP = matPath.col(1);		vecZP = matPath.col(2);

	// Iteration number
	int n, i;

	n = vecXP.rows()-2;

	matRetData.resize(0,0);

	/// Iteration for calculate whold of the cubic bezier spiral
	for (i=0; i<n; i++)
	{
		matData = One_Spiral_Curve_Segmentation_Smooth_L1(world, vecXP.segment(i,3), vecYP.segment(i,3),
									vecZP.segment(i,3), vecT, P, INFO, i);

		matRetData.conservativeResize(matRetData.rows()+matData.rows(), matData.cols());

		matRetData.bottomRows(matData.rows()) = matData;
		//i = i+1;
	}

	return matRetData;
}


template <typename T>
MatrixXd One_Spiral_Curve_Segmentation_Smooth_L1(T world, VectorXd vecXP3, VectorXd vecYP3, VectorXd vecZP3,
						 VectorXd vecT, setting_t P, info_t INFO, int i)
{
	VectorXd vecP2, vecXP, vecYP;
	double d, dLmax, dGamma, dBeta;
	MatrixXd matTM, matPnew;

	// Middle Point Value
	vecP2.resize(3);
	vecP2 << vecXP3(1), vecYP3(1), vecZP3(1);

	// Use data calculated before
	d = INFO.d_kappa(i,0);
	dLmax = INFO.Lmax(i,0);
	matTM.resizeLike(INFO.TM[i]);
	matTM = INFO.TM[i];
	matPnew.resizeLike(INFO.Pnew[i]);
	matPnew = INFO.Pnew[i];
	dGamma = INFO.gamma(i);
	dBeta = INFO.beta(i);

	// Calculate sine, cosine
	double sb, cb, sg, cg;
	sb = sin(dBeta);	cb = cos(dBeta);
	sg = sin(dGamma);	cg = cos(dGamma);

	// Use only x,y
	vecXP.resize(3);	vecYP.resize(3);
	vecXP = matPnew.block(0,0,1,3).transpose();
	vecYP = matPnew.block(1,0,1,3).transpose();



	/// *********** ///
	///  IF clause  ///
	/// *********** ///

	dataProcessRet_t dataProcRet;
	xbeCalRet_t xbeCalRet;

	VectorXd vecDataX, vecDataY, vecData;
	VectorXd vecXPb1, vecYPb1, vecZPb1;
	VectorXd vecXPe1, vecYPe1, vecZPe1;
	VectorXd vecXPb2, vecYPb2, vecZPb2;
	VectorXd vecXPe2, vecYPe2, vecZPe2;
	VectorXd vecXPb, vecYPb, vecZPb, vecXPe, vecYPe, vecZPe;

	vecXPb1.resize(4);
	vecXPb1(0) = 1e5;

	if (INFO.d_kappa(i,1)>0)
	{
		//vecXPb1.resize(4);
		vecYPb1.resize(4);	vecZPb1.resize(4);
		vecXPe1.resize(4);	vecYPe1.resize(4);	vecZPe1.resize(4);
		vecXPb2.resize(4);	vecYPb2.resize(4);	vecZPb2.resize(4);
		vecXPe2.resize(4);	vecYPe1.resize(4);	vecZPe2.resize(4);

		One_Spiral_Curve_Segmentation_Smooth_Sub_Bisect_Lr_Apply(vecXP, vecYP, dGamma, P, dLmax, vecDataX, vecDataY);
		dataProcRet = DataProcess1(vecData, vecDataX, vecDataY, matPnew, matTM);

		vecXPb1 = dataProcRet.xp_b1;	vecYPb1 = dataProcRet.yp_b1;	vecZPb1 = dataProcRet.zp_b1;
		vecXPe1 = dataProcRet.xp_e1;	vecYPe1 = dataProcRet.yp_e1;	vecZPe1 = dataProcRet.zp_e1;
		vecXPb2 = dataProcRet.xp_b2;	vecYPb2 = dataProcRet.yp_b2;	vecZPb1 = dataProcRet.zp_b2;
		vecXPe2 = dataProcRet.xp_e2;	vecYPe2 = dataProcRet.yp_e2;	vecZPe2 = dataProcRet.zp_e2;

		while ( (CollisionSmooth(dataProcRet.xp_b1, dataProcRet.yp_b1, dataProcRet.zp_b1, world, P.SZR, P.SZH)==1) |
			(CollisionSmooth(dataProcRet.xp_e1, dataProcRet.yp_e1, dataProcRet.zp_e1, world, P.SZR, P.SZH)==1) |
			(CollisionSmooth(dataProcRet.xp_b2, dataProcRet.yp_b2, dataProcRet.zp_b2, world, P.SZR, P.SZH)==1) |
			(CollisionSmooth(dataProcRet.xp_e2, dataProcRet.yp_e2, dataProcRet.zp_e2, world, P.SZR, P.SZH)==1) )
		{
			dLmax = dLmax - 0.5*P.INC;

			One_Spiral_Curve_Segmentation_Smooth_Sub_Bisect_Lr_Apply(vecXP,vecYP, dGamma, P, dLmax, vecDataX, vecDataY);
			dataProcRet = DataProcess1(vecData, vecDataX, vecDataY, matPnew, matTM);

			vecXPb1 = dataProcRet.xp_b1;	vecYPb1 = dataProcRet.yp_b1;	vecZPb1 = dataProcRet.zp_b1;
			vecXPe1 = dataProcRet.xp_e1;	vecYPe1 = dataProcRet.yp_e1;	vecZPe1 = dataProcRet.zp_e1;
			vecXPb2 = dataProcRet.xp_b2;	vecYPb2 = dataProcRet.yp_b2;	vecZPb1 = dataProcRet.zp_b2;
			vecXPe2 = dataProcRet.xp_e2;	vecYPe2 = dataProcRet.yp_e2;	vecZPe2 = dataProcRet.zp_e2;
		}
	}
	else
	{
		vecXPb.resize(4);	vecYPb.resize(4);	vecZPb.resize(4);
		vecXPe.resize(4);	vecYPe.resize(4);	vecZPe.resize(4);

		xbeCalRet = XBE_Calculation(vecXP, d, cg, sg, cb, sb, matPnew, matTM);

		vecXPb = xbeCalRet.xp_b;	vecYPb = xbeCalRet.yp_b;	vecZPb = xbeCalRet.zp_b;
		vecXPe = xbeCalRet.xp_e;	vecYPe = xbeCalRet.yp_e;	vecZPe = xbeCalRet.zp_e;

		/// Check if the d_kappa is close to Lmax

		if ( (dLmax-d)<P.INC )
			d = dLmax;

		/// Second If : whether there is a collision in d = d_kappa
		if ( (CollisionSmooth(vecXPb, vecYPb, vecZPb, world, P.SZR, P.SZH) == 1) |
			 (CollisionSmooth(vecXPe, vecYPe, vecZPe, world, P.SZR, P.SZH) == 1) )
		{
			while ( (CollisionSmooth(vecXPb, vecYPb, vecZPb, world, P.SZR, P.SZH)==1) |
				(CollisionSmooth(vecXPe, vecYPe, vecZPe, world, P.SZR, P.SZH)==1) )
			{
				d = d-0.5*P.INC;

				xbeCalRet = XBE_Calculation(vecXP, d, cg, sg, cb, sb, matPnew, matTM);

				vecXPb = xbeCalRet.xp_b;	vecYPb = xbeCalRet.yp_b;	vecZPb = xbeCalRet.zp_b;
				vecXPe = xbeCalRet.xp_e;	vecYPe = xbeCalRet.yp_e;	vecZPe = xbeCalRet.zp_e;
			}
		}
		else
		{
			while ( (CollisionSmooth(vecXPb, vecYPb, vecZPb, world, P.SZR, P.SZH)==0) &
				(CollisionSmooth(vecXPe, vecYPe, vecZPe, world, P.SZR, P.SZH)==0) &
				( P.Ind < (dLmax - 1e-6)) )
			{
				/// Condition
				if ( d == dLmax )
					d = dLmax;
				else if ( P.Ind < dLmax-P.INC )
					d = (d+P.INC);
				else if ( (P.Ind > dLmax - P.INC) & (P.Ind < dLmax) )
					d = dLmax;

				xbeCalRet = XBE_Calculation(vecXP, d, cg, sg, cb, sb, matPnew, matTM);

				vecXPb = xbeCalRet.xp_b;	vecYPb = xbeCalRet.yp_b;	vecZPb = xbeCalRet.zp_b;
				vecXPe = xbeCalRet.xp_e;	vecYPe = xbeCalRet.yp_e;	vecZPe = xbeCalRet.zp_e;

				VectorXd vecTempXYZ;

				vecTempXYZ.resize(3);

				vecTempXYZ << vecXPb(0), vecYPb(0), vecZPb(0);

				P.Ind = (vecP2 - vecTempXYZ).norm();
			} // end of while
		} // end of if



		/// ********************* ///
		///  Change the Sequence  ///
		/// ********************* ///

		VectorXd vecXPe_Temp, vecYPe_Temp, vecZPe_Temp;

		vecXPe_Temp.resize(4);
		vecYPe_Temp.resize(4);
		vecZPe_Temp.resize(4);

		for (i=3; i>-1; i--)
		{
			vecXPe_Temp(3-i) = vecXPe(i);
			vecYPe_Temp(3-i) = vecYPe(i);
			vecZPe_Temp(3-i) = vecZPe(i);
		}

		vecXPe = vecXPe_Temp;
		vecYPe = vecYPe_Temp;
		vecZPe = vecZPe_Temp;
	}// end of if (INFO.d_kappa(i,1)>0)



	/// ***************** ///
	///  Data Processing  ///
	/// ***************** ///

	MatrixXd matData;

	if (vecXPb1(0) != 1e5 )
	{
		matData.resize(4, vecXPb1.rows()+ vecYPb1.rows()+vecZPb1.rows());

		matData.row(0) = vecXPb1.transpose(), vecYPb1.transpose(), vecZPb1.transpose();
		matData.row(1) = vecXPe1.transpose(), vecYPe1.transpose(), vecZPe1.transpose();
		matData.row(2) = vecXPb2.transpose(), vecYPb2.transpose(), vecZPb2.transpose();
		matData.row(3) = vecXPe2.transpose(), vecYPe2.transpose(), vecZPe2.transpose();
	}
	else
	{
		matData.resize(2, vecXPb.rows()+vecYPb.rows()+vecZPb.rows());

		matData.row(0) << vecXPb.transpose(), vecYPb.transpose(), vecZPb.transpose();
		matData.row(1) << vecXPe.transpose(), vecYPe.transpose(), vecZPe.transpose();
	}

	return matData;
}

#endif /* CURVEGENERATIONTEMPLATE_H_ */
