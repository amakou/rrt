#include "MaxLengthCalculation.h"

info_t MaxLengthCalculationNormal(MatrixXd matPath, setting_t P, double dMax)
{
	int n, i;
	MatrixXd matWP, matDp3, matXPi, matAZi, matZPi, matYZi, matYPi, matTmp;
	vector<MatrixXd> vecTM, vecPnew;
	VectorXd vecL3;
	Vector3d vec3tmp, vec3tmp2;
	info_t Data;
	
	// Iteration number
	n = matPath.rows();
	
	matWP.resize(n,4); // WP = [XP, YP, ZP, 1];
	
	for (i=0; i<n; i++)
		matWP.row(i) << matPath(i,0), matPath(i,1), matPath(i,2), 1;
	
	
	
	/// Distance between two waypoints in 3D space
	matDp3.resize(n-1, matWP.cols());
	vecL3.resize(n-1);
	
	for (i=0; i<n-1; i++)
	{
		matDp3.row(i) = matWP.row(i+1) - matWP.row(i);
		vecL3(i) = matDp3.row(i).norm();
	}
	
		
	/// Transformation matrix calculation
	matXPi.resize(n-2, 3);	matAZi.resize(n-2, 3);
	matZPi.resize(n-2, 3);	matYZi.resize(n-2, 3);
	matYPi.resize(n-2, 3);
	vecTM.resize(n-2);		vecPnew.resize(n-2);
	vec3tmp.resize(3);		vec3tmp2.resize(3);
	
	for (i=0; i<n-2; i++)
	{
		// XPi Calculation
		matXPi.row(i) = matDp3.block(i,0,1,3)/vecL3(i);
		
		// ZPi Calculation
		vec3tmp = matDp3.block(i,0,1,3).transpose();
		vec3tmp2 = matDp3.block(i+1,0,1,3).transpose();
		
		matAZi.row(i) = vec3tmp.cross(vec3tmp2);
		matZPi.row(i) = matAZi.block(i,0,1,3)/(matAZi.block(i,0,1,3).norm());
		
		// YPi Calculation
		vec3tmp = matZPi.block(i,0,1,3).transpose();
		vec3tmp2 = matXPi.block(i,0,1,3).transpose();
		matYZi.row(i) = vec3tmp.cross(vec3tmp2);
		matYPi.row(i) = matYZi.block(i,0,1,3)/(matYZi.block(i,0,1,3).norm());
		
		// Transformation Maatrix
		vecTM[i].resize(4,4);
		vecTM[i].block(0,0,3,1) = matXPi.row(i).transpose();	vecTM[i](3,0) = 0;
		vecTM[i].block(0,1,3,1) = matYPi.row(i).transpose();	vecTM[i](3,1) = 0;
		vecTM[i].block(0,2,3,1) = matZPi.row(i).transpose();	vecTM[i](3,2) = 0;
		vecTM[i].block(0,3,4,1) = matWP.row(i).transpose();
		// New Positioin : z = 0
		matTmp.resize(4,3);
		matTmp.col(0) = matWP.row(i).transpose();
		matTmp.col(1) = matWP.row(i+1).transpose();
		matTmp.col(2) = matWP.row(i+2).transpose();    
		vecPnew[i] = vecTM[i].inverse()*matTmp;    
	}// end for (i=0; i<n-2; i++)
	
	
	
	/// Distance between two waypoints in 2D plane : L
	MatrixXd matDp1, matDp2;
	VectorXd vecL;
	
	matDp1.resize(1,vecPnew[0].rows());
	matDp1 = (vecPnew[0].col(1) - vecPnew[0].col(0)).transpose();
	
	vecL.resize(n-1);
	vecL(0) = matDp1.row(0).norm();
	
	matDp2.resize(n-2, matDp1.cols());
	
	for (i=0; i<n-2; i++)
	{
		matDp2.row(i) = (vecPnew[i].col(2) - vecPnew[i].col(1)).transpose();
		vecL(i+1) = matDp2.row(i).norm();
	}
	
	MatrixXd matdkappa, matDkappa;
	
	matdkappa.resize(n-2,2);	matdkappa.setZero();
	
	/// d_kappa calculation : d_kappa
	MatrixXd matXP, matYP;
	VectorXd vecGamma, vecBeta;
	
	matXP.resize(n-2,3);	matYP.resize(n-2,3);
	vecGamma.resize(n-2);	vecBeta.resize(n-2);
	
	for (i=0; i<n-2; i++)
	{
		// use only x,y
		matXP.row(i) = vecPnew[i].block(0,0,1,3);
		matYP.row(i) = vecPnew[i].block(1,0,1,3);
		
		// Find out gamma
		vecGamma(i) = atan2(matYP(i,2)-matYP(i,1), matXP(i,2)-matXP(i,1)); // angle between P3 and P2 : gamma
		
		
		
		// Change 1: Remove numerical error
		if (isnan(vecGamma(i)) )
			vecGamma(i) = 0;
		
		
		
		// G2 condition
		if (vecGamma(i) >= 0)
			vecBeta(i) = 0.5*vecGamma(i);
		else if (vecGamma(i) < 0)
			vecBeta(i) = M_PI*0.5*vecGamma(i);
		
		
		
		// Change 2 : Nonholonomic Constraint
		matdkappa(i,0) = 1.122592785417595*sin(vecBeta(i))/P.kappa/(cos(vecBeta(i))*cos(vecBeta(i)));
		
		if (matdkappa(i,0) > dMax)
			matdkappa(i,0) = dMax;
		
	}// end for (i=0; i<n-2; i++)
	
	
	
	/// Calculate the required Length : Lr
	VectorXd vecLr, vecLd, vecLmax;
	double dLmax;
	
	vecLr.resize(n-1);
	
	for (i=0; i<n-3; i++)
		vecLr(i+1) = matdkappa(i,0)+matdkappa(i+1,0);
	
	vecLr(0) = matdkappa(0,0);
	vecLr(n-2) = matdkappa(matdkappa.rows()-1,0);
	
	vecLd.resize(vecL.cols());
	vecLd = vecL - vecLr;
	
	
	// d_kappa
	if ( n==3 )
	{
		dLmax = vecL.minCoeff();
		vecLmax.resize(1);
		vecLmax << dLmax;
	}
	else if( n > 3 )
	{
		matDkappa.resizeLike(matdkappa);
		matDkappa = LmaxCalculation1(n-1, vecL, matdkappa, vecLr, vecLd);
		
		vecLmax.resize(matDkappa.rows());
		vecLmax = matDkappa.col(0);
	}
	
	
	
	/// Scale between 2D and 3D
	VectorXd vecSC;
	
	vecSC.resize(n-1);
	
	for (i=0; i<n-1; i++)
		vecSC(i) = vecL3(i)/vecL(i);
	
	
	
	/// Save data
	Data.L = vecL;
	Data.d_kappa = matdkappa;
	Data.Lmax = vecLmax;
	Data.TM = vecTM;
	Data.Pnew = vecPnew;
	Data.gamma = vecGamma;
	Data.beta = vecBeta;
	Data.SC = vecSC;
	
	return Data;
}



MatrixXd LmaxCalculation1(int n, VectorXd vecL, MatrixXd matDkappa, VectorXd vecLr, VectorXd vecLd)
{
	VectorXd vecTemp;
	int k, i;
	double dMAG;
	
	for (k=n-1; k>0; k--)
	{
		if (k==1)
		{
			vecTemp.resize(2);
			vecTemp << vecL(0), vecL(1)-matDkappa(1,0);
			dMAG = vecTemp.minCoeff();
			matDkappa(0,0) = dMAG;
		}
		else if ( k==n-1 )
		{
			vecTemp.resize(4);
			vecTemp << vecL(k), (matDkappa(k-1,0)+0.5*vecLd(k-1)), (matDkappa(k-2,0)+0.5*vecLd(k-1)),
		(matDkappa(k-2,0)+0.5*vecLd(k-2));
			
			dMAG = vecTemp.minCoeff();

			if ( dMAG == vecTemp(0) )
				matDkappa(k-1,0) = dMAG;
			else if ( dMAG == vecTemp(1) )
				matDkappa(k-1,0) = dMAG;
			else if ( dMAG == vecTemp(2) )
				matDkappa(k-1,0) = min(vecL(k), vecL(k-1)-dMAG);
			else if ( dMAG == vecTemp(3) )
				matDkappa(k-1,0) = min(vecL(k), vecL(k-1)-dMAG);
		}
		else
		{
			vecTemp.resize(4);	
			vecTemp << (vecL(k)-matDkappa(k,0)), (matDkappa(k-1,0)+0.5*vecLd(k-1)),
			(matDkappa(k-2,0)+0.5*vecLd(k-1)), (matDkappa(k-2,0)+0.5*vecLd(k-2));
	
			dMAG = vecTemp.minCoeff();
			
			if ( dMAG == vecTemp(0) )
				matDkappa(k-1,0) = dMAG;
			else if ( dMAG == vecTemp(1) )
				matDkappa(k-1,0) = dMAG;
			else if ( dMAG == vecTemp(2) )
				matDkappa(k-1,0) = min(vecL(k)-matDkappa(k,0), vecL(k-1)-dMAG);
			else if ( dMAG == vecTemp(3) )
				matDkappa(k-1,0) = min(vecL(k)-matDkappa(k,0), vecL(k-1)-dMAG);
		}// if (k==1)
		
		
		
		/// Calculate the required Length
		for (i=0; i<n-2; i++)
			vecLr(i+1) = matDkappa(i,0)+matDkappa(i+1,0);
		
		vecLr(0) = matDkappa(0,0);
		vecLr(n-1) = matDkappa(matDkappa.rows()-1,0);
		

		
		/// Difference between the real Length and the required Length
		for (i=0; i<n; i++)
			vecLd(i) = vecL(i) - vecLr(i);
		
	}// end for
	
	return matDkappa;
}
