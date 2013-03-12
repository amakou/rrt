#include "Kappa_Max_Calculation.h"

void Kappa_Max_Calculation(MatrixXd matPath, setting_t P, MatrixXd &d_kappa, VectorXd &beta)
{
	int n, i;
	MatrixXd matWP, matdp3, matXPi, matAZi, matZPi, matYZi, matYPi, matTmp;
	VectorXd vecL3;
	Vector3d vec3tmp, vec3tmp2;
	vector<MatrixXd> vecTM, vecPnew;
	
	// Iteration Number
	n = matPath.rows();
	
	// WP = [XP, YP, ZP, 1]
	matWP.resize(n, 4);
	
	for (i=0; i<n; i++)
	{
		matWP(i,0) = matPath(i,0);
		matWP(i,1) = matPath(i,1);
		matWP(i,2) = matPath(i,2);
		matWP(i,3) = 1;
	}
	
	
	/// Distance between two waypoints in 3D space
	matdp3.resize(n-1, 4);
	vecL3.resize(n-1);
	
	for (i=0; i<n-1; i++)
	{
		matdp3.row(i) = matWP.row(i+1) - matWP.row(i);
		vecL3(i) = matdp3.row(i).norm();
	}
	
	
	/// Transformation matrix calculation  
	matXPi.resize(n-2, 3);	matAZi.resize(n-2, 3);
	matZPi.resize(n-2, 3);	matYZi.resize(n-2, 3);
	matYPi.resize(n-2, 3);
	
	vecTM.resize(n-2);
	vecPnew.resize(n-2);
	
	for ( i=0; i<n-2; i++)
	{
		// XPi Calculation
		matXPi.row(i) = ( matdp3.block(i,0,1,3) )/vecL3(i);
		
		// ZPi Calculation
		vec3tmp.resize(3);	vec3tmp2.resize(3);
		vec3tmp = matdp3.block(i,0,1,3).transpose();
		vec3tmp2 = matdp3.block(i+1,0,1,3).transpose();
		
		matAZi.row(i) = vec3tmp.cross(vec3tmp2);
		matZPi.row(i) = matAZi.row(i)/matAZi.row(i).norm();
		
		// YPi Calculation
		vec3tmp = matZPi.block(i,0,1,3).transpose();
		vec3tmp2 = matXPi.block(i,0,1,3).transpose();
		
		matYZi.row(i) = vec3tmp.cross(vec3tmp2);
		matYPi.row(i) = matYZi.row(i)/matYZi.row(i).norm();
		
		// Transformation Matrix
		vecTM[i].resize(4,4);
		vecTM[i].block(0,0,3,1) = matXPi.row(i).transpose();	vecTM[i](3,0) = 0;
		vecTM[i].block(0,1,3,1) = matYPi.row(i).transpose();	vecTM[i](3,1) = 0;
		vecTM[i].block(0,2,3,1) = matZPi.row(i).transpose();	vecTM[i](3,2) = 0;
		vecTM[i].block(0,3,4,1) = matWP.row(i).transpose();
		
		if ( isnan(vecTM[i](0,0)) || isnan(vecTM[i](1,1)) || isnan(vecTM[i](2,2)) )
		{
			vecTM[i].col(0) << 1, 0, 0, 0;
			vecTM[i].col(1) << 0, 1, 0, 0;
			vecTM[i].col(2) << 0, 0, 1, 0;
			vecTM[i].col(3) << matWP.row(i).transpose();
		}
		
		// New Position : z = 0
		matTmp.resize(4,3);
		matTmp.col(0) = matWP.row(i).transpose();
		matTmp.col(1) = matWP.row(i+1).transpose();
		matTmp.col(2) = matWP.row(i+2).transpose();    
		vecPnew[i] = vecTM[i].inverse()*matTmp;
	} // end   for ( i=0; i<n-2; i++)
		
	
	/// d_kappa calculation : d_kappa
	MatrixXd matXP, matYP;
	VectorXd vecGamma, vec;
	
	matXP.resize(n-2, 3);
	matYP.resize(n-2, 3);  
	
	d_kappa.resize(n-2,2);
	//d_kappa.setZero();
	vecGamma.resize(n-2);
	beta.resize(n-2);
	
	for( i=0; i<n-2; i++)
	{
		// use only x, y
		matXP.row(i) = vecPnew[i].block(0,0,1,3);
		matYP.row(i) = vecPnew[i].block(1,0,1,3);
		
		// Find out gamma
		vecGamma(i) = atan2( matYP(i,2) - matYP(i,1), matXP(i,2) - matXP(i,1) );// angle between P3 and P2 : gamma
		
		if ( isnan( vecGamma(i) ) )
			vecGamma(i) = 0;
		
		beta(i) = 0.5*vecGamma(i);
		
		/// Nonholonomic Constraint    
		d_kappa(i,0) = 1.122592785417595*sin(beta(i))/P.kappa/(cos(beta(i))*cos(beta(i)));
	}
}


