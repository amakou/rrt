#ifndef PATHPRUNING_H
#define PATHPRUNING_H

#include "Struct.h"
#include "Kappa_Max_Calculation.h"
#include "Collision.h"

template <typename T>
MatrixXd NHRRT_PATH_PRUNING_POSITION_MOD3_Heading(MatrixXd matPathFinal, setting_t P, T world, double dMaxBeta)
{
	int flag, n, j, i;
	
	flag = 0;	n = matPathFinal.rows();
	j = 1;	i = 0;
	
	int pathCol = matPathFinal.cols();
	
	MatrixXd matFinalPoint_e, matFinalPoint_E, matFinalPoint;
	VectorXd vecBeginPo(pathCol), vecEndPo1(pathCol), vecEndPo2(pathCol), vecEndPoint(pathCol);
	VectorXd vecP0(pathCol), vecP1(pathCol), vecP2(pathCol), vecP3(pathCol), vecP4(pathCol);

	// Modification 6 : begin_po = path_final(3,:); end_po1   = path_final(4,:); end_po2   = path_final(5,:);
	std::cout<<vecBeginPo.cols()<<" "<<matPathFinal.rows()<<" "<<matPathFinal.cols()<<std::endl;
	vecBeginPo = matPathFinal.row(2).transpose();
	vecEndPo1 = matPathFinal.row(3).transpose();
	vecEndPo2 = matPathFinal.row(4).transpose();

	vecEndPoint = matPathFinal.bottomRows(1).transpose();
	
	matFinalPoint_e.resize(0,8);

	while ( (flag==0) && ( (vecEndPo2.segment(0,3)-vecEndPoint.segment(0,3)).norm() > 1e-6 ) )
	{
		if ( matFinalPoint_e.rows()==0 )
		{
			vecP1 = matPathFinal.row(0);
			vecP2 = vecBeginPo;		vecP3 = vecEndPo1;
			vecP4 = vecEndPo2;		vecP0 = matPathFinal.row(0);
		}
		else if( matFinalPoint_e.rows()==1 )
		{
			vecP1 = matPathFinal.row(0);
			vecP2 = matFinalPoint_e.row(0);	vecP3 = vecEndPo1;
			vecP4 = vecEndPo2;		vecP0 = matPathFinal.row(0);
		}
		else
		{
			vecP1 = matFinalPoint_e.row(matFinalPoint_e.rows()-1);
			vecP2 = vecBeginPo;		vecP3 = vecEndPo1;
			vecP4 = vecEndPo2;		vecP0 = matFinalPoint_e.bottomRows(1).transpose();
		}
		
		MatrixXd matWP1, matWP2, matDkappa1, matDkappa2;
		VectorXd vecBeta1, vecBeta2;
		
		matWP1.resize(3,3);
		matWP1.row(0) = vecP1.segment(0,3);
		matWP1.row(1) = vecP2.segment(0,3);
		matWP1.row(2) = vecP3.segment(0,3);
		
		matWP2.resize(3,3);
		matWP2.row(0) = vecP2.segment(0,3);
		matWP2.row(1) = vecP3.segment(0,3);
		matWP2.row(2) = vecP4.segment(0,3);
		
		Kappa_Max_Calculation(matWP1, P, matDkappa1, vecBeta1);
		Kappa_Max_Calculation(matWP2, P, matDkappa2, vecBeta2);
		
		if ( (Collision( vecP0.segment(0,3), vecP3.segment(0,3), world, P.SZR, P.SZH, 10)==0) &&
			(vecBeta1.array().abs()<= dMaxBeta).all() &&
			(vecBeta2.array().abs()<= dMaxBeta).all() )
		{
			j = j + 1;
			
			vecBeginPo = matPathFinal.row(j);
			vecEndPo1 = matPathFinal.row(j+1);
			vecEndPo2 = matPathFinal.row(j+2);
			
			if ( j > n-3 )
				flag = 1;
		}
		else
		{
			j = j + 1;
			
			vecBeginPo = matPathFinal.row(j);
			vecEndPo1 = matPathFinal.row(j+1);
			vecEndPo2 = matPathFinal.row(j+2);
			
			matFinalPoint_e.conservativeResize(matFinalPoint_e.rows()+1, matFinalPoint_e.cols() );
			
			matFinalPoint_e.row(i) = matPathFinal.row(j-1);
			
			i = i + 1;
		}
	} // end of while
	
	if ( matFinalPoint_e.rows()==0 )
		matFinalPoint_E.resize(0,0);
	else if ( (matFinalPoint_e.rows()>1) &&
		( (matFinalPoint_e.row(0).segment(0,3) - matFinalPoint_e.row(1).segment(0,3)).norm()<1e-6 ) )
	{
		matFinalPoint_E.resize(matFinalPoint_e.rows()-1, matFinalPoint_e.cols());
		matFinalPoint_E = matFinalPoint_e.block(1, 0, matFinalPoint_e.rows()-1, matFinalPoint_e.cols());
	}
	else
	{
		matFinalPoint_E.resize(matFinalPoint_e.rows(), matFinalPoint_e.cols());
		matFinalPoint_E = matFinalPoint_e;
	}

	// Modification 7  
	matFinalPoint.resize( 2 + matFinalPoint_E.rows() + 3, matPathFinal.cols() );
	matFinalPoint.topRows(2) = matPathFinal.topRows(2);
	matFinalPoint.block(2, 0, matFinalPoint_E.rows(), matFinalPoint_E.cols()) = matFinalPoint_E;
	matFinalPoint.bottomRows(3) = matPathFinal.bottomRows(3);  

	if ( (matFinalPoint.row(1).segment(0,3) - matFinalPoint.row(2).segment(0,3)).norm()<1e-6 )
	{
		MatrixXd matTemp;

		matTemp.resize(matFinalPoint.rows()-1, matFinalPoint.cols());
		matTemp.topRows(1) = matFinalPoint.row(0);
		matTemp.bottomRows(matTemp.rows()-1) = matFinalPoint.bottomRows(matTemp.rows()-1);

		return matTemp;
	}
	
	return matFinalPoint;
}
#endif
