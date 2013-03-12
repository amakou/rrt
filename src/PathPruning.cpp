// #include "PathPruning.h"
// #include "Kappa_Max_Calculation.h"
// #include "Collision.h"

/*
MatrixXd NHRRT_PATH_PRUNING_POSITION_MOD3(MatrixXd matPathFinal, setting_t P, world_t world, double dMaxBeta)
{
  int flag, n, j, i;
  
  flag = 0;	n = matPathFinal.rows();
  j = 1;	i = 0;
  
  MatrixXd matFinalPoint_e, matFinalPoint_E, matFinalPoint;
  VectorXd vecBeginPo, vecEndPo1, vecEndPo2, vecEndPoint;
  VectorXd vecP0, vecP1, vecP2, vecP3, vecP4;
  
  vecBeginPo.resize(matPathFinal.cols());
  vecEndPo1.resize(matPathFinal.cols());
  vecEndPo2.resize(matPathFinal.cols());
  vecEndPoint.resize(matPathFinal.cols());
  
  vecP0.resize(matPathFinal.cols());
  vecP1.resize(matPathFinal.cols());
  vecP2.resize(matPathFinal.cols());
  vecP3.resize(matPathFinal.cols());
  vecP4.resize(matPathFinal.cols());
  
  vecBeginPo = matPathFinal.row(1).transpose();
  vecEndPo1 = matPathFinal.row(2).transpose();
  vecEndPo2 = matPathFinal.row(3).transpose();
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
    
    if ( (Collision10( vecP0.segment(0,3), vecP3.segment(0,3), world, P.SZR, P.SZH )==0) &&
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
  
  matFinalPoint.resize( 1 + matFinalPoint_E.rows() + 3, matFinalPoint_E.cols() );
  matFinalPoint.topRows(1) = matPathFinal.row(0);
  matFinalPoint.block(1, 0, matFinalPoint_E.rows(), matFinalPoint_E.cols()) = matFinalPoint_E;
  matFinalPoint.bottomRows(3) = matPathFinal.bottomRows(3);
  
  return matFinalPoint;
}
*/