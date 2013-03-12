#include "PathPlanner.h"

/*void CPathPlanner::SetWorldBoundary(CURB_Point_3D sw, CURB_Point_3D ne)
{
	m_knownWorld = CreateWorld_Line(0,Vector3d(ne.x,ne.y,ne.z), Vector3d(sw.x,sw.y,sw.z));
	m_knownWorld.lineSeg.push_back(CURB_Segment(CURB_Point_2D(sw.x,sw.y),CURB_Point_2D(sw.x,ne.y)));
	m_knownWorld.lineSeg.push_back(CURB_Segment(CURB_Point_2D(sw.x,ne.y),CURB_Point_2D(ne.x,ne.y)));
	m_knownWorld.lineSeg.push_back(CURB_Segment(CURB_Point_2D(ne.x,ne.y),CURB_Point_2D(ne.x,sw.y)));
	m_knownWorld.lineSeg.push_back(CURB_Segment(CURB_Point_2D(ne.x,sw.y),CURB_Point_2D(sw.x,sw.y)));
	m_knownWorld.iNumObstacles = m_knownWorld.lineSeg.size();
}*/


void CPathPlanner::SetRobotState(CURB_Vector_3D robotState, double robotAngle)
{
	double *dRobotPos = new double[3];
	
	m_robotPosition = robotState;
	m_robotAngle = robotAngle;
	
	// 2D case
	dRobotPos[0] = m_robotPosition.x;
	dRobotPos[1] = m_robotPosition.y;
	dRobotPos[2] = m_robotAngle;
	
	m_extractor.Set_Robot_Position(dRobotPos);
	
	delete dRobotPos;

	MatrixXd matTemp;
	matTemp.resize(1,2);
	matTemp << m_robotPosition.x, m_robotPosition.y;
	
	m_Trace.conservativeResize(m_Trace.rows()+1,2);
	m_Trace.bottomRows(1) = matTemp;
}


void CPathPlanner::UpdateExtractorDbMap()
{
	MatrixXd db_map_info(m_knownWorld.iNumObstacles,7); // array info. is stored into the matrix.
	
	for(int i=0; i<m_knownWorld.iNumObstacles; i++)
	{
		db_map_info(i,0) = m_knownWorld.lineSeg[i].a;
		db_map_info(i,1) = m_knownWorld.lineSeg[i].b;
		db_map_info(i,2) = m_knownWorld.lineSeg[i].c;
		db_map_info(i,3) = m_knownWorld.lineSeg[i].startPnt.x;
		db_map_info(i,4) = m_knownWorld.lineSeg[i].startPnt.y;
		db_map_info(i,5) = m_knownWorld.lineSeg[i].endPnt.x;
		db_map_info(i,6) = m_knownWorld.lineSeg[i].endPnt.y;
	}
	
	m_extractor.Set_Db_Map(db_map_info);
}


void CPathPlanner::UpdateLRF()
{
	CURB_Vector_3D robotState;
	
	// 2D case
	robotState.x = m_robotPosition.x;
	robotState.y = m_robotPosition.y;
	robotState.z = m_robotAngle;
	
	m_vLRF.Update_Sensor_Range(robotState, m_world);
	m_LRF.SetRange(m_vLRF.GetRange());
}


void CPathPlanner::UpdateKnownWorld()
{
	int num_points = (int)m_LRF.GetRange().size();
	double *range = new double[num_points];
	
	for(int i=0; i<num_points; i++) range[i] = m_LRF.GetRange()[i];
	
	// 2D case
	double *dRobotPos = new double[3];
	dRobotPos[0] = m_robotPosition.x;
	dRobotPos[1] = m_robotPosition.y;
	dRobotPos[2] = m_robotAngle;

	m_extractor.Set_Robot_Position(dRobotPos);
	delete dRobotPos;
	
	m_extractor.Set_Range(range);
	delete range;
	
	m_extractor.Extract_Line();
	m_extractor.DB_matching();
	
	MatrixXd unknown_line = m_extractor.Get_Unknown_Line();
	MatrixXd known_line = m_extractor.Get_Known_Line();
	
	int numNewLine = unknown_line.cols();
	
	if(numNewLine==0)	return;
	else std::cout<<"numNewLine : "<<numNewLine<<std::endl;
	
	// add Unknown obstacle information
	vector<CURB_Segment> newObstacle;
	newObstacle.clear();
	
	for(int i=0;i<numNewLine;i++)
	{
		if ( (abs(unknown_line(9,numNewLine -i -1))<0.01) || (abs(unknown_line(10,numNewLine -i -1))<0.01) )
		{
//			std::cout<<"wrong line extracted"<<std::endl;
			unknown_line(9,numNewLine-i-1) = -1;
			unknown_line(10,numNewLine-i-1) = -1;
			unknown_line(8,numNewLine-i-1) = -1;
			unknown_line(7,numNewLine-i-1) = -1;
		}

		m_knownWorld.lineSeg.push_back(CURB_Segment(CURB_Point_2D((float)unknown_line(9,numNewLine -i -1),(float)unknown_line(10,numNewLine -i -1)),CURB_Point_2D((float)unknown_line(7,numNewLine -i -1),(float)unknown_line(8,numNewLine -i -1))));
		m_knownWorld.iNumObstacles = m_knownWorld.lineSeg.size();
	}
	
	FILE *fOut = NULL;
	
	SaveKnownWorld(fOut,m_knownWorld,m_pathGenNum);
}


void CPathPlanner::SetStartNode(CURB_Vector_3D startNode)
{
	m_startNode << startNode.x, startNode.y, startNode.z, 0, 0, 0, 0, 0;
	
	m_robotPosition.x = startNode.x;
	m_robotPosition.y = startNode.y;
	m_robotPosition.z = startNode.z;
}


void CPathPlanner::GenPath()
{
	VectorXd vecT;
	
	vecT = linspace(0,1,10);	// Bezier curve plotting
	
	///  RRT Path Generation
	MatrixXd matTree;
	
	matTree.resize(2,8);
	matTree.row(0) = m_startNode.transpose();
	
	
	/// Concentration Sampling
	Vector3d vec3Diff(m_startNode(0) - m_endNode(0), m_startNode(1) - m_endNode(1), m_startNode(2) - m_endNode(2));  
	double psi;
	sigma_t sigma;
	
	psi = atan2(m_endNode(1)-m_startNode(1),m_endNode(0)-m_startNode(0));
	psi = psi - 2*M_PI*(psi>M_PI);
	
	sigma.r = vec3Diff.norm();
	sigma.theta0 = deg2rad(m_robotAngle);
	sigma.theta = 0.10* M_PI;
	sigma.sp = m_startNode.segment(0,3);
	
	// maximum curvature of path
	int iMaxIte, iNumPath, iINDC, iIteNum, iFlag, count;;
	double dKmax, dBeta, dMaxBeta;
	double c1, c2, c3, c4, dMax;
	
	MatrixXd Path;
	extendTreeR_t extendTree;
	
	iMaxIte = 3;		// k - nearest neighbor
	dKmax = m_P.kappa;
	dBeta = 0.10*M_PI;
	dMaxBeta = dBeta;
  	
	c1 = 7.2364;
	c2 = 0.4*(sqrt(6.0)-1.0);
	c3 = (c2+4.0)/(c1+6.0);
	c4 = (c2+4.0)*(c2+4.0) / (54.0*c3);
	dMax = ( c4*sin(dBeta) )/( dKmax*cos(dBeta)*cos(dBeta) );
	
	count = 0;

	// Modification 1
	// Initial Heading Consisderation
	double s_angle, psx, psy;

	s_angle = m_robotAngle*M_PI/180.0;
	psx = m_startNode(0) + 1.5*dMax*cos(s_angle);
	psy = m_startNode(1) + 1.5*dMax*sin(s_angle);
	matTree.row(1) << psx, psy, m_startNode(2), 0, 0, 0, 0, 1;

	
	// check to see if start_node connects directly to end node
	if ( (sigma.r < (double)m_P.L) && (Collision(m_startNode, m_endNode, m_knownWorld, m_P.SZR, m_P.SZH)==0) )
	{
		Path.resize(2,8);
		Path.row(0) = m_startNode.transpose();
		Path.row(1) = m_endNode.transpose();
		Path(1,3) = 1;
		matTree = Path;
	}
	else
	{
		iNumPath = 0;
		iINDC = 0;
		iIteNum = 0;
		
		srand( (unsigned)time(NULL)); // initialize rand
		
		while (iNumPath<1)
		{
			//extendTree = ExtendTree_NHC_Sort_NoGS_NoTermCond(matTree, vecEndNode, world, P, dKmax, c4,
			//   						       dMax, dMaxBeta, sigma, iMaxIte, iINDC);
			//extendTree = ExtendTree_NHC_Sort_OnlyGS_TermCond(matTree, m_endNode, m_knownWorld, m_P, dKmax, c4,
			//						       dMax, dMaxBeta, sigma, iMaxIte, iINDC);
			extendTree = ExtendTree_NHC_Sort_OnlyGS_TermCond_Heading(matTree, m_endNode, m_knownWorld, m_P, dKmax, c4,
										       dMax, dMaxBeta, sigma, iMaxIte, iINDC);
			
			matTree	= extendTree.newTree;
			iFlag		= extendTree.flag;
			iINDC		= extendTree.INDC;
			sigma		= extendTree.sigma;
			iMaxIte		= extendTree.maxIte;
			iNumPath	= iNumPath + iFlag;
			
			count++;
		}
	}
	
	/// Find path with minimum cost to end node
	MatrixXd matPathFinal;
	
	matPathFinal = FindMinimumPath(matTree, m_endNode);

	
	/// Path Pruning
	MatrixXd matPathPruned;
	if (matPathFinal.rows() >= 5)
		matPathPruned = NHRRT_PATH_PRUNING_POSITION_MOD3_Heading(matPathFinal, m_P, m_knownWorld, dMaxBeta);
	else
	{
		matPathPruned = matPathFinal;
		std::cout<<"Skip path pruning process"<<std::endl;
	}

	if (matPathPruned.rows() == 2)
	{
		int len = vecT.rows();
		double m, n, angle;
		
		std::cout<<"Final Point consists of two nodes"<<std::endl;
		
		VectorXd vecPx(len), vecPy(len), vecPz(len), vecPsi(len);
		angle = rad2deg(atan2(matPathPruned.col(1)(1)-matPathPruned.col(1)(0),matPathPruned.col(0)(1)-matPathPruned.col(0)(0)));
		
		for (int i=0; i<len; i++)
		{
			m = vecT(i);
			n = vecT(len-1-i);
			
			vecPx(i) = n*matPathPruned.col(0)(0) + m*matPathPruned.col(0)(1);
			vecPy(i) = n*matPathPruned.col(1)(0) + m*matPathPruned.col(1)(1);
			vecPz(i) = n*matPathPruned.col(2)(0) + m*matPathPruned.col(2)(1);
			vecPsi(i) = angle;
		}
		
		m_Path = matPathPruned;
		m_Tree = matTree;
		m_CurveX = vecPx;
		m_CurveY = vecPy;
		m_CurveZ = vecPz;
		m_CurvePsi = vecPsi;
		m_curStep = 0;
		m_pathGenNum++;
		
		return;
	}	
	
	/// Simple Cubic Bezier Curve
	spiralCurveRet_t spiralCurveRet;
	MatrixXd matPathSegXss, matPathSegYss, matPathSegZss;
	VectorXd vecFsxs, vecFsys, vecFszs, vecFsPsis, vecFsKappa1s, vecFsKappa2s, vecFsKappa3s, vecFsKappa4s, vecFsLs;
	VectorXd vecMaxL;
	int Fsns;

	vecMaxL = MaxLength_C1CBC(matPathPruned.block(0, 0, matPathPruned.rows(), 3));
	
	spiralCurveRet = Full_Simple_Curve_Path(m_knownWorld, matPathPruned, vecT, m_P, vecMaxL);
	
	matPathSegXss	= spiralCurveRet.PathSegX;	matPathSegYss	= spiralCurveRet.PathSegY;
	matPathSegZss = spiralCurveRet.PathSegZ;	vecFsxs		= spiralCurveRet.P_x;
	vecFsys 	= spiralCurveRet.P_y;		vecFszs		= spiralCurveRet.P_z;
	vecFsPsis 	= spiralCurveRet.Psi;		vecFsKappa1s 	= spiralCurveRet.Kappa1;
	vecFsKappa2s 	= spiralCurveRet.Kappa2;	vecFsKappa3s 	= spiralCurveRet.Kappa3;
	vecFsKappa4s 	= spiralCurveRet.Kappa4;	vecFsLs		= spiralCurveRet.L;
	Fsns 		= spiralCurveRet.nn;
 	

 	/// Without Path Pruning case
	MatrixXd matPathSegXs, matPathSegYs, matPathSegZs;
	VectorXd vecFsx, vecFsy, vecFsz, vecFsPsi, vecFsKappa1, vecFsKappa2, vecFsKappa3, vecFsKappa4, vecFsL;
	int Fsn;
	info_t INFO1;
	
	// Path Smoothing without path pruning
	INFO1 = MaxLengthCalculationNormal(matPathFinal, m_P, dMax);
	
	spiralCurveRet = Full_Spiral_Curve_Path_L(m_knownWorld, matPathFinal, vecT, m_P, INFO1);
	
	matPathSegXs 	= spiralCurveRet.PathSegX;	matPathSegYs 	= spiralCurveRet.PathSegY;
	matPathSegZs 	= spiralCurveRet.PathSegZ;	vecFsx	 	= spiralCurveRet.P_x;
	vecFsy 	= spiralCurveRet.P_y;		vecFsz	 	= spiralCurveRet.P_z;
	vecFsPsi 	= spiralCurveRet.Psi;		vecFsKappa1 	= spiralCurveRet.Kappa1;
	vecFsKappa2 	= spiralCurveRet.Kappa2;	vecFsKappa3 	= spiralCurveRet.Kappa3;
	vecFsKappa4 	= spiralCurveRet.Kappa4;	vecFsL	 	= spiralCurveRet.L;
	Fsn 		= spiralCurveRet.nn;
	
	
	/// Path Pruning case
	MatrixXd matPathSegX, matPathSegY, matPathSegZ;
	VectorXd vecFx, vecFy, vecFz, vecFPsi, vecFKappa1, vecFKappa2, vecFKappa3, vecFKappa4, vecFL;
	int Fn;
	info_t INFO; 
	
	// Path Smoothing with path pruning
	INFO = MaxLengthCalculationNormal(matPathPruned, m_P, dMax);
	
	spiralCurveRet = Full_Spiral_Curve_Path_L(m_knownWorld, matPathPruned, vecT, m_P, INFO);
	
	matPathSegX	= spiralCurveRet.PathSegX;	matPathSegY 	= spiralCurveRet.PathSegY;
	matPathSegZ 	= spiralCurveRet.PathSegZ;	vecFx	 	= spiralCurveRet.P_x;
	vecFy 	= spiralCurveRet.P_y;		vecFz		= spiralCurveRet.P_z;
	vecFPsi 	= spiralCurveRet.Psi;		vecFKappa1 	= spiralCurveRet.Kappa1;
	vecFKappa2 	= spiralCurveRet.Kappa2;	vecFKappa3 	= spiralCurveRet.Kappa3;
	vecFKappa4 	= spiralCurveRet.Kappa4;	vecFL 		= spiralCurveRet.L;
	Fn 		= spiralCurveRet.nn;
	
	
	///  Length of the Curve
	int nnss, nns, nn, k;
	double curveLen;
	
	nnss = matPathSegXss.rows();
	nns = matPathSegXs.rows();
	nn = matPathSegX.rows();
	
	// Cubic Simple Bezier
	VectorXd vecLss1(0), vecLss2(0), vecLss3(0);
	
	for (k=0; k<nnss; k++)
	{
		curveLen = CurveLength(matPathSegXss.row(k), matPathSegYss.row(k));
		vecLss1.conservativeResize(vecLss1.rows()+1);
		vecLss1.tail(1) << curveLen;

		curveLen = CurveLength(matPathSegXss.row(k), matPathSegZss.row(k));
		vecLss2.conservativeResize(vecLss2.rows()+1);
		vecLss2.tail(1) << curveLen;
		
		curveLen = CurveLength(matPathSegZss.row(k), matPathSegXss.row(k));
		vecLss3.conservativeResize(vecLss3.rows()+1);
		vecLss3.tail(1) << curveLen;
	} 
	
	// Without Path Pruning
	VectorXd vecLs1(0), vecLs2(0), vecLs3(0);
	
	// x-y plane, x-z plane, z-x plane
	for (k=0; k<nns; k++)
	{
		curveLen = CurveLength(matPathSegXs.row(k), matPathSegYs.row(k));
		vecLs1.conservativeResize(vecLs1.rows()+1);
		vecLs1.tail(1) << curveLen;
		
		curveLen = CurveLength(matPathSegXs.row(k), matPathSegZs.row(k));
		vecLs2.conservativeResize(vecLs2.rows()+1);
		vecLs2.tail(1) << curveLen;
		
		curveLen = CurveLength(matPathSegZs.row(k), matPathSegXs.row(k));
		vecLs3.conservativeResize(vecLs3.rows()+1);
		vecLs3.tail(1) << curveLen;
	}
	
	// Path Pruning
	VectorXd vecL1(0), vecL2(0), vecL3(0);
	
	// x-y plane, x-z plane, z-x plane
	for (k=0; k<nn; k++)
	{
		curveLen = CurveLength(matPathSegX.row(k), matPathSegY.row(k));
		vecL1.conservativeResize(vecL1.rows()+1);
		vecL1.tail(1) << curveLen;
		
		curveLen = CurveLength(matPathSegX.row(k), matPathSegZ.row(k));
		vecL2.conservativeResize(vecL2.rows()+1);
		vecL2.tail(1) << curveLen;
		
		curveLen = CurveLength(matPathSegZ.row(k), matPathSegX.row(k));
		vecL3.conservativeResize(vecL3.rows()+1);
		vecL3.tail(1) << curveLen;
	}
	
	
//    /// *************** ///
//    ///  Data for plot  ///
//    /// *************** ///
//    FILE *fOut = NULL;
//    
//    // Save data for MATLAB plot
//    SaveResult(fOut, matTree, matPathFinal, matPathPruned, vecT,
//  	      vecFsx,		vecFsy,		vecFsz,
//   	      vecFsKappa1,	vecFsKappa2,	vecFsKappa3,	vecFsKappa4,
// 	      vecLs1, 		vecLs2,		vecLs3,
// 	      vecFx, 		vecFy, 		vecFz,
// 	      vecFKappa1, 	vecFKappa2, 	vecFKappa3, 	vecFKappa4,
// 	      vecL1, 		vecL2, 		vecL3,
// 	      vecFsPsi, 	vecFPsi);
//   
//    SaveResult(fOut,
// 	      vecFsxs,		vecFsys,	vecFszs,	vecFsPsis,
// 	      vecFsKappa1s, 	vecFsKappa2s, 	vecFsKappa3s, 	vecFsKappa4s,
// 	      vecFsLs, 		vecLss1, 	vecLss2, 	vecLss3);
//    
//    SaveParameter(fOut, m_P.d, m_P.SZR, m_P.SZH, nns, nn);
//    
//    SaveWorld(fOut,m_world);
//    SaveKnownWorld(fOut,m_knownWorld);
	
	m_Path = matPathPruned;
	m_Tree = matTree;
	m_CurveX = vecFx; m_CurveY = vecFy; m_CurveZ = vecFz; m_CurvePsi = vecFPsi;
	m_CurveKappa = vecFKappa4;
	m_L = vecL1;
	m_PathSegX = matPathSegX; m_PathSegY = matPathSegY; m_PathSegZ = matPathSegZ;
	m_curStep = 0;
	m_pathGenNum++;

	m_psi = vecFPsi(0)*180/M_PI;
	
	return;
}


bool CPathPlanner::IsArrived()
{
	double err;

	err = sqrt((m_endNode(0)-m_robotPosition.x)*(m_endNode(0)-m_robotPosition.x)+(m_endNode(1)-m_robotPosition.y)*(m_endNode(1)-m_robotPosition.y)); 

	std::cout<<"Robot Position :  "<<m_robotPosition.x<<" "<<m_robotPosition.y<<" "<<m_robotPosition.z<<" "<<m_robotAngle <<std::endl;
	std::cout<<"Distance to goal : "<<err<<std::endl;

	if ( err  < 0.1 )
		return true;
	else
		return false;
}


bool CPathPlanner::CollisionCheck()
{
	VectorXd vecCur(3), vecNext(3);
  
	for (int i=0; i<m_CurveX.rows()-2;i++)
	{
		vecCur << m_CurveX(i), m_CurveY(i), m_CurveZ(i);
		vecNext << m_CurveX(i+1), m_CurveY(i+1), m_CurveZ(i+1);
  		
		if(Collision10(vecCur, vecNext, m_knownWorld, m_P.SZR, m_P.SZH))
			return true;
	}
	
	return false;
}


void CPathPlanner::StackPath()
{
	int preRow;
	
	preRow = m_PathStack.rows();
	
	m_PathStack.conservativeResize(m_PathStack.rows()+m_curStep+1,m_Path.cols());
	
	m_PathStack.block(preRow, 0, m_curStep+1, m_Path.cols()) = m_Path.block(0,0,m_curStep+1,m_Path.cols());
}


void CPathPlanner::StackCurve()
{
	int preSize;
	
	preSize = m_CurveX_stack.rows();
	
	m_CurveX_stack.conservativeResize(preSize+m_curStep);
	m_CurveY_stack.conservativeResize(preSize+m_curStep);
	m_CurveZ_stack.conservativeResize(preSize+m_curStep);
	m_CurvePsi_stack.conservativeResize(preSize+m_curStep);
  
	for (int i=0; i<m_curStep; i++)
	{
		m_CurveX_stack(preSize+i) = m_CurveX(i);
		m_CurveY_stack(preSize+i) = m_CurveY(i);
		m_CurveZ_stack(preSize+i) = m_CurveZ(i);
		m_CurvePsi_stack(preSize+i) = m_CurvePsi(i);
	}
}


void CPathPlanner::MoveRobot()
{
	VectorXd vecCur(3), vecNew(3);
	
	vecCur << m_robotPosition.x, m_robotPosition.y, m_robotPosition.z;
	vecNew << m_CurveX(m_curStep+1), m_CurveY(m_curStep+1), m_CurveZ(m_curStep+1);
	
	if (Collision10(vecCur, vecNew, m_knownWorld, m_P.SZR, m_P.SZH) == 0)
	{
		m_curStep++;
		
		m_robotPosition = CURB_Vector_3D(vecNew(0), vecNew(1), vecNew(2));
		m_robotAngle = rad2deg(AS_PI(m_CurvePsi(m_curStep)));
		
		//std::cout<<"Move robot : "<<m_robotPosition.x<<" "<<m_robotPosition.y<<" "<<m_robotPosition.z<<" "<<m_robotAngle <<std::endl;
	}
	else
		std::cout<<"Move robot : collision detected"<<std::endl;
}


void CPathPlanner::SaveData()
{
	FILE *fOut = NULL;
	
	// [F_x; F_y; F_z]
	char buffer [50];
	
	sprintf(buffer,"data/Tree%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveMatData(fOut, m_Tree);
	fprintf(fOut,"\n");
	fclose(fOut);

	sprintf(buffer,"data/Curve%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveVecData(fOut, m_CurveX);  fprintf(fOut,"\n");
	SaveVecData(fOut, m_CurveY);  fprintf(fOut,"\n");
	SaveVecData(fOut, m_CurveZ);  fprintf(fOut,"\n");
	fclose(fOut);
	
	sprintf(buffer,"data/CurveStack%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveVecData(fOut, m_CurveX_stack);  fprintf(fOut,"\n");
	SaveVecData(fOut, m_CurveY_stack);  fprintf(fOut,"\n");
	SaveVecData(fOut, m_CurveZ_stack);  fprintf(fOut,"\n");
	fclose(fOut);
	
	// F_Psi
	sprintf(buffer,"data/Psi%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveVecData(fOut, m_CurvePsi);
	fclose(fOut);
	
	// F_Psi
	sprintf(buffer,"data/PsiStack%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveVecData(fOut, m_CurvePsi_stack);
	fclose(fOut);
	
	sprintf(buffer,"data/Path%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveMatData(fOut, m_Path);
	fclose(fOut);
	
	fOut = fopen("data/Parameters.txt","wt");
	fprintf(fOut,"%f\t%f\t%f\t%d\t", m_P.d, m_P.SZR, m_P.SZH, m_pathGenNum);
	fclose(fOut);
	
	sprintf(buffer,"data/Node%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveVecData(fOut, m_startNode); fprintf(fOut,"\n");
	SaveVecData(fOut, m_endNode);   fprintf(fOut,"\n");
	fclose(fOut);
	
	sprintf(buffer,"data/Kappa%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveVecData(fOut, m_CurveKappa);   fprintf(fOut,"\n");
	fclose(fOut);
	
	sprintf(buffer,"data/L%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveVecData(fOut, m_L);   fprintf(fOut,"\n");
	fclose(fOut);
	
	sprintf(buffer,"data/PathSegX%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveMatData(fOut, m_PathSegX);   fprintf(fOut,"\n");
	fclose(fOut);
	
	sprintf(buffer,"data/PathSegY%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveMatData(fOut, m_PathSegY);   fprintf(fOut,"\n");
	fclose(fOut);
	
	sprintf(buffer,"data/PathSegZ%d.txt",m_pathGenNum);
	fOut = fopen(buffer,"wt");
	SaveMatData(fOut, m_PathSegZ);   fprintf(fOut,"\n");
	fclose(fOut);

	sprintf(buffer,"data/Trace.txt");
	fOut = fopen(buffer,"wt");
	SaveMatData(fOut, m_Trace);   fprintf(fOut,"\n");
	fclose(fOut);
	
	SaveWorld(fOut,m_world,m_pathGenNum);
	SaveKnownWorld(fOut,m_knownWorld, m_pathGenNum);
}
