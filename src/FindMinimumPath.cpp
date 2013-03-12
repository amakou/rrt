#include "FindMinimumPath.h"

/// find the lowest ost path o the end node
MatrixXd FindMinimumPath(MatrixXd matTree, VectorXd vecEndNode)
{
	/// find nodes that connect to end_node
	int treeLen, i, minVecIdx, iParentNode;
	double minVecVal;
	
	MatrixXd matConnectingNodes, matPath;
	VectorXd vecTemp;
	
	treeLen = matTree.rows();
	
	matConnectingNodes.resize(0, matTree.cols());

	for (i = 0; i< treeLen; i++)
	{
		if ( matTree(i,3)==1 )
		{
			matConnectingNodes.conservativeResize( matConnectingNodes.rows()+1, matTree.cols() );
			matConnectingNodes.bottomRows(1) = matTree.row(i);
		}
	}
		
	/// find minimum cost last node
	vecTemp.resize(matConnectingNodes.rows());
	vecTemp = matConnectingNodes.col(4);

	minVecVal = vecTemp.minCoeff(&minVecIdx);
	
	
	/// construct lowest cost path
	matPath.resize(2, matConnectingNodes.cols());
	matPath.topRows(1) = matConnectingNodes.row(minVecIdx);
	matPath.bottomRows(1) = vecEndNode.transpose();
	
	iParentNode = matConnectingNodes(minVecIdx, matConnectingNodes.cols()-1);
	
	MatrixXd matTmp;
	
	matTmp.resizeLike(matPath);
	matTmp = matPath;
	
	matPath.resize(matPath.rows()+1,matPath.cols());
	matPath.topRows(1) = matTree.row(iParentNode-1);
	matPath.bottomRows(matTmp.rows()) = matTmp;
	
	while (iParentNode>1)
	{
		iParentNode = matTree(iParentNode-1, matTree.cols()-1);
		
		
		matTmp.resizeLike(matPath);
		matTmp = matPath;
		
		matPath.resize(matPath.rows()+1,matPath.cols());
		matPath.topRows(1) = matTree.row(iParentNode-1);
		matPath.bottomRows(matTmp.rows()) = matTmp;
	} // while (iParentNode>1)

	return matPath;
}

