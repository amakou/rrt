#include "FindMinimumPath.h"

// find the lowest ost path o the end node
MatrixXd FindMinimumPath(MatrixXd matTree, VectorXd vecEndNode)
{
	int minVecIdx, iParentNodeIdx, iParentNode;
	double minVecVal;
	MatrixXd matConnectingNodes, matPath;

	// find nodes that connect to end_node
	for (int i=0; i < matTree.rows(); i++)	{
		if ( matTree(i, 3)==1 )	{
			matConnectingNodes.conservativeResize( matConnectingNodes.rows()+1, matTree.cols() );
			matConnectingNodes.bottomRows(1) = matTree.row(i);
		}
	}
		
	// find minimum cost last node
	VectorXd vecTemp(matConnectingNodes.rows());
	vecTemp = matConnectingNodes.col(4);	// cost

	minVecVal = vecTemp.minCoeff(&minVecIdx);	// index -> minVecIdx, value -> minVecVal
	
	/// construct lowest cost path
	matPath.resize(2, matConnectingNodes.cols());
	matPath.topRows(1) = matConnectingNodes.row(minVecIdx);
	matPath.bottomRows(1) = vecEndNode.transpose();
	
	iParentNodeIdx = matConnectingNodes(minVecIdx, matConnectingNodes.cols()-1);
	
	MatrixXd matTmp = matPath;
	
	matPath.resize(matPath.rows()+1,matPath.cols());
	matPath.topRows(1) = matTree.row(iParentNode-1);
	matPath.bottomRows(matTmp.rows()) = matTmp;
	
	while (iParentNodeIdx>1) {
		iParentNodeIdx = matTree(iParentNodeIdx-1, matTree.cols()-1);
		matTmp = matPath;
		
		matPath.resize(matPath.rows()+1, matPath.cols());
		matPath.topRows(1) = matTree.row(iParentNodeIdx-1);
		matPath.bottomRows(matTmp.rows()) = matTmp;
	}

	return matPath; // ... parent of last node, last node, end node
}

