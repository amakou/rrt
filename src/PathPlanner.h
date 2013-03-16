#pragma once

#include "../RRT.h"

class CPathPlanner
{
protected:
	setting_t m_P;
	VectorXd m_startNode, m_endNode;	// x, y, z, is last node, cost, ext length, abs, parent index
	MatrixXd m_Path, m_PathStack, m_Tree;
	VectorXd m_CurveX, m_CurveY, m_CurveZ, m_CurvePsi, m_CurveKappa, m_L;
	VectorXd m_CurveX_stack, m_CurveY_stack, m_CurveZ_stack, m_CurvePsi_stack, m_CurveKappa_stack;
	MatrixXd m_PathSegX, m_PathSegY, m_PathSegZ;
	MatrixXd m_PathSegX_stack, m_PathSegY_stack, m_PathSegZ_stack;
	MatrixXd m_Trace;
	
	CURB_Vector_3D m_robotPosition;
	double m_robotAngle, m_psi;// degree
	worldLine_t m_knownWorld, m_world;
	CVirtualLRF m_vLRF;
	CLRFModel m_LRF;
	
	Line_Param_Extract m_extractor;
	
	int m_curStep, m_pathGenNum;
	
public:
	CPathPlanner(){m_startNode.resize(8); m_endNode.resize(8); m_pathGenNum = 0; m_knownWorld.iNumObstacles = 0;}
	~CPathPlanner(){}
	
	void SetCondition(setting_t P){m_P = P;}
	void SetWorld(worldLine_t world){m_world = world;}
	void SetRobotState(CURB_Vector_3D robotPosition, double robotAngle);
	void SetLRF(CLRFModel lrf){m_LRF = lrf;}
	void SetvLRF(CVirtualLRF vlrf){m_vLRF = vlrf;}
	void SetExtractor(double errThres){m_extractor.Set_Sensor_Model(m_LRF); m_extractor.Set_Err_Thres(errThres);}
	void SetStartNode(CURB_Vector_3D startNode);
	void SetEndNode(CURB_Vector_3D endNode){m_endNode << endNode.x, endNode.y, endNode.z, 0, 0, 0, 0, 0;}
	
	CURB_Vector_3D GetRobotPosition(){return m_robotPosition;}
	double GetRobotAngle(){return m_robotAngle;}
	double GetPsi(){return m_psi;}
	MatrixXd GetPathX(){return m_PathSegX;}
	MatrixXd GetPathY(){return m_PathSegY;}
	MatrixXd GetPathZ(){return m_PathSegZ;}
	int GetNumKnownObstacles(){return m_knownWorld.iNumObstacles;}
	
	void UpdateExtractorDbMap();
	void UpdateLRF();
	void UpdateLRF_ros(vector<float> newData){m_LRF.SetRange(newData);}
	void UpdateKnownWorld();
	
	double calculateDMax(double kappa);
	void GenPath();
	bool IsArrived();
	bool CollisionCheck();
	void StackCurve();
	void SaveData();
};

class PathNode {
private:
	CURB_Point_3D m_position;
	bool m_isLastNode;
	double m_cost, m_extLength;
	double m_abs;
	PathNode* m_parent;

	int m_index;
	int m_parentIndex;

public:
	PathNode ();
	PathNode (float x, float y, float z, bool isLastNode, double cost, double extLength, double abs, int parentIndex, int index, PathNode *p) {
		setValue(x, y, z, isLastNode, cost, extLength, abs, parentIndex, index, p);
	}
	PathNode (VectorXd vecNode) {
		PathNode(vecNode(0), vecNode(1), vecNode(2), (vecNode(3)==1), vecNode(4),
		         vecNode(5), vecNode(6), vecNode(7), -1, NULL);
	}
	PathNode (float x, float y, float z) { setPosition(x, y, z); }
	PathNode (CURB_Point_3D p) { setPosition(p); }

	void setValue(float x, float y, float z, bool isLastNode, double cost, double extLength, double abs, int parentIndex, int index, PathNode *pn) {
		setPosition(x, y, z); 	m_isLastNode = isLastNode; 	m_cost = cost; 	m_extLength = extLength; 	m_abs = abs;
		m_index = index;		m_parentIndex = parentIndex;	m_parent = pn;
	}
	void setValue(CURB_Point_3D p, bool isLastNode, double cost, double extLength, double abs, int parentIndex, int index, PathNode *pn) {
		setPosition(p); 		m_isLastNode = isLastNode; 	m_cost = cost; 	m_extLength = extLength; 	m_abs = abs;
		m_index = index;		m_parentIndex = parentIndex;	m_parent = pn;
	}

	void setPosition(CURB_Point_3D p){ this->m_position = p; }
	void setPosition(float x, float y, float z){ this->m_position = CURB_Point_3D(x, y, z); }
	CURB_Point_3D getPosition() { return m_position; }

    bool operator <(const PathNode &other) const {
         return m_cost < other.m_cost;
    }
};
