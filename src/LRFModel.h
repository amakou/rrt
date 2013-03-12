/////////////////////////////////////////////////////////////////////////////////////////////
// Basic line-based path planner for a humanoid robot
// Soo-Hyun Ryu       	okrsh@korea.ac.kr
// Nakju Lett Doh	nakju@korea.ac.kr
// More information   	http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
/////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "URB_Core.h"
#include "Struct.h"

class CLRFModel
{
protected:
	vector<float> m_data;

	// sensor parameter
	CURB_Point_3D m_pos;	// sensor position from robot
	CURB_Vector_3D m_angPos;// yaw,pitch,roll of sensor
	float m_maxRange;	// maximum distance
	float m_minRange;	// minimum distance
	float m_viewField;	// Field of view (rad)
	float m_resolution;	// angle between each lasers (rad)
	int m_numRay;		

public:
	CLRFModel();
	CLRFModel(CURB_Vector_3D sensorPos,CURB_Vector_3D angPos,float maxRange,float minRange, float viewField,float resolution);
	~CLRFModel();

	void SetParam(CURB_Vector_3D sensorPos,CURB_Vector_3D angPos,float maxRange,float minRange, float viewField,float resolution);
	vector<float> GetRange(){ return m_data; }
	void SetRange(vector<float> newData){ m_data = newData; }
	CURB_Point_3D GetSensorPos(){ return m_pos; }
	CURB_Vector_3D GetSensorAngPos(){ return m_angPos; }
	float GetMaxRange(){ return m_maxRange; }
	float GetMinRange(){ return m_minRange; }
	float GetViewField(){ return m_viewField; }
	float GetResolution(){ return m_resolution; }
	int GetNumRay(){ return m_numRay; }

};
