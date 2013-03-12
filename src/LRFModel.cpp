/////////////////////////////////////////////////////////////////////////////////////////////
// Basic line-based path planner for a humanoid robot
// Soo-Hyun Ryu       	okrsh@korea.ac.kr
// Nakju Lett Doh	nakju@korea.ac.kr
// More information   	http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
/////////////////////////////////////////////////////////////////////////////////////////////

#include "LRFModel.h"

CLRFModel::CLRFModel(void)
{
}
CLRFModel::CLRFModel(CURB_Point_3D sensorPos,CURB_Vector_3D sensorAngPos, float maxRange, float minRange, float viewField,float resolution)
{
	m_pos = sensorPos;
	m_angPos = sensorAngPos;
	m_maxRange = maxRange;
	m_minRange = minRange;
	m_viewField = viewField;
	m_resolution = resolution;
	m_numRay = round_urb(viewField/resolution)+1;
}

CLRFModel::~CLRFModel(void)
{
}

void CLRFModel::SetParam(CURB_Point_3D sensorPos,CURB_Vector_3D sensorAngPos,float maxRange, float minRange, float viewField,float resolution)
{
	*this = CLRFModel(sensorPos,sensorAngPos,maxRange,minRange,viewField,resolution);
}
