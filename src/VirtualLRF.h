/////////////////////////////////////////////////////////////////////////////////////////////
// Basic line-based path planner for a humanoid robot
// Soo-Hyun Ryu       	okrsh@korea.ac.kr
// Nakju Lett Doh	nakju@korea.ac.kr
// More information   	http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
/////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "URB_Core.h"
#include "LRFModel.h"
#include "Struct.h"

// Generating sensor data Class
class CVirtualLRF:public CLRFModel
{
protected:
    vector<CURB_ihPoint_2D> m_ihX_intersection;	// to display
	float m_noise_st_dev;

	void Update_Sensor_Point(CURB_Vector_3D &robotPos, worldLine_t &lineMap);
		
public:
	CVirtualLRF(void):CLRFModel(){	m_ihX_intersection.clear();	m_noise_st_dev = 0.1f;}
	CVirtualLRF(CLRFModel LRF_model,float noise_st_dev)
		:CLRFModel(LRF_model){ m_ihX_intersection.clear();	m_noise_st_dev = noise_st_dev; }
	CVirtualLRF(CURB_Point_3D sensorPos,CURB_Vector_3D sensorAngPos,float maxRange,float minRange, float viewField,float resolution,float noise_st_dev)
		:CLRFModel(sensorPos,sensorAngPos,maxRange,minRange,viewField,resolution){ m_ihX_intersection.clear();	m_noise_st_dev = noise_st_dev; }
	~CVirtualLRF(void){}

	vector<CURB_ihPoint_2D> &GetIntersecPoint(){ return m_ihX_intersection; }

	void Update_Sensor_Range(CURB_Vector_3D &robotPos,worldLine_t &lineMap);
	
	vector<float> Point2Range(vector<CURB_ihPoint_2D> &ihX_point,CURB_Vector_3D &robotPos);
	vector<CURB_ihPoint_2D> Range2Point(vector<float> &data,CURB_Vector_3D &robotPos);
	

};
