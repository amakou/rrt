//////////////////////////////////////////////////////////////////////////////////////////////////
// Toward humanoid path planner in realistic environments
// Soo-Hyun Ryu       	okrsh@korea.ac.kr
// Nakju Lett Doh		nakju@korea.ac.kr
// Keonyong Lee		kuengr00@korea.ac.kr
// More information   	http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include "cmath" // The cmath(complex math) header file in C++

//#define SCPPNT_USE_REGIONS //for using partial information of matrix
#define SCPPNT_NO_DIR_PREFIX // this part is necessary to use header file properly
//see the structure of head file

// #include "../LineExt Include/SCPPNT/cmat.h"
// #include "../LineExt Include/SCPPNT/transv.h" //for using transpose function
// #include "../LineExt Include/jama125/jama_svd.h"

#include "LRFModel.h"
#include "URB_Define.h"

using namespace std;
// using namespace SCPPNT;
// using namespace JAMA;

class Line_Param_Extract
{
private:
	CLRFModel m_sensor;
	
	double err_thres;
	double db_Err_thres;
	double* range;
	MatrixXd robot_position;
	MatrixXd sensor_points;
	MatrixXd db_map_info;
	MatrixXd lines_after_merge;
	MatrixXd used_points;
	MatrixXd known_lines;
	MatrixXd unknown_lines;
	void Sensor_Projection();
	
public:
	Line_Param_Extract();
	Line_Param_Extract(CLRFModel sensor);
	~Line_Param_Extract(){ if(range!=NULL) delete range; }
	
	void Set_Sensor_Model(CLRFModel sensor){ m_sensor = sensor; }
	
	void Set_Robot_Position(double* _robot_position)
	{
		for(int i=0; i<=1; i++)
			robot_position(i,0) = _robot_position[i];
		
		robot_position(2,0) = deg2rad(_robot_position[2]);
	}
	
	
	void Set_Err_Thres(double _err_thres){ err_thres = _err_thres;}
	
	void Set_Range(double* _range)
	{
		range = new double[m_sensor.GetNumRay()];
		
		for(int i=0;i<m_sensor.GetNumRay();i++)	range[i] = _range[i];
	}
	
	void Set_Db_Map(MatrixXd &_db_map_info){ db_map_info = _db_map_info; }
	
	void Extract_Line();
	void DB_matching();
	MatrixXd Get_Unknown_Line(){ return unknown_lines; }
	MatrixXd Get_Known_Line(){ return known_lines; }
};
