//////////////////////////////////////////////////////////////////////////////////////////////////
// Toward humanoid path planner in realistic environments
// Soo-Hyun Ryu      		okrsh@korea.ac.kr
// Nakju Lett Doh		nakju@korea.ac.kr
// Keonyong Lee		kuengr00@korea.ac.kr
// More information   	http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
//////////////////////////////////////////////////////////////////////////////////////////////////

#define SCPPNT_NO_DIR_PREFIX // this part is necessary to use header file properly
//see the structure of head file
// #include "../LineExt Include/SCPPNT/cmat.h"
#include "Struct.h"

// using namespace SCPPNT;

class Split_Merge
{
private:
	MatrixXd sensor_points;
	MatrixXd temp_param;
	MatrixXd splitted_lines;
	MatrixXd corresponding_points;
	MatrixXd lines_after_merge; // 11 rows : a,b,c,length,err_1,err_2,num_points,p1(x),p1(y),p2(x),p2(y)

	double split_err_thres,merge_err_thres;
	int point_thres;
	int num_of_lines;
	
	void Split(MatrixXd temp_group);
	void Merge();
	MatrixXd Extract_Line_Param(MatrixXd &points);
	MatrixXd Projection(MatrixXd &vec_1,MatrixXd &line_point_1,MatrixXd &line_point_2);

public:
	Split_Merge(MatrixXd &_sensor_points, double _err_thres);
	
	void Split_And_Merge();
	MatrixXd Get_Lines_After_Merge(){	return lines_after_merge;	}
	MatrixXd Get_Used_Points(){	return corresponding_points; }
};
