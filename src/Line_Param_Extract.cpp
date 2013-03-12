//////////////////////////////////////////////////////////////////////////////////////////////////
// Toward humanoid path planner in realistic environments
// Soo-Hyun Ryu       	okrsh@korea.ac.kr
// Nakju Lett Doh	  	nakju@korea.ac.kr
// Keonyong Lee		kuengr00@korea.ac.kr
// More information   	http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
//////////////////////////////////////////////////////////////////////////////////////////////////

#include "Line_Param_Extract.h"
#include "Split_Merge.h"
#include "URB_Core.h"
#include "SaveData.h"

Line_Param_Extract::Line_Param_Extract()
{
	err_thres = 0.5f;
	db_Err_thres = .20f; // 2.0f
	range = NULL;
	robot_position.resize(3,1);
}

Line_Param_Extract::Line_Param_Extract(CLRFModel sensor)
{
	Set_Sensor_Model(sensor);
	Line_Param_Extract();
}

void Line_Param_Extract::Extract_Line()
{
	Sensor_Projection();
	Split_Merge object_2(sensor_points,err_thres);
	object_2.Split_And_Merge();
	lines_after_merge = object_2.Get_Lines_After_Merge();

	for (int i=0;i<=lines_after_merge.cols()-1;i++)
	{
		if ( (abs(lines_after_merge(7,i)) < 0.01) || (abs(lines_after_merge(8,i)) < 0.01) )
			std::cout<<"wrong st point ~~~~"<<std::endl;

		if ( (abs(lines_after_merge(9,i)) < 0.01) || (abs(lines_after_merge(10,i)) < 0.01) )
			std::cout<<"wrong end point ~~~~"<<std::endl;
	}
	used_points = object_2.Get_Used_Points();
}

void Line_Param_Extract::Sensor_Projection()
{
	// coordinate information
	//   y
	//   |
	//   |
	//   ---------------> x
	int i,j;
	int c = -1;
	double x, y, z;
	double x_temp, y_temp;
	double pos_z;
	int numPoints = m_sensor.GetNumRay();
	float max_range = m_sensor.GetMaxRange();
	float min_range = m_sensor.GetMinRange();
	float viewField = m_sensor.GetViewField();
	float resolution = m_sensor.GetResolution();
	float sensor_angle = m_sensor.GetSensorAngPos().y; // pitch
	float robot_height = m_sensor.GetSensorPos().z;
	float robotX = float(robot_position(0,0));
	float robotY = float(robot_position(1,0));
	float robotTheta = float(robot_position(2,0)) - M_PI/2;

	float tmpX,tmpY;
	
	MatrixXd temp(2,numPoints), tempZ(1,numPoints), angMat(2,numPoints);
	
	double ang = 0;//-viewField/2;
	bool minusPitch;

	if (sensor_angle < 0)
	{
		sensor_angle = M_PI/2 + sensor_angle;
		minusPitch = true;
	}
	else
	{
		sensor_angle = M_PI/2 - sensor_angle;
		minusPitch = false;	
	}

	for ( i=0; i<numPoints; i++, ang += resolution )
	{
		if ( range[i] > max_range*0.99 ) continue;	// 0.95 : virtual sensor noise
		if ( range[i] < 1.1*min_range ) continue;
				
		if (minusPitch)
		{
			x =  range[i]*cos(ang);
			y =  range[i]*sin(ang)*sin(sensor_angle);
			z = -range[i]*sin(ang)*cos(sensor_angle);
		}
		else
		{
			x = range[i]*cos(ang);
			y = range[i]*sin(ang)*sin(sensor_angle);
			z = range[i]*sin(ang)*cos(sensor_angle);
		}
		
		// apply robot position
		pos_z = robot_height + z;
		
		if ( fabs(pos_z)  > 0.1)
		{
			//rotate transformation
			x_temp = cos(robotTheta)*x - sin(robotTheta)*y;
			y_temp = sin(robotTheta)*x + cos(robotTheta)*y;
			
			c++;
			temp(0,c) = robotX + x_temp;
			temp(1,c) = robotY + y_temp;
			
			if ((abs(temp(0,c))<0.1) && (abs(temp(1,c))<0.1))
			{
				continue;
				std::cout<<"wrong point projection"<<x_temp<<" "<<y_temp<<" "<<range[i]<<std::endl;
				std::cout<<robotX<<" "<<robotY<<" "<<robotTheta<<" "<<ang<<std::endl;
			}
		}
	} // for ( i=0; i<numPoints; i++, ang += resolution )
	
	sensor_points.resize(2,c+1);
	
	for(i=0; i<=1; i++)
		for(j=0; j<=c; j++)
			sensor_points(i,j) = temp(i,j);

}

void Line_Param_Extract::DB_matching()
{
	int i, j, k;
	int num;
	bool flag;
	
	int num_1 = lines_after_merge.cols();
	int num_2 = db_map_info.rows();
	
	known_lines.resize(0,0);
	unknown_lines.resize(0,0);
	
	for(i=0; i<=num_1-1; i++)
	{
		flag = false;
		
		CURB_Point_2D AftStartPnt(float(lines_after_merge(7,i)),float(lines_after_merge(8,i)));
		CURB_Point_2D AftEndPnt(float(lines_after_merge(9,i)),float(lines_after_merge(10,i)));
		
		for(j=0; j<=num_2-1; j++)
		{
			CURB_Segment dbSeg;
			dbSeg.startPnt = CURB_Point_2D(float(db_map_info(j,3)),float(db_map_info(j,4)));
			dbSeg.endPnt = CURB_Point_2D(float(db_map_info(j,5)),float(db_map_info(j,6)));
			
			if((dbSeg.GetDistance(AftStartPnt) < db_Err_thres)
				&&(dbSeg.GetDistance(AftEndPnt) < db_Err_thres))
			{
				flag = true;
				break;
			}
		}
		
		if ( flag == true )
		{
			num = known_lines.cols();
			
			MatrixXd temp(11,num+1);
			
			if ( num > 0 )
			{
				for(j=0; j<=10; j++)
					for(k=0; k<=num-1; k++)
						temp(j,k) = known_lines(j,k);
			}
			
			for(j=0; j<=10; j++)
				temp(j,num) = lines_after_merge(j,i);
			
			known_lines = temp;
		}
		else
		{
			num = unknown_lines.cols();
			MatrixXd temp(11,num+1);
			
			if ( num > 0 )
			{
				for(j=0; j<=10; j++)
					for(k=0; k<=num-1; k++)
						temp(j,k) = unknown_lines(j,k);
			}
			
			for(j=0; j<=10; j++)
				temp(j,num) = lines_after_merge(j,i);
			
			unknown_lines = temp;
		}
	}
}
