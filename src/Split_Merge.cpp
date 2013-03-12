//////////////////////////////////////////////////////////////////////////////////////////////////
// Toward humanoid path planner in realistic environments
// Soo-Hyun Ryu       	okrsh@korea.ac.kr
// Nakju Lett Doh		nakju@korea.ac.kr
// Keonyong Lee		kuengr00@korea.ac.kr
// More information   	http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
//////////////////////////////////////////////////////////////////////////////////////////////////

#include "Split_Merge.h"
#include "Line_Param_Extract.h"
#include <Eigen/SVD>


using namespace Eigen;

// sick
/*Split_Merge::Split_Merge()
{
	split_err_thres = 0.05f; // 3.0
	merge_err_thres = 0;
	point_thres = 30; // 3
	sensor_points.resize(0,0);
}

Split_Merge::Split_Merge(MatrixXd &_sensor_points, double _err_thres)
{
	point_thres = 20;		// 2
	split_err_thres = 0.05f;	// 1.0
	merge_err_thres = _err_thres;
	sensor_points = _sensor_points;
}*/

Split_Merge::Split_Merge()
{
	split_err_thres = 0.05f; // 3.0
	merge_err_thres = 0;
	point_thres = 10; // 3
	sensor_points.resize(0,0);
}

// used in RRT code
Split_Merge::Split_Merge(MatrixXd &_sensor_points, double _err_thres)
{
	point_thres = 5;		// 2
	split_err_thres = .1f;	// 1.0
	merge_err_thres = _err_thres;
	sensor_points = _sensor_points;
}

void Split_Merge::Split_And_Merge()
{
	splitted_lines.resize(0,0);
	temp_param.resize(0,0);
	corresponding_points.resize(0,0);
	lines_after_merge.resize(0,0);
	num_of_lines = 0;
	MatrixXd temp_group = sensor_points;
	Split(temp_group);
	Merge();
}

void Split_Merge::Split(MatrixXd temp_group)
{
	int i, j, k;
	int num;
	int num_of_points = temp_group.cols();
	MatrixXd temp;
	MatrixXd group_1, group_2, temp_group_2;

	temp.resize(11,1);
	

	if (num_of_points > point_thres)
	{
		double x_1 = temp_group(0,0);
		double y_1 = temp_group(1,0);
		double x_2 = temp_group(0,num_of_points-1);
		double y_2 = temp_group(1,num_of_points-1);
		int check = 0;
		
		//line equation: ax + by + c = 0 form
		double a = y_2 - y_1;
		double b = x_1 - x_2;
		double c = y_2*(x_2-x_1) - x_2*(y_2-y_1);
		
		double max_distance = 0;
		int max_point = 0;
		
		for(i=2; i<=num_of_points-1; i++)
		{
			double x = temp_group(0,i-1);
			double y = temp_group(1,i-1);			
			double distance = fabs(a*x + b*y + c) / sqrt(pow(a,2)+pow(b,2));
			
			if ( distance > max_distance )
			{
				max_distance = distance;
				max_point = i;
			}
		}
		
		if ( max_distance < split_err_thres )
		{
			for (i=0; i<=num_of_points-2-1; i++)
			{
				double mag_1 = sqrt(pow(temp_group(0,i+1)-temp_group(0,i),2) + pow(temp_group(1,i+1)-temp_group(1,i),2));
				double mag_2 = sqrt(pow(temp_group(0,i+2)-temp_group(0,i+1),2) + pow(temp_group(1,i+2)-temp_group(1,i+1),2));
				
				if ( mag_2/mag_1 < 0.5 )
				{
					///
					group_1.resize(2,i+1);
					group_2.resize(2,num_of_points-(i+1));
					
					for(k=0; k<=1; k++)
					{
						for(j=0; j<=i; j++)	group_1(k,j) = temp_group(k,j);
						
						for(j=i+1; j<=num_of_points-1; j++)	group_2(k,j-i-1) = temp_group(k,j);
					}
					
					Split(group_1);
					Split(group_2);
					check = 1;
					break;
				}
			}
			
			if (check == 0)
			{
				for (i=num_of_points-1; i>=2; i--)
				{
					double mag_1 = sqrt(pow(temp_group(0,i-1)-temp_group(0,i-2),2) + pow(temp_group(1,i-1)-temp_group(1,i-2),2));
					double mag_2 = sqrt(pow(temp_group(0,i)-temp_group(0,i-1),2) + pow(temp_group(1,i)-temp_group(1,i-1),2));
					
					if ( mag_2/mag_1 > 2 )
					{
						group_1.resize(2,i-1);
						group_2.resize(2,num_of_points-i+1);
						
						for(k=0; k<=1; k++)
						{
							for(j=0; j<=i-2; j++) group_1(k,j) = temp_group(k,j);
					
							for(j=i-1; j<=num_of_points-1; j++) group_2(k,j-i+1) = temp_group(k,j);
						}
						Split(group_1);
						Split(group_2);
						check = 1;
						break;
					}
				}
			}
			
			if (check == 0)
			{
				num_of_points = temp_group.cols();
				temp_param = Extract_Line_Param(temp_group);
				
				for(i=0; i<=5; i++) temp(i,0) = temp_param(i,0);
				
				temp(6,0) = num_of_points;
				
				for(i=7; i<=10; i++)	temp(i,0) = temp_param(i-7,1); //temp is 11 X 1 matrix

				num_of_lines++;
				num = corresponding_points.cols() + 1;

				MatrixXd temp_lines(11,num_of_lines);
				MatrixXd temp_points(2,num+num_of_points-1);
				
				if ( num_of_lines > 1 )
				{
					for(i=0; i<=10; i++)
						for(j=0; j<num_of_lines-1; j++)
							temp_lines(i,j) = splitted_lines(i,j);
				}
				for(i=0; i<=10; i++)
					temp_lines(i,num_of_lines-1) = temp(i,0);
				
				splitted_lines = temp_lines;
				
				if ( num_of_lines > 1 )
				{
					for(i=0; i<=1; i++)
						for(j=0; j<num-1; j++)
							temp_points(i,j) = corresponding_points(i,j);
				}
				
				for(i=0; i<=1; i++)
					for(j=num-1; j<num-1+num_of_points-1; j++)
						temp_points(i,j) = temp_group(i,j-(num-1));
		
				corresponding_points = temp_points;
			}
		}
		else
		{
			group_1.resize(2,max_point);
			group_2.resize(2,num_of_points-max_point+1);
			
			for(i=0; i<=1; i++)
			{
				for(j=0; j<=max_point-1; j++)
					group_1(i,j) = temp_group(i,j);
				
				for(j=max_point-1; j<=num_of_points-1; j++)
					group_2(i,j-max_point+1) = temp_group(i,j);
			}
			
			Split(group_1);
			Split(group_2);
		}
	}
}

void Split_Merge::Merge()
{
	//splitted_lines are the matrix of 11 x n
	//a, b, c, length, roh, psi, n(number of points compose of the line
	//remained four points : first point and last point of each line.
	MatrixXd new_group;
	MatrixXd param(11,1);
	
	int number_of_groups = splitted_lines.cols();
	int change = 1; //check for updating
	int i, j;
	
	while ( change > 0 )
	{
		MatrixXd lines;
		change = 0;
		int start_point = 1;
		int count = 1;

		while ( count < number_of_groups )
		{
			int final_point = start_point - 1 + (int)(splitted_lines(6,count-1) + splitted_lines(6,count));
			int middle_point = start_point + (int)splitted_lines(6,count-1) - 1;
			double distance = sqrt(pow(corresponding_points(0,middle_point-1)-corresponding_points(0,middle_point),2) + pow(corresponding_points(1,middle_point-1)-corresponding_points(1,middle_point),2));
			double dist_1 = sqrt(pow(corresponding_points(0,middle_point-1)-corresponding_points(0,middle_point-2),2) + pow(corresponding_points(1,middle_point-1)-corresponding_points(1,middle_point-2),2));
			double dist_2 = sqrt(pow(corresponding_points(0,middle_point)-corresponding_points(0,middle_point+1),2) + pow(corresponding_points(1,middle_point)-corresponding_points(1,middle_point+1),2));
			
			
			new_group.resize(2,int(final_point-start_point+1));
			
			for(i=0; i<=1; i++)
				for(j=start_point-1; j<=final_point-1; j++)
					new_group(i,j-(start_point-1)) = corresponding_points(i,j);
			
			temp_param = Extract_Line_Param(new_group);
			
			for(i=0; i<=5; i++)
				param(i,0) = temp_param(i,0);
			
			int num = lines.cols();
			MatrixXd temp_lines(11,num+1);
			
			if ( param(4,0) < merge_err_thres && distance/dist_1 < 4 && distance/dist_2 < 4)
			{
				std::cout<<param(4,0)<<endl;
				param(6,0) = splitted_lines(6,count-1) + splitted_lines(6,count);
				
				for(i=0; i<=3; i++)
					param(i+7,0) = temp_param(i,1);
				
				if ( num > 0 )
				{
					for(i=0; i<=10; i++)
						for(j=0; j<=num-1; j++)
							temp_lines(i,j) = lines(i,j);
				}
				
				for(i=0; i<=10; i++)
					temp_lines(i,num) = param(i,0);
				
				change = change + 1;
				start_point = final_point + 1;
				count = count + 1;
			}
			else
			{
				if ( num > 0 )
				{
					for(i=0; i<=10; i++)
						for(j=0; j<=num-1; j++)
							temp_lines(i,j) = lines(i,j);
				}
				
				for(i=0; i<=10; i++)
					temp_lines(i,num) = splitted_lines(i,count-1);
				
				start_point = start_point + (int)splitted_lines(6,count-1);
			}
			
			lines = temp_lines;
			count = count + 1;
		}
		
		if ( count == number_of_groups )
		{
			int num = lines.cols();
			MatrixXd temp_lines(11,num+1);
			
			for(i=0; i<=10; i++)
				for(j=0; j<=num-1; j++)
					temp_lines(i,j) = lines(i,j);
			
			for(i=0; i<=10; i++)
				temp_lines(i,num) = splitted_lines(i,count-1);
			
			lines = temp_lines;
		}
		
		splitted_lines = lines;
		number_of_groups = lines.cols();
	}
	
	lines_after_merge = splitted_lines; // 11 rows : a,b,c,length,err_1,err_2,num_points,p1(x),p1(y),p2(x),p2(y)
}


MatrixXd Split_Merge::Extract_Line_Param(MatrixXd &points)
{
	int i, j;
	MatrixXd line_param(6,2);
	int num = points.cols();
	
	//start the process of the least square approximation !!
	MatrixXd A(num,2);
	MatrixXd B(num,1);
	MatrixXd p_1(2,1);
	MatrixXd p_2(2,1);
	MatrixXd line(3,1);

	for(i=0; i<=num-1; i++)
	{
		A(i,0) = points(0,i);
		A(i,1) = 1;
		B(i,0) = points(1,i);
	}
	
	JacobiSVD<MatrixXd> svd(A,ComputeThinU | ComputeThinV);
	VectorXd singular;
	
	singular = svd.singularValues();

	
	MatrixXd a_1;
	MatrixXd a_2;
	MatrixXd inv_a_1(2,2);
	MatrixXd line_temp;
	
	if( singular(0)/singular(singular.rows()-1) < 10000 )
	{
		a_1 = A.transpose()*A;
		
		inv_a_1(0,0) = a_1(1,1);
		inv_a_1(0,1) = -a_1(0,1);
		inv_a_1(1,0) = -a_1(1,0);
		inv_a_1(1,1) = a_1(0,0);
				
		inv_a_1 /= ( a_1(0,0)*a_1(1,1) - a_1(0,1)*a_1(1,0) );
		
		a_2 = A.transpose()*B;
		
		line_temp = inv_a_1 * a_2;
		
		line(0,0) = line_temp(0,0);
		line(1,0) = -1;
		line(2,0) = line_temp(1,0);
		
		p_1(0,0) = 0;
		p_1(1,0) = line(2,0);
		p_2(0,0) = 1;
		p_2(1,0) = line(0,0) + line(2,0);
		//TRACE("\n%.2f %.2f %.2f %.2f ",p_1(1,1),p_1(2,1),p_2(1,1),p_2(2,1));
	}
	else
	{
		for(i=0; i<=num-1; i++)
		{
			A(i,0) = points(1,i);
			A(i,1) = 1;
			B(i,0) = points(0,i);
		}
		
		
		a_1 = A.transpose()*A;
		
		inv_a_1(0,0) = a_1(1,1);
		inv_a_1(0,1) = -a_1(0,1);
		inv_a_1(1,0) = -a_1(1,0);
		inv_a_1(1,1) = a_1(0,0);
		
		inv_a_1 /= ( a_1(0,0)*a_1(1,1) - a_1(0,1)*a_1(1,0) );
		
		a_2 = A.transpose()*B;
		
		line_temp = inv_a_1 * a_2;
		
		line(0,0) = 1;
		line(1,0) = -line_temp(0,0);
		line(2,0) = -line_temp(1,0);
		
		p_1(0,0) = -line(2,0);
		p_1(1,0) = 0;
		p_2(0,0) = -line(1,0) - line(2,0);
		p_2(1,0) = 1;
	}
	
	MatrixXd first_point, last_point;
	
	MatrixXd f_point(2,1);
	MatrixXd l_point(2,1);
	
	f_point(0,0) = points(0,0);
	f_point(1,0) = points(1,0);
	
	l_point(0,0) = points(0,num-1);
	l_point(1,0) = points(1,num-1);
	
	first_point = Projection(f_point,p_1,p_2);
	last_point = Projection(l_point,p_1,p_2);

	double error = 0;
	
	for(i=0; i<num+1-1; i++)
	{
		MatrixXd p(2,1);
		MatrixXd proj_point;
		MatrixXd d_point;

		p(0,0) = points(0,i);
		p(1,0) = points(1,i);
		proj_point = Projection(p,first_point,last_point);
		d_point = p - proj_point;
		
		double distance = sqrt( pow(d_point(0,0),2.0) + pow(d_point(1,0),2.0) );
		error = error + pow(distance,2.0);
	}
	
	MatrixXd len = last_point - first_point;
	double line_length = sqrt( pow(len(0,0),2.0) + pow(len(1,0),2.0) );
	
	double s = sqrt( pow(line(0,0),2) + pow(line(1,0),2) + pow(line(2,0),2) );
	
	for(i=0; i<=2; i++)
		line(i,0) = line(i,0) / s;

	double sample_var = error / (num-1);
	double roh_std_dev = sqrt( sample_var );
	double psi_std_dev = atan(2*sample_var/line_length) / M_PI * 180;
	
	MatrixXd _line(6,1);
	_line(0,0) = line(0,0);
	_line(1,0) = line(1,0);
	_line(2,0) = line(2,0);
	_line(3,0) = line_length;
	_line(4,0) = roh_std_dev;
	_line(5,0) = psi_std_dev;
	
	for(i=0; i<6; i++)
		line_param(i,0) = _line(i,0);
	
	line_param(0,1) = first_point(0,0);
	line_param(1,1) = first_point(1,0);
	line_param(2,1) = last_point(0,0);
	line_param(3,1) = last_point(1,0);
	
	return line_param; //6 x 2 matrix !!
}

MatrixXd Split_Merge::Projection(MatrixXd &vec_1, MatrixXd &line_point_1, MatrixXd &line_point_2)
{
	MatrixXd b = vec_1 - line_point_1;
	MatrixXd a = line_point_2 - line_point_1;
	
	MatrixXd t_1 = a.transpose() * b;
	MatrixXd t_2 = a.transpose() * a;
	
	MatrixXd p;
	
	a *= ( t_1(0,0) / t_2(0,0) );
	MatrixXd proj_point = line_point_1 + a;
	
	return proj_point;
}
