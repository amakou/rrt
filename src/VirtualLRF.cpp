/////////////////////////////////////////////////////////////////////////////////////////////
// Basic line-based path planner for a humanoid robot
// Soo-Hyun Ryu       	okrsh@korea.ac.kr
// Nakju Lett Doh	nakju@korea.ac.kr
// More information   	http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
/////////////////////////////////////////////////////////////////////////////////////////////

#include "VirtualLRF.h"
#include "Functions.h"

double normal_random( double mean, double sigma )
{
	static double V1, V2, S;
	static int phase = 0;
	double X;
	
	if (phase == 0)
	{
		do
		{
			double U1 = (double)Uniform01();
			double U2 = (double)Uniform01();
			
			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
			
		} while (S >= 1 || S == 0);
		
		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;
	
	return mean + sigma*X;
}

void CVirtualLRF::Update_Sensor_Range(CURB_Vector_3D &robotPos,worldLine_t &lineMap)
{
	Update_Sensor_Point(robotPos,lineMap);
	m_data = this->Point2Range(m_ihX_intersection,robotPos);
}

void CVirtualLRF::Update_Sensor_Point(CURB_Vector_3D &robotPos,worldLine_t &lineMap)
{
	// R : 3x3 matrix R = [R1;R2;R3]
	CURB_Vector_3D R1(cos(deg2rad(robotPos.z)+m_angPos.x), -sin(deg2rad(robotPos.z)+m_angPos.x),robotPos.x+m_pos.x);
	CURB_Vector_3D R2(sin(deg2rad(robotPos.z)+m_angPos.x),  cos(deg2rad(robotPos.z)+m_angPos.x),robotPos.y+m_pos.y);
	CURB_Vector_3D R3(0,0,1);
	
	int i=0,j=0;
	CURB_ihPoint_2D temp_ray_point;
	vector<CURB_ihPoint_2D> ihX_LRF;
	float ray_angle = -m_viewField/2;
	float sensor_angle = m_angPos.y;
	float tmpX,tmpY;
	
	for (i=0; i<m_numRay; i++,ray_angle += m_resolution)
	{
		tmpX = cos(ray_angle)*cos(sensor_angle);
		tmpY = sin(ray_angle);
		//tmpZ = cos(ray_angle)*sin(sensor_angle);
		
		CURB_Vector_2D tmpVec(tmpX,tmpY);
		tmpVec = tmpVec*(m_maxRange/tmpVec.norm());

		CURB_Vector_3D tempRay(tmpVec,1);		//(m_maxRange*cos(ray_angle)*cos(sensor_angle),m_maxRange*sin(ray_angle),1);
		temp_ray_point = CURB_hPoint_2D(R1.DotProduct(tempRay),R2.DotProduct(tempRay),R3.DotProduct(tempRay)).h2ih_2D();
		ihX_LRF.push_back(temp_ray_point);
	}
	
	CURB_ihPoint_2D ihX_origin(robotPos.x,robotPos.y);
	
	float MAX = 99999;
	float ihX_intersection_min = MAX;
	int num_line = (int)lineMap.iNumObstacles;
	
	m_ihX_intersection.clear();
	
	for (i=0;i<m_numRay;i++)
	{
		m_ihX_intersection.push_back(CURB_ihPoint_2D(0,0));	// initialize
		ihX_intersection_min = MAX;
		CURB_Segment seg1(ihX_origin,ihX_LRF[i]), seg2;
		int f_intersection=0;
		
		for (j=0;j<num_line;j++)
		{
			f_intersection=0;
			seg2 = lineMap.lineSeg[j];
			CURB_hPoint_2D hX_intersect = seg1.calIntersection(lineMap.lineSeg[j],&f_intersection);
			
			if ( f_intersection==1 )
			{
				CURB_ihPoint_2D ihX_intersect = hX_intersect.h2ih_2D();
				float mag = (ihX_origin-ihX_intersect).norm();
				
				if ( mag < ihX_intersection_min)
				{
					ihX_intersection_min = mag;
					m_ihX_intersection[i] = ihX_intersect;
				}
			}
		}
		
		if ( fabs(ihX_intersection_min - MAX) < 100*EPS)	// NO intersection
		{
			m_ihX_intersection[i] = ihX_LRF[i];
		}
	}

	// Add random number
	vector<CURB_ihPoint_2D> ih_Noise;
	
	CURB_ihPoint_2D tR1(R1.x,R1.y), tR2(R2.x,R2.y);
	float noise = 0;
	
	srand((unsigned int)time(NULL));
	
	ray_angle = -m_viewField/2;
	
	for (i=0;i<m_numRay;i++)
	{
		noise = m_noise_st_dev*(float)normal_random(0,1);	// rand 함수 수정
		CURB_ihPoint_2D tempRay(noise*cos(ray_angle),noise*sin(ray_angle));
		CURB_ihPoint_2D temp_noise(tR1.DotProduct(tempRay),tR2.DotProduct(tempRay));
		ih_Noise.push_back(temp_noise);
		ray_angle += m_resolution;
	}
	
	for (i=0;i<m_numRay;i++)	m_ihX_intersection[i] += ih_Noise[i];
}

vector<float> CVirtualLRF::Point2Range(vector<CURB_ihPoint_2D> &ihX_point,CURB_Vector_3D &robotPos)
{
	CURB_Vector_2D robotP(robotPos);
	
	int num = (int)ihX_point.size();
	vector<float> data;
	data.resize(num);
	
	// 2D distance
	if (m_angPos.y==0)
	{
		for (int i=0;i<num;i++)	data[i] = (ihX_point[i]-robotP).norm();
		return data;
	}

	// 3D distance
	float tmpX,tmpY,tmpZ;
	float sensor_angle = m_angPos.y;
	float h;
	float newDis;
	
	for (int i=0; i<num; i++)
	{
		CURB_ihPoint_2D tmpPos = ihX_point[i]-robotP;
		tmpPos = tmpPos.vecRotate(-robotPos.z);
		tmpX = tmpPos.x;
		tmpY = tmpPos.y;
		
		h = tmpX*tan(sensor_angle);
		tmpZ = m_pos.z + h;
		
		if (tmpZ>=0)
			newDis = sqrt(tmpX*tmpX+tmpY*tmpY+h*h);
		else
			newDis = fabs(m_pos.z/h)*sqrt(tmpX*tmpX+tmpY*tmpY+h*h);
		
		if ( newDis > m_maxRange ) newDis = m_maxRange;
		
		data[i] = newDis;
	}
	
	return data;
}

vector<CURB_ihPoint_2D> CVirtualLRF::Range2Point(vector<float> &data,CURB_Vector_3D &robotPos)
{
	// R : 3x3 matrix R = [R1;R2;R3]
	CURB_Vector_3D R1(cos(deg2rad(robotPos.z+m_angPos.x)), -sin(deg2rad(robotPos.z+m_angPos.x)),robotPos.x+m_pos.x);
	CURB_Vector_3D R2(sin(deg2rad(robotPos.z+m_angPos.x)),  cos(deg2rad(robotPos.z+m_angPos.x)),robotPos.y+m_pos.y);
	CURB_Vector_3D R3(0,0,1);
	
	int i=0,j=0;
	CURB_ihPoint_2D temp_ray_point;
	vector<CURB_ihPoint_2D> ih_point;
	
	float ray_angle = -m_viewField/2;;
	for (i=0; i<m_numRay; i++,ray_angle += m_resolution)
	{
		CURB_Vector_3D tempRay(data[i-1]*cos(ray_angle),data[i-1]*sin(ray_angle),1);
		temp_ray_point = CURB_hPoint_2D(R1.DotProduct(tempRay),R2.DotProduct(tempRay),R3.DotProduct(tempRay)).h2ih_2D();
		ih_point.push_back(temp_ray_point);
	}
	
	return ih_point;
}
