//////////////////////////////////////////////////////////////////////////////////////////////////
// Toward humanoid path planner in realistic environments
// Soo-Hyun Ryu       okrsh@korea.ac.kr
// Nakju Lett Doh	  nakju@korea.ac.kr
// More information   http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
//////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "URB_XState.h"

class CURB_Segment;

class CURB_Line
{
public:
	float a,b,c;	// line parameter. ax+by+c=0
	CURB_Line(){ a=0,b=0,c=0; }
	CURB_Line(float A,float B, float C){ a=A, b=B, c=C; }
	~CURB_Line(){}

	// calculate homogeneous intersection point between this line and input line.
	CURB_hPoint_2D calIntersection(CURB_Line &line2, int *f_intersect);
	
	float GetDistance(CURB_Point_2D &Point);
	float GetDistance(CURB_Line &line2);

};

class CURB_Segment : public CURB_Line
{
public:
	CURB_Point_2D startPnt;
	CURB_Point_2D endPnt;

	CURB_Segment():CURB_Line(){ startPnt = CURB_Point_2D(0,0); endPnt = CURB_Point_2D(0,0); }
	CURB_Segment(CURB_Point_2D p0,CURB_Point_2D p1):CURB_Line(){ startPnt = p0; endPnt = p1; calLineParameter(); }
	CURB_Segment(float x0,float y0, float x1,float y1):CURB_Line(){ startPnt = CURB_Point_2D(x0,y0); endPnt = CURB_Point_2D(x1,y1); calLineParameter(); }
	CURB_Segment(float A,float B,float C,CURB_Point_2D p0,CURB_Point_2D p1):CURB_Line(A,B,C){ startPnt = p0; endPnt = p1; }
	~CURB_Segment(){}

	float GetLength() const;
	CURB_Line GetLineParameter(){ return CURB_Line(a,b,c); }

	// calculate homogeneous intersection point between this line and input line.
	CURB_hPoint_2D calIntersection(CURB_Segment &seg2, int *f_intersect);

	CURB_Segment GetMinSeg(CURB_Point_2D &Point);
	CURB_Segment GetMinSeg(CURB_Segment &Seg);

	float GetDistance(CURB_Point_2D &Point);
	float GetDistance(CURB_Segment &Seg);

private:
	void calLineParameter();
};

