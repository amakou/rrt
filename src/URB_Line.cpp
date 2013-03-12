//////////////////////////////////////////////////////////////////////////////////////////////////
// Toward humanoid path planner in realistic environments
// Soo-Hyun Ryu       	okrsh@korea.ac.kr
// Nakju Lett Doh	nakju@korea.ac.kr
// More information   	http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
//////////////////////////////////////////////////////////////////////////////////////////////////

#include "URB_Line.h"

#include <stdlib.h>

// URB_Line class
CURB_hPoint_2D CURB_Line::calIntersection(CURB_Line &line2, int *f_intersect)
{
	*f_intersect = 0;
	CURB_hPoint_2D hIntersecPnt;
	hIntersecPnt = CURB_Vector_3D(a,b,c).CrossProduct(CURB_Vector_3D(line2.a,line2.b,line2.c));
	
	if( fabs(hIntersecPnt.z) <= 1000*EPS )
	{
		float lineDis=this->GetDistance(line2);
		
		if( lineDis <= 1.0e-5 )
			*f_intersect = 2;
		else
			*f_intersect = 0;
		
		return CURB_hPoint_2D(0,0,0);
	}
	
	*f_intersect = 1;
	return hIntersecPnt;
}


float CURB_Line::GetDistance(CURB_Point_2D &Point)
{
	CURB_Vector_2D tmp(a,b);
	
	return (tmp.DotProduct(Point)+c)/tmp.norm();
}


float CURB_Line::GetDistance(CURB_Line &line2)
{
	CURB_ihPoint_2D tmpPnt(0,0);
	if(a!=0)	tmpPnt = CURB_ihPoint_2D(-c/a,0);
	else		tmpPnt = CURB_ihPoint_2D(0,-c/b);
	
	return line2.GetDistance(tmpPnt);
}

//// End of URB_Line class ////////////




// URB_Segment class
float CURB_Segment::GetLength() const
{
	return (endPnt-startPnt).norm();
}

CURB_hPoint_2D CURB_Segment::calIntersection(CURB_Segment &seg2, int *f_intersect)
{
	CURB_hPoint_2D X_intersect;
	*f_intersect=0;

	// STEP1 : Get line Equations
	CURB_Line line1 = this->GetLineParameter(); 
	CURB_Line line2 = seg2.GetLineParameter();

	// STEP 2 : Get Temporary Intersection Point
	CURB_hPoint_2D t_X_intersection = line1.calIntersection(line2,f_intersect);
	if(*f_intersect!=1)	return CURB_hPoint_2D(0,0,0);
	
	// STEP 3 : Get X_i11, X_i12, X_i21, X_i22 by composition
	CURB_ihPoint_2D X_world_i = -t_X_intersection.h2ih_2D();
	CURB_ihPoint_2D X_i11,X_i12,X_i21,X_i22;

	X_i11 = X_world_i + this->startPnt;
	X_i12 = X_world_i + this->endPnt;
	X_i21 = X_world_i + seg2.startPnt;
	X_i22 = X_world_i + seg2.endPnt;

	// STEP 4 : Project it to orthogonal coordinate
	float magX_i11 = X_i11.norm();
	float magX_i12 = X_i12.norm();
	float magX_i21 = X_i21.norm();
	float magX_i22 = X_i22.norm();

	CURB_Vector_2D unit_vec1, unit_vec2;

	if( magX_i11 < 100*EPS && magX_i12 < 100*EPS )
	{
		//TRACE("calIntersection: WARNING! Points are too close to intersection points. Please use another method to check the intersection.\n");
	}
	else if( magX_i11 < 100*EPS)
		unit_vec1 = X_i12/magX_i12;
	else
		unit_vec1 = X_i11/magX_i11;

	if( magX_i21 < 100*EPS && magX_i22 < 100*EPS )
	{
		//TRACE("calIntersection: WARNING! Points are too close to intersection points. Please use another method to check the intersection.\n");
	}
	else if( magX_i21 < 100*EPS)
		unit_vec2 = X_i22/magX_i22;
	else
		unit_vec2 = X_i21/magX_i21;

	float a=unit_vec1.x, b=unit_vec2.x, c=unit_vec1.y, d=unit_vec2.y;
	inverse(a,b,c,d);
	CURB_ihPoint_2D t_Inv1(a,b),t_Inv2(c,d);

	CURB_ihPoint_2D P11(t_Inv1.DotProduct(X_i11),t_Inv2.DotProduct(X_i11));
	CURB_ihPoint_2D P12(t_Inv1.DotProduct(X_i12),t_Inv2.DotProduct(X_i12));
	CURB_ihPoint_2D P21(t_Inv1.DotProduct(X_i21),t_Inv2.DotProduct(X_i21));
	CURB_ihPoint_2D P22(t_Inv1.DotProduct(X_i22),t_Inv2.DotProduct(X_i22));
	
	// STEP 5 : Check the intersection point
	if(P11.x * P12.x <= 0 && P21.y * P22.y <=0)
	{
		X_intersect = t_X_intersection;

		CURB_ihPoint_2D ih_X_intersect = X_intersect.h2ih_2D();
		
		float dist1 = (ih_X_intersect-this->startPnt).norm();
		float dist2 = (ih_X_intersect-this->endPnt).norm();
		float dist3 = (ih_X_intersect-seg2.startPnt).norm();
		float dist4 = (ih_X_intersect-seg2.endPnt).norm();

		dist1 = dist1<dist2 ? dist1 : dist2;
		dist3 = dist3<dist4 ? dist3 : dist4;

		float dist = dist1<dist3 ? dist1 : dist3;

		if( dist < 1000*EPS)
			*f_intersect = 2;
		else
			*f_intersect = 1;
	}
	else
	{
		*f_intersect = 0;
		X_intersect = CURB_hPoint_2D(0,0,0);
	}


	return X_intersect;
}

void CURB_Segment::calLineParameter()
{
	// cross
	CURB_Vector_3D lineParam;
	lineParam = CURB_Vector_3D(startPnt.x,startPnt.y,1).CrossProduct(CURB_Vector_3D(endPnt.x,endPnt.y,1));
	// normalize
	float mag = lineParam.norm();
	if(mag!=0)
	{
		lineParam/=mag;
		a = lineParam.x;
		b = lineParam.y;
		c = lineParam.z;
	}
	
	if( (fabs(a) < EPS)&&(fabs(b) < EPS)&&(fabs(c) < EPS) )
	{
		//TRACE("line_intersection_from_four_points > line_from_two_points_2D: line_from_two_points_2D: Warning! Line is not constituted!");
	}
}

CURB_Segment CURB_Segment::GetMinSeg(CURB_Point_2D &Point)
{
	CURB_Vector_2D v = endPnt - startPnt;
	CURB_Vector_2D w = Point - startPnt;
	CURB_Segment minSeg;
	
	float c1 = w.DotProduct(v);
	if ( c1 <= 0 )	return CURB_Segment(Point,startPnt);
	
	float c2 = v.DotProduct(v);
	if ( c2 <= c1 )	return CURB_Segment(Point,endPnt);
	
	float b = c1 / c2;
	CURB_Point_2D Pb = startPnt + v*b;
	
	return CURB_Segment(Point,Pb);
}

// reference : http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm
CURB_Segment CURB_Segment::GetMinSeg(CURB_Segment &Seg)
{
	if( Seg.startPnt.norm() > Seg.endPnt.norm() ) // for correcting error from variable type
		Seg = CURB_Segment(Seg.endPnt,Seg.startPnt);

	CURB_Vector_2D  u = this->endPnt - this->startPnt;
	CURB_Vector_2D  v = Seg.endPnt - Seg.startPnt;
	CURB_Vector_2D	w = this->startPnt - Seg.startPnt;

	CURB_Vector_2D zeroVec(0,0);
	if(u==zeroVec && v==zeroVec)	return CURB_Segment(this->startPnt,Seg.startPnt);
	else if(u==zeroVec)				return Seg.GetMinSeg(this->startPnt);
	else if(v==zeroVec)				return this->GetMinSeg(Seg.startPnt);
	
	float a = u.DotProduct(u);        // always >= 0
	float b = u.DotProduct(v);
	float c = v.DotProduct(v);        // always >= 0
	float d = u.DotProduct(w);
	float e = v.DotProduct(w);
	float D = a*c - b*b;       // always >= 0
	float sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
	float tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0
	
	// compute the line parameters of the two closest points
	if (D < EPS)	// the lines are almost parallel
	{
		sN = 0.0;        // force using point P0 on segment S1
		sD = 1.0;        // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else	// get the closest points on the infinite lines
	{
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < 0.0)
		{
			// sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD)
		{
			// sc > 1 => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}
	
	if (tN < 0.0)	// tc < 0 => the t=0 edge is visible
	{
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD)	// tc > 1 => the t=1 edge is visible
	{
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d + b);
			sD = a;
		}
	}
	
	// finally do the division to get sc and tc
	sc = (fabs(sN) < EPS ? 0.0f : sN / sD );
	tc = (fabs(tN) < EPS ? 0.0f : tN / tD );
	
	return CURB_Segment(this->startPnt + u*sc,Seg.startPnt + v*tc);
}

float CURB_Segment::GetDistance(CURB_Point_2D &Point)
{
	return GetMinSeg(Point).GetLength();
}

float CURB_Segment::GetDistance(CURB_Segment &Seg)
{
	return GetMinSeg(Seg).GetLength();
}

//// End of URB_Segment class ////////////

