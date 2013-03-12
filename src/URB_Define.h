//////////////////////////////////////////////////////////////////////////////////////////////////
// Toward humanoid path planner in realistic environments
// Soo-Hyun Ryu       okrsh@korea.ac.kr
// Nakju Lett Doh	  nakju@korea.ac.kr
// More information   http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
//////////////////////////////////////////////////////////////////////////////////////////////////


#pragma once

typedef int BOOL;
#define TRUE 1
#define FALSE 0

#ifndef PI	
#define PI 3.14159265f	//3.141592653589793
#endif

#ifndef rad2deg
#define rad2deg(rad) ((rad)*180/PI)
#endif

#ifndef deg2rad
#define deg2rad(deg) ((deg)*PI/180)
#endif

#ifndef EPS
#define EPS (1.0e-10)	//(2.2204e-016)	//(pow(2,-52))
#endif

#ifndef sign
#define sign(x)		(x<0? -1:1)
#endif

#ifndef round_urb
#define round_urb(x)	int((x)+sign(x)*0.5f)	// x: float
#endif

inline float AS(float ang)
{
	int Q = int(ang/(2*PI));
	float ret = ang - 2*PI*Q;

	if( ret<0 ) return ret + 2*PI;
	else		return ret;
}

inline float AS_PI(float ang)
{
	float ret = AS(ang);

	if(ret>PI)	return ret - 2*PI;
	else		return ret;
}

// TEMP function
inline void inverse(float &a,float &b,float &c,float &d)
{
	float det = a*d-b*c;
	if(det!=0.0f)
	{
		float tA,tB,tC,tD;
		tA = d/det, tB = -b/det, tC = -c/det, tD = a/det;
		a = tA, b = tB, c = tC, d = tD;
	}
}
