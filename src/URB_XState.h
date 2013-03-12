//////////////////////////////////////////////////////////////////////////////////////////////////
// Toward humanoid path planner in realistic environments
// Soo-Hyun Ryu       okrsh@korea.ac.kr
// Nakju Lett Doh	  nakju@korea.ac.kr
// More information   http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
//////////////////////////////////////////////////////////////////////////////////////////////////


#pragma once

//#include "stdafx.h"
#include <math.h>
#include "URB_Define.h"

class CURB_Vector_2D;
class CURB_Vector_3D;
typedef CURB_Vector_2D CURB_Point_2D, CURB_ihPoint_2D;
typedef CURB_Vector_3D CURB_Point_3D, CURB_hPoint_2D;

// 2D Vector class
class CURB_Vector_2D
{
public:
	float x,y;
	CURB_Vector_2D(){ x=0.0f, y=0.0f; }
	CURB_Vector_2D(float X,float Y){ x=X, y=Y; }
	~CURB_Vector_2D(){}

	float DotProduct(const CURB_Vector_2D& v2) const;
	float CrossProduct(const CURB_Vector_2D& v2) const;
	float norm() const;

	// Rotate vector
	// Input
	//	- ang : rotating angle
	// Output
	//	- rotated vector
	const CURB_Vector_2D vecRotate(float ang) const;
	
	// compute relative angle from this vector to target vector
	// Input
	//	- v2 : target vector
	// Output
	//	- angle
	float calAngle(const CURB_Vector_2D &v2) const;


	bool operator==(const CURB_Vector_2D &fP) const {	return (x==fP.x && y==fP.y);}
	bool operator!=(const CURB_Vector_2D &fP) const {	return !(*this == fP);		}

	const CURB_Vector_2D operator+(const CURB_Vector_2D &fP) const {	return CURB_Vector_2D(fP.x+x,fP.y+y);	}
	const CURB_Vector_2D operator-(void) const { return CURB_Vector_2D(-x,-y); }
	const CURB_Vector_2D operator-(const CURB_Vector_2D &fP) const {	return (*this + (-fP)); }

	const CURB_Vector_2D operator*(float n)	const {	return CURB_Vector_2D(n*x,n*y);	}
	const CURB_Vector_2D operator*(int n)	const {	return (*this * float(n));	}

	const CURB_Vector_2D operator/(float n)	const
	{
		if(n!=0.0f)		return CURB_Vector_2D(x/n,y/n);
		else
		{ 
			//TRACE("Warning in URB_XState.h file. Tried to divide by 0\n");
			return CURB_Vector_2D(0,0);
		}
	}
	const CURB_Vector_2D operator/(int n)	const {	return (*this / float(n));	}

	CURB_Vector_2D &operator-=(const CURB_Vector_2D &fP) {	return (*this = *this - fP); }
	CURB_Vector_2D &operator+=(const CURB_Vector_2D &fP) {	return (*this = *this + fP); }
	CURB_Vector_2D &operator*=(float n)	{	return (*this = *this * n); }
	CURB_Vector_2D &operator*=(int n)	{	return (*this = *this * n); }
	CURB_Vector_2D &operator/=(float n)	{	return (*this = *this / n); }
	CURB_Vector_2D &operator/=(int n)	{	return (*this = *this / n); }

	CURB_hPoint_2D ih2h_2D();

};

// 3D Vector class
class CURB_Vector_3D:public CURB_Vector_2D
{
public:
	float z;
	
	CURB_Vector_3D():CURB_Vector_2D(){ z=0; };
	CURB_Vector_3D(float X,float Y,float Z):CURB_Vector_2D(X,Y){ z=Z; }
	CURB_Vector_3D(CURB_Vector_2D fP,float Z):CURB_Vector_2D(fP.x,fP.y){ z=Z; }
	~CURB_Vector_3D(){}
	bool operator==(const CURB_Vector_3D &fP) const {	return (x==fP.x && y==fP.y && z==fP.z);}
	bool operator!=(const CURB_Vector_3D &fP) const {	return !(*this == fP); }

	const CURB_Vector_3D operator+(const CURB_Vector_3D &fP) const {	return CURB_Vector_3D(fP.x+x,fP.y+y,fP.z+z);	}
	const CURB_Vector_3D operator-(void) const { return CURB_Vector_3D(-x,-y,-z); }
	const CURB_Vector_3D operator-(const CURB_Vector_3D &fP) const {	return (*this + (-fP)); }

	const CURB_Vector_3D operator+(const CURB_Vector_2D &fP) const {	return CURB_Vector_3D(fP.x+x,fP.y+y,z);	}
	const CURB_Vector_3D operator-(const CURB_Vector_2D &fP) const {	return (*this + (-fP)); }

	const CURB_Vector_3D operator*(float n)	const {	return CURB_Vector_3D(n*x,n*y,n*z);	}
	const CURB_Vector_3D operator*(int n)	const {	return (*this * float(n));	}

	const CURB_Vector_3D operator/(float n)	const
	{
		if(n!=0.0f)		return CURB_Vector_3D(x/n,y/n,z/n);
		else
		{ 
			//TRACE("Warning in URB_XState.h file. Tried to divide by 0\n");
			return CURB_Vector_3D(0,0,0);
		}
	}
	const CURB_Vector_3D operator/(int n)	const {	return (*this / float(n));	}

	CURB_Vector_3D &operator=(const CURB_Vector_2D &fP)	{ x=fP.x, y=fP.y; return *this;}

	CURB_Vector_3D &operator-=(const CURB_Vector_3D &fP) {	return (*this = *this - fP); }
	CURB_Vector_3D &operator+=(const CURB_Vector_3D &fP) {	return (*this = *this + fP); }
	CURB_Vector_3D &operator*=(float n)	{	return (*this = *this * n); }
	CURB_Vector_3D &operator*=(int n)	{	return (*this = *this * n); }
	CURB_Vector_3D &operator/=(float n)	{	return (*this = *this / n); }
	CURB_Vector_3D &operator/=(int n)	{	return (*this = *this / n); }

	float DotProduct(const CURB_Vector_3D& v2) const;
	CURB_Vector_3D CrossProduct(const CURB_Vector_3D& v2) const;
	float norm() const;

	CURB_ihPoint_2D h2ih_2D();

};
