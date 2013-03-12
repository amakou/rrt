//////////////////////////////////////////////////////////////////////////////////////////////////
// Toward humanoid path planner in realistic environments
// Soo-Hyun Ryu       okrsh@korea.ac.kr
// Nakju Lett Doh	  nakju@korea.ac.kr
// More information   http://urobot.korea.ac.kr/h_board/view_popup.php?bbs_id=mainFrame&doc_num=8
//////////////////////////////////////////////////////////////////////////////////////////////////

#include "URB_XState.h"
#include <stdlib.h>

// URB_Vector_2D class
float CURB_Vector_2D::DotProduct(const CURB_Vector_2D& v2) const
{
	return x*v2.x + y*v2.y;
}
float CURB_Vector_2D::CrossProduct(const CURB_Vector_2D& v2) const
{	
	return x * v2.y - y * v2.x; 
}
float CURB_Vector_2D::norm() const
{ 
	return sqrt(x*x + y*y); 
}
const CURB_Vector_2D CURB_Vector_2D::vecRotate(float ang) const
{
	// ang : degree
	float tmpAng = deg2rad(ang);
	float A = cos(tmpAng),	B = sin(tmpAng);
	return CURB_Vector_2D(A*x - B*y, A*y + B*x);
}

float CURB_Vector_2D::calAngle(const CURB_Vector_2D &v2) const
{
	// angle to v2
	// left direction : +, right direction : -
	// angle range : -180 deg ~ +180 deg

	float absMag,direct,tmp;
	float disP=this->norm();
	float disC=v2.norm();
	float min, max;

	if ((disP==0) || (disC==0))	return 0;

	tmp = this->DotProduct(v2)/(disP*disC);
	
	if (tmp > 1)

	  min = 1;

	else

	  min = tmp;

	

	if (tmp > -1)

	  max = tmp;

	else

	  max = -1;

	

	tmp = (tmp>0)?min:max; // for correcting error from variable type
		
	absMag = rad2deg(acos(tmp));  //0~180'
	direct = this->CrossProduct(v2); // left:+, right:-

	return (direct<0? -1:1)*absMag;
}

CURB_hPoint_2D CURB_ihPoint_2D::ih2h_2D()
{ 
	return CURB_hPoint_2D(*this,1);
}

//// End of URB_Vector_2D class /////////


// URB_Vector_3D class

float CURB_Vector_3D::DotProduct(const CURB_Vector_3D& v2) const
{
	return x*v2.x + y*v2.y + z*v2.z;
}

CURB_Vector_3D CURB_Vector_3D::CrossProduct(const CURB_Vector_3D& v2) const
{
	CURB_Vector_3D result;

	result.x = y * v2.z - z * v2.y;
	result.y = z * v2.x - x * v2.z;
	result.z = x * v2.y - y * v2.x;

	CURB_Vector_3D result2;
	
	result2 = result;
	
	return result2;
}

float CURB_Vector_3D::norm() const
{ 
	return sqrt(x*x + y*y + z*z);
}

CURB_ihPoint_2D CURB_hPoint_2D::h2ih_2D()
{
	CURB_ihPoint_2D X_ih2D = *this;
	if( fabs(this->z) < EPS)
	{
		//TRACE("h2ih_2D: WARNING! Point at infinity is changed to homogeneous coordinate\n");
		//TRACE("h2ih_2D: To avoid singular case, it is replaced by epsilon");
		this->z = float(sign(this->z)*EPS);
	}

	X_ih2D /= this->z;

	return X_ih2D;
}



//// End of URB_Vector_3D class /////////


