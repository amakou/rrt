#include "Functions.h"

VectorXd linspace(double d1, double d2, double n)
{
	double test;
	VectorXd y;
	int nF = floor(n);
	
	if( nF > 0 ) {
		y.resize(nF);
		
		for (int i = 0; i<=nF-2; i++) {
			test = d1+i*(d2-d1)/(floor(n)-1);
			y(i) = test;
		}
		y(nF-1) = d2;

	} else {
		y.resize(1);
		y << d2;
	}
	return y;
}


double Uniform01(void)
{
	return rand() / ((double) RAND_MAX + 1);
}
	

/* normal distribution random number generator,
 * using uniform01 and C90 standard math library,
 * can make (RAND_MAX + 1) kinds of values in the interval (-inf, inf).
 *
 * Ref: Box-Muller transform, Basic form in Wikipedia
 * URL: http://en.wikipedia.org/wiki/Box-Muller_transform
 *
 * Ref: Boost Random Number Library, normal distribution
 * URL: http://boost.org/boost/random/normal_distribution.hpp */

double quasi_normal_random(void)
{
	static double R, theta;
	static int change = 1;
	
	if (change)
	{
		double U1 = Uniform01(), U2 = Uniform01(), pi = acos(-1);
		
		/* U1 is in the interval [0, 1).
		 * So (1-U1) is in the interval (0, 1] */
		R = sqrt(-2 * log(1-U1));
		theta = 2 * pi * U2;
	}
	
	change = !change;
	
	return R * (change ? sin(theta) : cos(theta));
}


void SortVec(VectorXd &vecSort, VectorXd &idx)
{
	int i, j;
	VectorXd vecRef;
	idx.resize(vecSort.rows());
	vecRef = vecSort;
	
	std::sort(vecSort.data(), vecSort.data()+ vecSort.rows());
	
	for(i=0; i<vecSort.rows(); i++)
	{
		for(j=0; j<vecSort.rows(); j++)
		{
			if( vecRef(i)==vecSort(j) )
				idx(j) = i;
		}
	}
}



