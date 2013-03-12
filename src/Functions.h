#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "Struct.h"

VectorXd linspace(double d1, double d2, double n);

double Uniform01(void);

double quasi_normal_random(void);

void SortVec(VectorXd &vecSort, VectorXd &idx);

#endif /* FUNCTIONS_H_ */

