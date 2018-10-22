/*
 * Prints integer approximations of the tangent function and their respective
 * error.  Used for optimizing the constants in the xworld() function in
 * zone.c.
 * Copyright (C)2011 Mike Bourgeous.  Released under AGPLv3 in 2018.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc, char *argv[])
{
	double val = argc == 2 ? atof(argv[1]) : 28.0;
	double tanval = tan(val * M_PI / 180.0);
	double result;
	double rounded;
	int i;

	for(i = 0; i < 20; i++) {
		result = tanval * (1 << i);
		rounded = round(result);

		printf("tan(%.0f): %f\t<<%d: %f\ttrunc: %f\terr: %.3f%%\n",
				val,
				tanval,
				i,
				result,
				rounded,
				fabs(result - rounded) / result * 100.0);
	}
	
	return 0;
}
