#ifndef MATHUTILITIES_H
#define MATHUTILITIES_H

#include "Constants.h"

// C++ compilers: don't mangle us
#ifdef __cplusplus
extern "C" {
#endif

//! 1D interpolation
#define interpolate(ref, ind0, ind1, dep0, dep1) (((ref)-(ind0))*((dep1)-(dep0))/((ind1)-(ind0)) + (dep0))

//! Add two angles together accounting for circular wrap
double addAngles(double first, double second);

//! Subtract one angle from another, account for circular wrap
double subtractAngles(double left, double right);

//! Adjust an angle for circular wrap
double wrapAngle(double angle);

//! Adjust an angle for circular wrap with the wrap point at -270/+90
double wrapAngle90(double angle);

//! Apply a first order low pass filter
double firstOrderFilter(double prev, double sig, double tau, double sampleTime);

//! Add two angles together accounting for circular wrap
float addAnglesf(float first, float second);

//! Subtract one angle from another, account for circular wrap
float subtractAnglesf(float left, float right);

//! Adjust an angle for circular wrap
float wrapAnglef(float angle);

//! Adjust an angle for circular wrap with the wrap point at -270/+90
float wrapAngle90f(float angle);

//! Adjust an angle for circular wrap with the wrap point at 0/360
float wrapAngle360f(float angle);

//! Apply a first order low pass filter
float firstOrderFilterf(float prev, float sig, float tau, float sampleTime);

//! Apply a rate of change limit
float rateOfChangeLimitf(float prev, float value, float limit, float sampleTime);

// C++ compilers: don't mangle us
#ifdef __cplusplus
}
#endif

#endif // MATHUTILITIES_H
