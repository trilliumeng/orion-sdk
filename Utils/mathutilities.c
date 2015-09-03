#include "mathutilities.h"


/*!
 * Add two angles together accounting for circular wrap
 * \param first is the first angle in radians in the range -PI to PI
 * \param second is the second angle in radians in the range -PI to PI
 * \return the correctly wrapped sum of first and second, in the range -PI to PI
 */
double addAngles(double first, double second)
{
    return wrapAngle(first + second);
}


/*!
 * Subtract one angle from another accounting for circular wrap
 * \param left is the first angle in radians in the range -PI to PI
 * \param serightcond is the second angle in radians in the range -PI to PI
 * \return the correctly wrapped difference of left minus right, in the range -PI to PI
 */
double subtractAngles(double left, double right)
{
    return wrapAngle(left - right);
}


/*!
 * Adjust an angle for circular wrap. The input angle will only be adjusted by
 * one circle (2PI). Arbitrary angles should be adjusted using fmod(angle, 2PI)
 * \param angle is the angle to adjust in radians, in the range of -3PI to 3PI.
 * \return an equivalent angle in the range of -PI to PI.
 */
double wrapAngle(double angle)
{
    if(angle > PId)
        angle -= 2*PId;
    else if(angle <= -PId)
        angle += 2*PId;

    return angle;
}


/*!
 * Adjust an angle for circular wrap with the wrap point at -270/+90. The input
 * angle will only be adjusted by one circle (2PI).
 * \param angle is the angle to adjust in radians, in the range of -3.5PI to 2.5PI.
 * \return an equivalent angle in the range of -1.5PI to 0.5PI.
 */
double wrapAngle90(double angle)
{
	if(angle > PId/2)
		angle -= 2*PId;
	else if(angle <= -3*PId/2)
		angle += 2*PId;

	return angle;
}


/*!
 * A simple first order low pass filter where state is stored by the caller.
 * Note that the filter time constant tau should be more than half sampleTime
 * or this function will amplify noise rather than filter it.
 * \param prev is the previous output of the filter
 * \param sig is the new signal
 * \param tau is the filter time constant, which should be more than half sampleTime
 * \param sampleTime is the iteration period of the filter, in the same units as tau
 * \return the new filtered value, which the caller must store
 */
double firstOrderFilter(double prev, double sig, double tau, double sampleTime)
{
    double alpha = (2*tau - sampleTime)/(2*tau + sampleTime);

    return alpha*prev + (1 - alpha)*sig;

}// firstOrderFilter


/*!
 * Add two angles together accounting for circular wrap
 * \param first is the first angle in radians in the range -PI to PI
 * \param second is the second angle in radians in the range -PI to PI
 * \return the correctly wrapped sum of first and second, in the range -PI to PI
 */
float addAnglesf(float first, float second)
{
    return wrapAnglef(first + second);
}


/*!
 * Subtract one angle from another accounting for circular wrap
 * \param left is the first angle in radians in the range -PI to PI
 * \param serightcond is the second angle in radians in the range -PI to PI
 * \return the correctly wrapped difference of left minus right, in the range -PI to PI
 */
float subtractAnglesf(float left, float right)
{
    return wrapAnglef(left - right);
}


/*!
 * Adjust an angle for circular wrap. The input angle will only be adjusted by
 * one circle (2PI). Arbitrary angles should be adjusted using fmod(angle, 2PI)
 * \param angle is the angle to adjust in radians, in the range of -3PI to 3PI.
 * \return an equivalent angle in the range of -PI to PI.
 */
float wrapAnglef(float angle)
{
    if(angle > PIf)
        angle -= 2*PIf;
    else if(angle <= -PIf)
        angle += 2*PIf;

    return angle;
}


/*!
 * Adjust an angle for circular wrap with the wrap point at -270/+90. The input
 * angle will only be adjusted by one circle (2PI).
 * \param angle is the angle to adjust in radians, in the range of -3.5PI to 2.5PI.
 * \return an equivalent angle in the range of -1.5PI to 0.5PI.
 */
float wrapAngle90f(float angle)
{
	if(angle > PIf/2)
		angle -= 2*PIf;
	else if(angle <= -3*PIf/2)
		angle += 2*PIf;

	return angle;
}


/*!
 * A simple first order low pass filter where state is stored by the caller.
 * Note that the filter time constant tau should be more than half sampleTime
 * or this function will amplify noise rather than filter it.
 * \param prev is the previous output of the filter
 * \param sig is the new signal
 * \param tau is the filter time constant, which should be more than half sampleTime
 * \param sampleTime is the iteration period of the filter, in the same units as tau
 * \return the new filtered value, which the caller must store
 */
float firstOrderFilterf(float prev, float sig, float tau, float sampleTime)
{
    float alpha = (2*tau - sampleTime)/(2*tau + sampleTime);

    return alpha*prev + (1 - alpha)*sig;

}// firstOrderFilterf
