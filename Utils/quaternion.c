#include "quaternion.h"
#include "mathutilities.h"
#include <math.h>

#ifndef SQRT_2f
#define SQRT_2f 1.4142135623730950488016887242097f
#endif

#ifndef SQRT_2
#define SQRT_2 1.4142135623730950488016887242097
#endif

/*!
 * Convert a quaternion to a DCM
 * \param quat is the quaternion to convert
 * \param dcm receives the direction cosine matrix
 */
void quaternionToDCM(const float quat[NQUATERNION], DCM_t* dcm)
{
    // squares of the quaternion elements
    float q0sq = SQR(quat[Q0]);
    float q1sq = SQR(quat[Q1]);
    float q2sq = SQR(quat[Q2]);
    float q3sq = SQR(quat[Q3]);

    // This form taken from Groves
    dcmSet(dcm, 0, 0, q0sq + q1sq - q2sq - q3sq);
    dcmSet(dcm, 0, 1, 2*(quat[Q1]*quat[Q2] - quat[Q3]*quat[Q0]));
    dcmSet(dcm, 0, 2, 2*(quat[Q1]*quat[Q3] + quat[Q2]*quat[Q0]));

    dcmSet(dcm, 1, 0, 2*(quat[Q1]*quat[Q2] + quat[Q3]*quat[Q0]));
    dcmSet(dcm, 1, 1, q0sq - q1sq + q2sq - q3sq);
    dcmSet(dcm, 1, 2, 2*(quat[Q2]*quat[Q3] - quat[Q1]*quat[Q0]));

    dcmSet(dcm, 2, 0, 2*(quat[Q1]*quat[Q3] - quat[Q2]*quat[Q0]));
    dcmSet(dcm, 2, 1, 2*(quat[Q2]*quat[Q3] + quat[Q1]*quat[Q0]));
    dcmSet(dcm, 2, 2, q0sq - q1sq - q2sq + q3sq);

}// quaternionToDCM


/*!
 * Convert a DCM to quaternion
 * \param dcm is the direction cosine matrix to convert
 * \param quat receives the quaternion
 */
void dcmToQuaternion(const DCM_t* dcm, float quat[NQUATERNION])
{
    const float* dcmdata = dcm->data;
    float mult;
    int imax, i;

    // The diagonal terms, quat is a temporary here. Notice the fabs. This
    // protects agains sqrt of negative. If the argument would have been
    // negative then the fabs changes the sign of all terms of the quat. This is
    // allowable as inverting all 4 elements does not change the rotation.
    //
    // This code is derived from:
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    quat[Q0] = fabsf(1 + dcmdata[T11] + dcmdata[T22] + dcmdata[T33]);
    quat[Q1] = fabsf(1 + dcmdata[T11] - dcmdata[T22] - dcmdata[T33]);
    quat[Q2] = fabsf(1 - dcmdata[T11] + dcmdata[T22] - dcmdata[T33]);
    quat[Q3] = fabsf(1 - dcmdata[T11] - dcmdata[T22] + dcmdata[T33]);

    // Find the largest one
    imax = 0;
    for(i = 1; i < 4; i++)
    {
        if(quat[i] > quat[imax])
            imax = i;
    }

    // The largest term
    quat[imax] = 0.5f*sqrtf(quat[imax]);

    // Multiplier on all the other elements. It is this division here that leads
    // to singularities if we don't select the largest quaternion element
    mult = 0.25f/quat[imax];

    // Compute the remaining terms from the off-diagonal elements. The equation
    // is a function of which term we computed from the diagonal element.
    switch(imax)
    {
    default:
    case Q0:
        quat[Q1] = mult*(dcmdata[T32] - dcmdata[T23]);
        quat[Q2] = mult*(dcmdata[T13] - dcmdata[T31]);
        quat[Q3] = mult*(dcmdata[T21] - dcmdata[T12]);
        break;
    case Q1:
        quat[Q0] = mult*(dcmdata[T32] - dcmdata[T23]);
        quat[Q2] = mult*(dcmdata[T21] + dcmdata[T12]);
        quat[Q3] = mult*(dcmdata[T13] + dcmdata[T31]);
        break;
    case Q2:
        quat[Q0] = mult*(dcmdata[T13] - dcmdata[T31]);
        quat[Q1] = mult*(dcmdata[T21] + dcmdata[T12]);
        quat[Q3] = mult*(dcmdata[T32] + dcmdata[T23]);
        break;
    case Q3:
        quat[Q0] = mult*(dcmdata[T21] - dcmdata[T12]);
        quat[Q1] = mult*(dcmdata[T13] + dcmdata[T31]);
        quat[Q2] = mult*(dcmdata[T32] + dcmdata[T23]);
        break;
    }

    // Finally, we would like the leading element to be positive
    if(quat[Q0] < 0.0f)
    {
        // Same rotation if all signs reversed.
        quat[Q0] *= -1;
        quat[Q1] *= -1;
        quat[Q2] *= -1;
        quat[Q3] *= -1;
    }

}// dcmToQuaternion


/*!
 * Fill out the quaternion for 0 roll, 0 pitch, 0 yaw
 * \param quat is initialized
 */
void initQuaternion(float quat[NQUATERNION])
{
    quat[Q0] = 1.0f;
    quat[Q1] = 0.0f;
    quat[Q2] = 0.0f;
    quat[Q3] = 0.0f;
}


/*!
 * Fill out the quaternion from an Euler roll angle
 * \param quat is filled out according to the angle
 * \param roll is the Euler roll angle in radians
 */
void setQuaternionBasedOnRoll(float quat[NQUATERNION], float roll)
{
    quat[Q0] = (SQRT_2f*0.5f)*sqrtf(1 + cosf(roll));
    quat[Q1] = 0.5f*sinf(roll)/quat[Q0];
    quat[Q2] = 0.0f;
    quat[Q3] = 0.0f;
}

/*!
 * Fill out the quaternion from an Euler pitch angle
 * \param quat is filled out according to the angle
 * \param pitch is the Euler Pitch angle in radians
 */
void setQuaternionBasedOnPitch(float quat[NQUATERNION], float pitch)
{
    quat[Q0] = (SQRT_2f*0.5f)*sqrtf(1 + cosf(pitch));
    quat[Q1] = 0.0f;
    quat[Q2] = 0.5f*sinf(pitch)/quat[Q0];
    quat[Q3] = 0.0f;
}

/*!
 * Fill out the quaternion from an Euler yaw angle
 * \param quat is filled out according to the angle
 * \param yaw is the Euler Yaw angle in radians
 */
void setQuaternionBasedOnYaw(float quat[NQUATERNION], float yaw)
{
    quat[Q0] = (SQRT_2f*0.5f)*sqrtf(1 + cosf(yaw));
    quat[Q1] = 0.0f;
    quat[Q2] = 0.0f;
    quat[Q3] = 0.5f*sinf(yaw)/quat[Q0];
}


/*!
 * Fill out the quaternion from an Euler, yaw, then pitch, then roll rotation
 * \param quat is filled out according to the angles.
 * \param yaw is the Euler yaw angle in radians
 * \param pitch is the Euler pitch angle in radians
 * \param roll is the Euler roll angle in radians
 */
void setQuaternionBasedOnEuler(float quat[NQUATERNION], float yaw, float pitch, float roll)
{
    stackAllocateDCM(dcm);

    setDCMBasedOnEuler(&dcm, yaw, pitch, roll);

    dcmToQuaternion(&dcm, quat);
}


/*!
 * Compute the Euler yaw angle of a quaternion.
 * \param quat is the body to reference quaternion.
 * \return the Euler yaw angle in radians, from -PI to PI.
 */
float quaternionYaw(const float quat[NQUATERNION])
{
    return atan2f(2*(quat[Q1]*quat[Q2] + quat[Q3]*quat[Q0]), SQR(quat[Q0]) + SQR(quat[Q1]) - SQR(quat[Q2]) - SQR(quat[Q3]));

}// quaternionYaw


/*!
 * Compute the Euler pitch rotation of a quaternion.
 * \param quat is the body to reference quaternion.
 * \return the Euler pitch angle in radians, from -PI/2 to PI/2.
 */
float quaternionPitch(const float quat[NQUATERNION])
{
    return asinf(quaternionSinPitch(quat));

}// quaternionPitch


/*!
 * Compute the cosine of the Euler pitch angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the cosine of the Euler pitch angle.
 */
float quaternionCosPitch(const float quat[NQUATERNION])
{
    float sinp = quaternionSinPitch(quat);

    // pitch goes from -90 to 90, so cosine of pitch must be positive.
    return sqrtf(1.0f - sinp*sinp);

}// quaternionCosPitch


/*!
 * Compute the sin of the Euler pitch angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the sin of the Euler pitch angle.
 */
float quaternionSinPitch(const float quat[NQUATERNION])
{
    return SATURATE(-2*(quat[Q1]*quat[Q3] - quat[Q2]*quat[Q0]), 1.0f);
}


/*!
 * Compute the Euler roll rotation.
 * \param quat is the body to reference quaternion.
 * \return the Euler roll angle in radians, from -PI to PI.
 */
float quaternionRoll(const float quat[NQUATERNION])
{
    return atan2f(2*(quat[Q2]*quat[Q3] + quat[Q1]*quat[Q0]), SQR(quat[Q0]) - SQR(quat[Q1]) - SQR(quat[Q2]) + SQR(quat[Q3]));

}// roll


/*!
 * Compute the cosine of the Euler roll angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the cosine of the Euler roll angle.
 */
float quaternionCosRoll(const float quat[NQUATERNION])
{
    return cosf(quaternionRoll(quat));
}


/*!
 * Compute the sin of the Euler roll angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the sin of the Euler roll angle.
 */
float quaternionSinRoll(const float quat[NQUATERNION])
{
    return sinf(quaternionRoll(quat));
}


/*!
 * Use a quaternion to rotate a vector. The input and output
 * vector can be the same vector
 * \param quat is the quaternion.
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void quaternionApplyRotation(const float quat[NQUATERNION], const float input[], float output[])
{
    float zero, one, two;

    stackAllocateDCM(rotation);

    quaternionToDCM(quat, &rotation);

    // Doing it this way makes it possible for input and output to be the same vector
    zero = rotationdata[0]*input[0] + rotationdata[1]*input[1] + rotationdata[2]*input[2];
    one  = rotationdata[3]*input[0] + rotationdata[4]*input[1] + rotationdata[5]*input[2];
    two  = rotationdata[6]*input[0] + rotationdata[7]*input[1] + rotationdata[8]*input[2];
    output[0] = zero;
    output[1] = one;
    output[2] = two;

}// quaternionApplyRotation


/*!
 * Use a quaternion to rotate a vector, in the reverse direction. The input and output
 * vector can be the same vector.
 * \param quat is the quaternion.
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void quaternionApplyReverseRotation(const float quat[NQUATERNION], const float input[], float output[])
{
    float zero, one, two;

    stackAllocateDCM(rotation);

    quaternionToDCM(quat, &rotation);

    // Doing it this way makes it possible for input and output to be the same vector
    zero = rotationdata[0]*input[0] + rotationdata[3]*input[1] + rotationdata[6]*input[2];
    one  = rotationdata[1]*input[0] + rotationdata[4]*input[1] + rotationdata[7]*input[2];
    two  = rotationdata[2]*input[0] + rotationdata[5]*input[1] + rotationdata[8]*input[2];
    output[0] = zero;
    output[1] = one;
    output[2] = two;

}// quaternionApplyReverseRotation


/*!
 * Compute the length of a quaternion which should be 1.0
 * \param quat is the quaternion whose length is computed
 * \return the length of the quaternion
 */
float quaternionLength(const float quat[NQUATERNION])
{
    return sqrtf(SQR(quat[Q0]) + SQR(quat[Q1]) + SQR(quat[Q2]) + SQR(quat[Q3]));
}


/*!
 * Multiply two quaternions together such that r = p*q. Quaternions are stored
 * in the form [w,x,y,z].
 * \param p is the left operand quaternion.
 * \param q is the right operand quaternion.
 * \param r receives the multiplication of p*q.
 *        r must not share memory with p or q.
 * \return a pointer to r.
 */
float* quaternionMultiply( const float p[NQUATERNION], const float q[NQUATERNION], float r[NQUATERNION])
{
    r[Q0] = p[Q0]*q[Q0] - p[Q1]*q[Q1] - p[Q2]*q[Q2] - p[Q3]*q[Q3];
    r[Q1] = p[Q0]*q[Q1] + p[Q1]*q[Q0] + p[Q2]*q[Q3] - p[Q3]*q[Q2];
    r[Q2] = p[Q0]*q[Q2] - p[Q1]*q[Q3] + p[Q2]*q[Q0] + p[Q3]*q[Q1];
    r[Q3] = p[Q0]*q[Q3] + p[Q1]*q[Q2] - p[Q2]*q[Q1] + p[Q3]*q[Q0];

    return r;
}


/*!
 * Multiply two quaternions together such that r = p^-1*q. Quaternions are stored
 * in the form [w,x,y,z].
 * \param p is the left operand quaternion.
 * \param q is the right operand quaternion.
 * \param r receives the multiplication of p*q.
 *        r must not share memory with p or q.
 * \return a pointer to r.
 */
float* quaternionMultiplyInverseA( const float p[NQUATERNION], const float q[NQUATERNION], float r[NQUATERNION])
{
    r[Q0] = p[Q0]*q[Q0] + p[Q1]*q[Q1] + p[Q2]*q[Q2] + p[Q3]*q[Q3];
    r[Q1] = p[Q0]*q[Q1] - p[Q1]*q[Q0] - p[Q2]*q[Q3] + p[Q3]*q[Q2];
    r[Q2] = p[Q0]*q[Q2] + p[Q1]*q[Q3] - p[Q2]*q[Q0] - p[Q3]*q[Q1];
    r[Q3] = p[Q0]*q[Q3] - p[Q1]*q[Q2] + p[Q2]*q[Q1] - p[Q3]*q[Q0];

    return r;
}


/*!
 * Multiply two quaternions together such that r = p*q^-1. Quaternions are stored
 * in the form [w,x,y,z].
 * \param p is the left operand quaternion.
 * \param q is the right operand quaternion.
 * \param r receives the multiplication of p*q.
 *        r must not share memory with p or q.
 * \return a pointer to r.
 */
float* quaternionMultiplyInverseB( const float p[NQUATERNION], const float q[NQUATERNION], float r[NQUATERNION])
{
    r[Q0] =  p[Q0]*q[Q0] + p[Q1]*q[Q1] + p[Q2]*q[Q2] + p[Q3]*q[Q3];
    r[Q1] = -p[Q0]*q[Q1] + p[Q1]*q[Q0] - p[Q2]*q[Q3] + p[Q3]*q[Q2];
    r[Q2] = -p[Q0]*q[Q2] + p[Q1]*q[Q3] + p[Q2]*q[Q0] - p[Q3]*q[Q1];
    r[Q3] = -p[Q0]*q[Q3] - p[Q1]*q[Q2] + p[Q2]*q[Q1] + p[Q3]*q[Q0];

    return r;
}


/*!
 * Convert a quaternion to a rotation vector
 * \param quat is the quaternion to convert
 * \param rotVec receives the rotation vector
 * \return a pointer to rotVec
 */
float* quaternionToRotVec( const float quat[NQUATERNION],  float rotVec[NVECTOR3])
{
    if( fabsf(quat[Q0]) < 1.0f )
    {
        float scale = 2.0f*acosf(quat[Q0])/sqrtf( 1.0f-SQR(quat[Q0]) );
        rotVec[0] = scale*quat[Q1];
        rotVec[1] = scale*quat[Q2];
        rotVec[2] = scale*quat[Q3];
    }
    else
    {
        rotVec[0] = 0;
        rotVec[1] = 0;
        rotVec[2] = 0;
    }

    return rotVec;
}


/*!
 * Convert a rotation vector to a quaternion
 * \param rotVec is the rotation vector to convert
 * \param quat receies the quaternion
 * \return a poitner to quat
 */
float* rotVecToQuaternion( const float rotVec[NVECTOR3], float quat[NQUATERNION] )
{
    // get the magnitude of the rotation vector
    float magRot = vector3Lengthf(rotVec);

    if( magRot > 0.0f )
    {
        float magRot_2 = magRot/2.0f;
        float sinTerm = sinf(magRot_2)/magRot;
        quat[Q0] = cosf( magRot_2 );
        quat[Q1] = rotVec[0] * sinTerm;
        quat[Q2] = rotVec[1] * sinTerm;
        quat[Q3] = rotVec[2] * sinTerm;
    }
    else
    {
        quat[Q0] = 1.0f;
        quat[Q1] = 0.0f;
        quat[Q2] = 0.0f;
        quat[Q3] = 0.0f;
    }

    return quat;
}


/*!
 * Test quaternion operations
 * \return TRUE if test passed
 */
BOOL testQuaternion(void)
{
    float error = 0.0f;
    float quat[NQUATERNION];
    stackAllocateDCM(dcm);

    matrixSetIdentityf(&dcm);
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += testForIdentityf(&dcm);

    setDCMBasedOnYaw(&dcm, deg2radf(13.5f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(13.5f) - quaternionYaw(quat));

    setDCMBasedOnPitch(&dcm, deg2radf(-27.5f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(-27.5f) - quaternionPitch(quat));

    setDCMBasedOnRoll(&dcm, deg2radf(160.0f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(160.0f) - quaternionRoll(quat));

    setDCMBasedOnYaw(&dcm, deg2radf(160.0f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(160.0f) - quaternionYaw(quat));

    setDCMBasedOnPitch(&dcm, deg2radf(85.5f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(85.5f) - quaternionPitch(quat));

    setQuaternionBasedOnYaw(quat, deg2radf(13.5f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(13.5f) - dcmYaw(&dcm));

    setQuaternionBasedOnPitch(quat, deg2radf(-27.5f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-27.5f) - dcmPitch(&dcm));

    setQuaternionBasedOnRoll(quat, deg2radf(-160.0f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-160.0f) - dcmRoll(&dcm));

    setQuaternionBasedOnYaw(quat, deg2radf(-160.0f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-160.0f) - dcmYaw(&dcm));

    setQuaternionBasedOnPitch(quat, deg2radf(-85.5f));
    error += fabsf(1.0f - quaternionLength(quat));
    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-85.5f) - dcmPitch(&dcm));

    setDCMBasedOnEuler(&dcm, deg2radf(-13.5f), deg2radf(27.5f), deg2radf(-160.0f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(-13.5f) - quaternionYaw(quat));
    error += fabsf(deg2radf(27.5f) - quaternionPitch(quat));
    error += fabsf(deg2radf(-160.0f) - quaternionRoll(quat));

    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-13.5f) - dcmYaw(&dcm));
    error += fabsf(deg2radf(27.5f) - dcmPitch(&dcm));
    error += fabsf(deg2radf(-160.0f) - dcmRoll(&dcm));

    setDCMBasedOnEuler(&dcm, deg2radf(-85.5f), deg2radf(75.0f), deg2radf(-160.0f));
    dcmToQuaternion(&dcm, quat);
    error += fabsf(1.0f - quaternionLength(quat));
    error += fabsf(deg2radf(-85.5f) - quaternionYaw(quat));
    error += fabsf(deg2radf(75.0f) - quaternionPitch(quat));
    error += fabsf(deg2radf(-160.0f) - quaternionRoll(quat));

    quaternionToDCM(quat, &dcm);
    error += fabsf(deg2radf(-85.5f) - dcmYaw(&dcm));
    error += fabsf(deg2radf(75.0f) - dcmPitch(&dcm));
    error += fabsf(deg2radf(-160.0f) - dcmRoll(&dcm));

    if(error < 0.001f)
        return TRUE;
    else
        return FALSE;

}// testQuaternion


//////////////////////////////////////////////////
// Functions for double precision quaternion below


/*!
 * Convert a quaternion to a DCM
 * \param quat is the quaternion to convert
 * \param dcm receives the direction cosine matrix
 */
void quaterniondToDCM(const double quat[NQUATERNION], DCMd_t* dcm)
{
    // squares of the quaternion elements
    double q0sq = SQR(quat[Q0]);
    double q1sq = SQR(quat[Q1]);
    double q2sq = SQR(quat[Q2]);
    double q3sq = SQR(quat[Q3]);

    // This form taken from Groves
    dcmdSet(dcm, 0, 0, q0sq + q1sq - q2sq - q3sq);
    dcmdSet(dcm, 0, 1, 2*(quat[Q1]*quat[Q2] - quat[Q3]*quat[Q0]));
    dcmdSet(dcm, 0, 2, 2*(quat[Q1]*quat[Q3] + quat[Q2]*quat[Q0]));

    dcmdSet(dcm, 1, 0, 2*(quat[Q1]*quat[Q2] + quat[Q3]*quat[Q0]));
    dcmdSet(dcm, 1, 1, q0sq - q1sq + q2sq - q3sq);
    dcmdSet(dcm, 1, 2, 2*(quat[Q2]*quat[Q3] - quat[Q1]*quat[Q0]));

    dcmdSet(dcm, 2, 0, 2*(quat[Q1]*quat[Q3] - quat[Q2]*quat[Q0]));
    dcmdSet(dcm, 2, 1, 2*(quat[Q2]*quat[Q3] + quat[Q1]*quat[Q0]));
    dcmdSet(dcm, 2, 2, q0sq - q1sq - q2sq + q3sq);

}// quaterniondToDCM


/*!
 * Convert a quaternion to a DCM (float)
 * \param quat is the quaternion to convert
 * \param dcm receives the direction cosine matrix (float)
 */
void quaterniondToDCMf(const double quat[NQUATERNION], DCM_t* dcm)
{
    float quatf[NQUATERNION] = {(float)quat[Q0], (float)quat[Q1], (float)quat[Q2], (float)quat[Q3]};

    quaternionToDCM(quatf, dcm);

}// quaterniondToDCMf


/*!
 * Convert a DCM to quaternion
 * \param dcm is the direction cosine matrix to convert
 * \param quat receives the quaternion
 */
void dcmToQuaterniond(const DCMd_t* dcm, double quat[NQUATERNION])
{
    const double* dcmdata = dcm->data;
    double mult;
    int imax, i;

    // The diagonal terms, quat is a temporary here. Notice the fabs. This
    // protects agains sqrt of negative. If the argument would have been
    // negative then the fabs changes the sign of all terms of the quat. This is
    // allowable as inverting all 4 elements does not change the rotation.
    //
    // This code is derived from:
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    quat[Q0] = fabs(1 + dcmdata[T11] + dcmdata[T22] + dcmdata[T33]);
    quat[Q1] = fabs(1 + dcmdata[T11] - dcmdata[T22] - dcmdata[T33]);
    quat[Q2] = fabs(1 - dcmdata[T11] + dcmdata[T22] - dcmdata[T33]);
    quat[Q3] = fabs(1 - dcmdata[T11] - dcmdata[T22] + dcmdata[T33]);

    // Find the largest one
    imax = 0;
    for(i = 1; i < 4; i++)
    {
        if(quat[i] > quat[imax])
            imax = i;
    }

    // The largest term
    quat[imax] = 0.5*sqrt(quat[imax]);

    // Multiplier on all the other elements. It is this division here that leads
    // to singularities if we don't select the largest quaternion element
    mult = 0.25f/quat[imax];

    // Compute the remaining terms from the off-diagonal elements. The equation
    // is a function of which term we computed from the diagonal element.
    switch(imax)
    {
    default:
    case Q0:
        quat[Q1] = mult*(dcmdata[T32] - dcmdata[T23]);
        quat[Q2] = mult*(dcmdata[T13] - dcmdata[T31]);
        quat[Q3] = mult*(dcmdata[T21] - dcmdata[T12]);
        break;
    case Q1:
        quat[Q0] = mult*(dcmdata[T32] - dcmdata[T23]);
        quat[Q2] = mult*(dcmdata[T21] + dcmdata[T12]);
        quat[Q3] = mult*(dcmdata[T13] + dcmdata[T31]);
        break;
    case Q2:
        quat[Q0] = mult*(dcmdata[T13] - dcmdata[T31]);
        quat[Q1] = mult*(dcmdata[T21] + dcmdata[T12]);
        quat[Q3] = mult*(dcmdata[T32] + dcmdata[T23]);
        break;
    case Q3:
        quat[Q0] = mult*(dcmdata[T21] - dcmdata[T12]);
        quat[Q1] = mult*(dcmdata[T13] + dcmdata[T31]);
        quat[Q2] = mult*(dcmdata[T32] + dcmdata[T23]);
        break;
    }

    // Finally, we would like the leading element to be positive
    if(quat[Q0] < 0.0)
    {
        // Same rotation if all signs reversed.
        quat[Q0] *= -1;
        quat[Q1] *= -1;
        quat[Q2] *= -1;
        quat[Q3] *= -1;
    }

}// dcmToQuaterniond


/*!
 * Convert a DCM (float) to quaternion (double)
 * \param dcm is the direction cosine matrix to convert
 * \param quat receives the quaternion
 */
void dcmfToQuaterniond(const DCM_t* dcm, double quat[NQUATERNION])
{
    float quatf[NQUATERNION];
    dcmToQuaternion(dcm, quatf);
    quat[Q0] = quatf[Q0];
    quat[Q1] = quatf[Q1];
    quat[Q2] = quatf[Q2];
    quat[Q3] = quatf[Q3];
}


/*!
 * Fill out the quaternion for 0 roll, 0 pitch, 0 yaw
 * \param quat is initialized
 */
void initQuaterniond(double quat[NQUATERNION])
{
    quat[Q0] = 1.0;
    quat[Q1] = 0.0;
    quat[Q2] = 0.0;
    quat[Q3] = 0.0;
}


/*!
 * Fill out the quaternion from an Euler roll angle
 * \param quat is filled out according to the angle
 * \param roll is the Euler roll angle in radians
 */
void setQuaterniondBasedOnRoll(double quat[NQUATERNION], double roll)
{
    quat[Q0] = (SQRT_2*0.5)*sqrt(1 + cos(roll));
    quat[Q1] = 0.5*sin(roll)/quat[Q0];
    quat[Q2] = 0.0;
    quat[Q3] = 0.0;
}

/*!
 * Fill out the quaternion from an Euler pitch angle
 * \param quat is filled out according to the angle
 * \param pitch is the Euler Pitch angle in radians
 */
void setQuaterniondBasedOnPitch(double quat[NQUATERNION], double pitch)
{
    quat[Q0] = (SQRT_2*0.5)*sqrt(1 + cos(pitch));
    quat[Q1] = 0.0;
    quat[Q2] = 0.5*sin(pitch)/quat[Q0];
    quat[Q3] = 0.0;
}

/*!
 * Fill out the quaternion from an Euler yaw angle
 * \param quat is filled out according to the angle
 * \param yaw is the Euler Yaw angle in radians
 */
void setQuaterniondBasedOnYaw(double quat[NQUATERNION], double yaw)
{
    quat[Q0] = (SQRT_2*0.5)*sqrt(1 + cos(yaw));
    quat[Q1] = 0.0;
    quat[Q2] = 0.0;
    quat[Q3] = 0.5*sin(yaw)/quat[Q0];
}


/*!
 * Fill out the quaternion from an Euler, yaw, then pitch, then roll rotation
 * \param quat is filled out according to the angles.
 * \param yaw is the Euler yaw angle in radians
 * \param pitch is the Euler pitch angle in radians
 * \param roll is the Euler roll angle in radians
 */
void setQuaterniondBasedOnEuler(double quat[NQUATERNION], double yaw, double pitch, double roll)
{
    stackAllocateDCMd(dcm);

    setDCMdBasedOnEuler(&dcm, yaw, pitch, roll);

    dcmToQuaterniond(&dcm, quat);
}


/*!
 * Compute the Euler yaw angle of a quaternion.
 * \param quat is the body to reference quaternion.
 * \return the Euler yaw angle in radians, from -PI to PI.
 */
double quaterniondYaw(const double quat[NQUATERNION])
{
    return atan2(2*(quat[Q1]*quat[Q2] + quat[Q3]*quat[Q0]), SQR(quat[Q0]) + SQR(quat[Q1]) - SQR(quat[Q2]) - SQR(quat[Q3]));

}// quaternionYaw


/*!
 * Compute the Euler pitch rotation of a quaternion.
 * \param quat is the body to reference quaternion.
 * \return the Euler pitch angle in radians, from -PI/2 to PI/2.
 */
double quaterniondPitch(const double quat[NQUATERNION])
{
    return asin(quaterniondSinPitch(quat));

}// quaternionPitch


/*!
 * Compute the cosine of the Euler pitch angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the cosine of the Euler pitch angle.
 */
double quaterniondCosPitch(const double quat[NQUATERNION])
{
    double sinp = quaterniondSinPitch(quat);

    // pitch goes from -90 to 90, so cosine of pitch must be positive.
    return sqrt(1.0 - sinp*sinp);

}// quaterniondCosPitch


/*!
 * Compute the sin of the Euler pitch angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the sin of the Euler pitch angle.
 */
double quaterniondSinPitch(const double quat[NQUATERNION])
{
    return SATURATE(-2*(quat[Q1]*quat[Q3] - quat[Q2]*quat[Q0]), 1.0);
}


/*!
 * Compute the Euler roll rotation.
 * \param quat is the body to reference quaternion.
 * \return the Euler roll angle in radians, from -PI to PI.
 */
double quaterniondRoll(const double quat[NQUATERNION])
{
    return atan2(2*(quat[Q2]*quat[Q3] + quat[Q1]*quat[Q0]), SQR(quat[Q0]) - SQR(quat[Q1]) - SQR(quat[Q2]) + SQR(quat[Q3]));

}// roll


/*!
 * Compute the cosine of the Euler roll angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the cosine of the Euler roll angle.
 */
double quaterniondCosRoll(const double quat[NQUATERNION])
{
    return cos(quaterniondRoll(quat));
}


/*!
 * Compute the sin of the Euler roll angle of a quaternion
 * \param quat is the body to reference quaternion.
 * \return the sin of the Euler roll angle.
 */
double quaterniondSinRoll(const double quat[NQUATERNION])
{
    return sin(quaterniondRoll(quat));
}


/*!
 * Use a quaternion to rotate a vector. The input and output
 * vector can be the same vector
 * \param quat is the quaternion.
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void quaterniondApplyRotation(const double quat[NQUATERNION], const double input[], double output[])
{
    double zero, one, two;

    stackAllocateDCMd(rotation);

    quaterniondToDCM(quat, &rotation);

    // Doing it this way makes it possible for input and output to be the same vector
    zero = rotationdata[0]*input[0] + rotationdata[1]*input[1] + rotationdata[2]*input[2];
    one  = rotationdata[3]*input[0] + rotationdata[4]*input[1] + rotationdata[5]*input[2];
    two  = rotationdata[6]*input[0] + rotationdata[7]*input[1] + rotationdata[8]*input[2];
    output[0] = zero;
    output[1] = one;
    output[2] = two;

}// quaterniondApplyRotation


/*!
 * Use a quaternion to rotate a vector, in the reverse direction. The input and output
 * vector can be the same vector.
 * \param quat is the quaternion.
 * \param input is the 3 element vector to be rotated.
 * \param output is the 3 element vector that receives the rotated result.
 */
void quaterniondApplyReverseRotation(const double quat[NQUATERNION], const double input[], double output[])
{
    double zero, one, two;

    stackAllocateDCMd(rotation);

    quaterniondToDCM(quat, &rotation);

    // Doing it this way makes it possible for input and output to be the same vector
    zero = rotationdata[0]*input[0] + rotationdata[3]*input[1] + rotationdata[6]*input[2];
    one  = rotationdata[1]*input[0] + rotationdata[4]*input[1] + rotationdata[7]*input[2];
    two  = rotationdata[2]*input[0] + rotationdata[5]*input[1] + rotationdata[8]*input[2];
    output[0] = zero;
    output[1] = one;
    output[2] = two;

}// quaterniondApplyReverseRotation


/*!
 * Compute the length of a quaternion which should be 1.0
 * \param quat is the quaternion whose length is computed
 * \return the length of the quaternion
 */
double quaterniondLength(const double quat[NQUATERNION])
{
    return sqrt(SQR(quat[Q0]) + SQR(quat[Q1]) + SQR(quat[Q2]) + SQR(quat[Q3]));
}


/*!
 * Multiply two quaternions together such that r = p*q. Quaternions are stored
 * in the form [w,x,y,z].
 * \param p is the left operand quaternion.
 * \param q is the right operand quaternion.
 * \param r receives the multiplication of p*q.
 *        r must not share memory with p or q.
 * \return a pointer to r.
 */
double* quaterniondMultiply( const double p[NQUATERNION], const double q[NQUATERNION], double r[NQUATERNION])
{
    r[Q0] = p[Q0]*q[Q0] - p[Q1]*q[Q1] - p[Q2]*q[Q2] - p[Q3]*q[Q3];
    r[Q1] = p[Q0]*q[Q1] + p[Q1]*q[Q0] + p[Q2]*q[Q3] - p[Q3]*q[Q2];
    r[Q2] = p[Q0]*q[Q2] - p[Q1]*q[Q3] + p[Q2]*q[Q0] + p[Q3]*q[Q1];
    r[Q3] = p[Q0]*q[Q3] + p[Q1]*q[Q2] - p[Q2]*q[Q1] + p[Q3]*q[Q0];

    return r;
}

/*!
 * Multiply two quaternions together such that r = p^-1*q. Quaternions are stored
 * in the form [w,x,y,z].
 * \param p is the left operand quaternion.
 * \param q is the right operand quaternion.
 * \param r receives the multiplication of p*q.
 *        r must not share memory with p or q.
 * \return a pointer to r.
 */
double* quaterniondMultiplyInverseA( const double p[NQUATERNION], const double q[NQUATERNION], double r[NQUATERNION])
{
    r[Q0] = p[Q0]*q[Q0] + p[Q1]*q[Q1] + p[Q2]*q[Q2] + p[Q3]*q[Q3];
    r[Q1] = p[Q0]*q[Q1] - p[Q1]*q[Q0] - p[Q2]*q[Q3] + p[Q3]*q[Q2];
    r[Q2] = p[Q0]*q[Q2] + p[Q1]*q[Q3] - p[Q2]*q[Q0] - p[Q3]*q[Q1];
    r[Q3] = p[Q0]*q[Q3] - p[Q1]*q[Q2] + p[Q2]*q[Q1] - p[Q3]*q[Q0];
    return r;
}

/*!
 * Multiply two quaternions together such that r = p*q^-1. Quaternions are stored
 * in the form [w,x,y,z].
 * \param p is the left operand quaternion.
 * \param q is the right operand quaternion.
 * \param r receives the multiplication of p*q.
 *        r must not share memory with p or q.
 * \return a pointer to r.
 */
double* quaterniondMultiplyInverseB( const double p[NQUATERNION], const double q[NQUATERNION], double r[NQUATERNION])
{
    r[Q0] =  p[Q0]*q[Q0] + p[Q1]*q[Q1] + p[Q2]*q[Q2] + p[Q3]*q[Q3];
    r[Q1] = -p[Q0]*q[Q1] + p[Q1]*q[Q0] - p[Q2]*q[Q3] + p[Q3]*q[Q2];
    r[Q2] = -p[Q0]*q[Q2] + p[Q1]*q[Q3] + p[Q2]*q[Q0] - p[Q3]*q[Q1];
    r[Q3] = -p[Q0]*q[Q3] - p[Q1]*q[Q2] + p[Q2]*q[Q1] + p[Q3]*q[Q0];

    return r;
}


/*!
 * Convert a quaternion to a rotation vector
 * \param quat is the quaternion to convert
 * \param rotVec receives the rotation vector
 * \return a pointer to rotVec
 */
double* quaterniondToRotVec( const double quat[NQUATERNION],  double rotVec[NVECTOR3])
{
    if( fabs(quat[Q0]) < 1.0 )
    {
        double scale = 2.0*acos(quat[Q0])/sqrt( 1.0-SQR(quat[Q0]) );
        rotVec[0] = scale*quat[Q1];
        rotVec[1] = scale*quat[Q2];
        rotVec[2] = scale*quat[Q3];
    }
    else
    {
        rotVec[0] = 0;
        rotVec[1] = 0;
        rotVec[2] = 0;
    }

    return rotVec;
}


/*!
 * Convert a rotation vector to a quaternion
 * \param rotVec is the rotation vector to convert
 * \param quat receies the quaternion
 * \return a poitner to quat
 */
double* rotVecToQuaterniond( const double rotVec[NVECTOR3], double quat[NQUATERNION] )
{
    // get the magnitude of the rotation vector
    double magRot = vector3Length(rotVec);

    if( magRot > 0.0 )
    {
        double magRot_2 = magRot/2.0;
        double sinTerm = sin(magRot_2)/magRot;
        quat[Q0] = cos( magRot_2 );
        quat[Q1] = rotVec[0] * sinTerm;
        quat[Q2] = rotVec[1] * sinTerm;
        quat[Q3] = rotVec[2] * sinTerm;
    }
    else
    {
        quat[Q0] = 1.0;
        quat[Q1] = 0.0;
        quat[Q2] = 0.0;
        quat[Q3] = 0.0;
    }

    return quat;
}


/*!
 * Test quaternion operations
 * \return TRUE if test passed
 */
BOOL testQuaterniond(void)
{
    double error = 0.0;
    double quat[NQUATERNION];
    stackAllocateDCMd(dcm);

    matrixSetIdentity(&dcm);
    dcmToQuaterniond(&dcm, quat);
    error += fabs(1.0 - quaterniondLength(quat));
    quaterniondToDCM(quat, &dcm);
    error += testForIdentity(&dcm);

    setDCMdBasedOnYaw(&dcm, deg2rad(13.5));
    dcmToQuaterniond(&dcm, quat);
    error += fabs(1.0 - quaterniondLength(quat));
    error += fabs(deg2rad(13.5) - quaterniondYaw(quat));

    setDCMdBasedOnPitch(&dcm, deg2rad(-27.5));
    dcmToQuaterniond(&dcm, quat);
    error += fabs(1.0 - quaterniondLength(quat));
    error += fabs(deg2rad(-27.5) - quaterniondPitch(quat));

    setDCMdBasedOnRoll(&dcm, deg2rad(160.0));
    dcmToQuaterniond(&dcm, quat);
    error += fabs(1.0 - quaterniondLength(quat));
    error += fabs(deg2rad(160.0) - quaterniondRoll(quat));

    setDCMdBasedOnYaw(&dcm, deg2rad(160.0));
    dcmToQuaterniond(&dcm, quat);
    error += fabs(1.0 - quaterniondLength(quat));
    error += fabs(deg2rad(160.0) - quaterniondYaw(quat));

    setDCMdBasedOnPitch(&dcm, deg2rad(85.5));
    dcmToQuaterniond(&dcm, quat);
    error += fabs(1.0 - quaterniondLength(quat));
    error += fabs(deg2rad(85.5) - quaterniondPitch(quat));

    setQuaterniondBasedOnYaw(quat, deg2rad(13.5));
    error += fabs(1.0 - quaterniondLength(quat));
    quaterniondToDCM(quat, &dcm);
    error += fabs(deg2rad(13.5) - dcmdYaw(&dcm));

    setQuaterniondBasedOnPitch(quat, deg2rad(-27.5));
    error += fabs(1.0 - quaterniondLength(quat));
    quaterniondToDCM(quat, &dcm);
    error += fabs(deg2rad(-27.5) - dcmdPitch(&dcm));

    setQuaterniondBasedOnRoll(quat, deg2rad(-160.0));
    error += fabs(1.0 - quaterniondLength(quat));
    quaterniondToDCM(quat, &dcm);
    error += fabs(deg2rad(-160.0) - dcmdRoll(&dcm));

    setQuaterniondBasedOnYaw(quat, deg2rad(-160.0));
    error += fabs(1.0 - quaterniondLength(quat));
    quaterniondToDCM(quat, &dcm);
    error += fabs(deg2rad(-160.0) - dcmdYaw(&dcm));

    setQuaterniondBasedOnPitch(quat, deg2rad(-85.5));
    error += fabs(1.0 - quaterniondLength(quat));
    quaterniondToDCM(quat, &dcm);
    error += fabs(deg2rad(-85.5) - dcmdPitch(&dcm));

    setDCMdBasedOnEuler(&dcm, deg2rad(-13.5), deg2rad(27.5), deg2rad(-160.0));
    dcmToQuaterniond(&dcm, quat);
    error += fabs(1.0 - quaterniondLength(quat));
    error += fabs(deg2rad(-13.5) - quaterniondYaw(quat));
    error += fabs(deg2rad(27.5) - quaterniondPitch(quat));
    error += fabs(deg2rad(-160.0) - quaterniondRoll(quat));

    quaterniondToDCM(quat, &dcm);
    error += fabs(deg2rad(-13.5) - dcmdYaw(&dcm));
    error += fabs(deg2rad(27.5) - dcmdPitch(&dcm));
    error += fabs(deg2rad(-160.0) - dcmdRoll(&dcm));

    setDCMdBasedOnEuler(&dcm, deg2rad(-85.5), deg2rad(75.0), deg2rad(-160.0));
    dcmToQuaterniond(&dcm, quat);
    error += fabs(1.0 - quaterniondLength(quat));
    error += fabs(deg2rad(-85.5) - quaterniondYaw(quat));
    error += fabs(deg2rad(75.0) - quaterniondPitch(quat));
    error += fabs(deg2rad(-160.0) - quaterniondRoll(quat));

    quaterniondToDCM(quat, &dcm);
    error += fabs(deg2rad(-85.5) - dcmdYaw(&dcm));
    error += fabs(deg2rad(75.0) - dcmdPitch(&dcm));
    error += fabs(deg2rad(-160.0) - dcmdRoll(&dcm));

    if(error < 0.001)
        return TRUE;
    else
        return FALSE;
}

