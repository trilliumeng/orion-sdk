#include "GpsDataReceive.h"
#include "OrionPublicPacket.h"
#include "earthposition.h"
#include "earthrotation.h"
#include <math.h>

/*!
 * Fill out null-encoded ECEF position and velocity members of the GpsData_t
 * after it was received via ORION_PKT_GPS_DATA packet, by converting from the
 * LLA and NED data members. Some other null-encoded data members are also
 * set by this function.
 * \param gpsdata_t points to the GpsData_t whose ECEF position and velocity data are updated.
 */
void constructGpsEcefPosVel(void* gpsdata_t)
{
    llaTrig_t trig;
    GpsData_t* gps = (GpsData_t *) gpsdata_t;

    // Compute the ECEF position from the LLA
    llaToECEFandTrig(&gps->Latitude, gps->posEcef, &trig);

    // Compute the ECEF velocity from the NED
    nedToECEFtrigf(gps->VelNED, gps->velEcef, &trig);

    ////////////////////////////////////////////////
    // We actually do some uncertainty conversions here using the older data in
    // the structure, because very old forms of the packet may not have
    // included the detailed uncertainty used by constructGpsEcefUncertainty()

    // Construct an NED position accuracy estimate
    gps->posAccuracy[DOWN] = gps->Vacc;
    gps->posAccuracy[NORTH] = gps->posAccuracy[EAST] = 0.707107f*gps->Hacc;

    // Rotate from NED to ECEF
    nedToECEFtrigf(gps->posAccuracy, gps->posEcefAccuracy, &trig);

    // Construct an NED velocity accuracy estimate
    gps->velAccuracy[NORTH] = gps->velAccuracy[EAST] = gps->velAccuracy[DOWN] = 0.57735f*gps->SpeedAcc;

    // No point in rotating a scalar
    gps->velEcefAccuracy[ECEFX] = gps->velAccuracy[NORTH];
    gps->velEcefAccuracy[ECEFY] = gps->velAccuracy[EAST];
    gps->velEcefAccuracy[ECEFZ] = gps->velAccuracy[DOWN];


    if(gps->Week != 0)
        gps->TimeValid = 1;
    else
        gps->TimeValid = 0;

    // Determine if this is a valid, not deadreckoned, 3D fix
    if((gps->FixType == 3) && (gps->FixState & 0x01) && (gps->TrackedSats >= 4))
        gps->valid3DFix = TRUE;
    else
        gps->valid3DFix = FALSE;

}


/*!
 * Fill out null-encoded ECEF position and velocity uncertainty members of the GpsData_t
 * after it was received via ORION_PKT_GPS_DATA packet, by converting from the
 * LLA and NED data members.
 * \param gpsdata_t points to the GpsData_t whose ECEF uncertainty data are updated.
 */
void constructGpsEcefUncertainty(void* gpsdata_t)
{
    llaTrig_t trig;
    GpsData_t* gps = (GpsData_t *) gpsdata_t;

    if(gps->detailedAccuracyValid)
    {
        // Compute the trigonometric quantities of latitude and longitude
        llaToTrig(&gps->Latitude, &trig);

        // Use them to compute the ECEF accuracy vectors from the NED vectors
        nedToECEFtrigf(gps->posAccuracy, gps->posEcefAccuracy, &trig);
        nedToECEFtrigf(gps->velAccuracy, gps->velEcefAccuracy, &trig);
    }

}

