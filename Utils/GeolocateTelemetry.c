#include "GeolocateTelemetry.h"
#include "earthposition.h"
#include "earthrotation.h"
#include "mathutilities.h"
#include "linearalgebra.h"
#include "WGS84.h"

#include <string.h>


/*!
 * Create a GeolocateTelemetry packet
 * \param pPkt receives the formatted packet
 * \param pGeo is the geo locate telemetry to encode
 */
void FormGeolocateTelemetry(OrionPkt_t *pPkt, const GeolocateTelemetry_t *pGeo)
{
    encodeGeolocateTelemetryCorePacketStructure(pPkt, &pGeo->base);

}// FormGeolocateTelemetry


/*!
 * Parse a GeolocateTelemetry packet
 * \param pPkt is the received packet packet
 * \param pGeo receives the parsed data, including locally constructed data
 * \return TRUE if the packet was successfully decoded
 */
BOOL DecodeGeolocateTelemetry(const OrionPkt_t *pPkt, GeolocateTelemetry_t *pGeo)
{
    // Only parse this packet if the ID and length look right
    if (decodeGeolocateTelemetryCorePacketStructure(pPkt, &pGeo->base))
    {
        // If it decoded properly, extrapolate the core data to the full GeolocateTelemetry_t struct
        ConvertGeolocateTelemetryCore(&pGeo->base, pGeo);
        return TRUE;
    }
    else
        return FALSE;

}// DecodeGeolocateTelemetry


/*!
 * Convert a GeolocateTelemetryCore_t struct to GeolocateTelemetry_t
 * \param pCore is a GeolocateTelemetryCore_t message to be converted
 * \param pGeo receives a copy of pCore, as well as some locally constructed data
 */
void ConvertGeolocateTelemetryCore(const GeolocateTelemetryCore_t *pCore, GeolocateTelemetry_t *pGeo)
{
    stackAllocateDCM(tempDcm);
    float Pan, Tilt;

    // Copy the core data in (if need be)
    if (pCore != &pGeo->base)
        memcpy(&pGeo->base, pCore, sizeof(GeolocateTelemetryCore_t));

    // Date and time
    computeDateAndTimeFromWeekAndItow(pGeo->base.gpsWeek, pGeo->base.gpsITOW, pGeo->base.leapSeconds, &pGeo->Year, &pGeo->Month, &pGeo->Day, &pGeo->Hour, &pGeo->Minute, &pGeo->Second);

    // convert tilt from -180 to 180 into -270 to 90
    if(pGeo->base.tilt > deg2radf(90))
        pGeo->base.tilt -= deg2radf(360);

    // Construct the data that was not transmitted
    structInitDCM(pGeo->gimbalDcm);
    structInitDCM(pGeo->cameraDcm);

    // ECEF position and velocity
    llaToECEFandTrig(&(pGeo->base.posLat), pGeo->posECEF, &pGeo->llaTrig);
    nedToECEFtrigf(pGeo->base.velNED, pGeo->velECEF, &pGeo->llaTrig);

    // Rotation from gimbal to nav
    quaternionToDCM(pGeo->base.gimbalQuat, &pGeo->gimbalDcm);

    // Gimbals Euler attitude
    pGeo->gimbalEuler[AXIS_ROLL]  = dcmRoll(&pGeo->gimbalDcm);
    pGeo->gimbalEuler[AXIS_PITCH] = dcmPitch(&pGeo->gimbalDcm);
    pGeo->gimbalEuler[AXIS_YAW]   = dcmYaw(&pGeo->gimbalDcm);

    // Offset the pan/tilt angles with the current estab output shifts
    Pan  = subtractAnglesf(pGeo->base.pan,  pGeo->base.outputShifts[GIMBAL_AXIS_PAN]);
    Tilt = subtractAnglesf(pGeo->base.tilt, pGeo->base.outputShifts[GIMBAL_AXIS_TILT]);

    // Convert tilt from -180 to 180 into -270 to 90
    pGeo->base.tilt = wrapAngle90f(pGeo->base.tilt);

    // Rotation from camera to gimbal, note that this only works if pan
    // is over tilt (pan first, then tilt, just like Euler)
    if (fabsf(pGeo->base.imageRotation) > radiansf(0.1f))
        setDCMBasedOnEuler(&tempDcm, Pan, Tilt, pGeo->base.imageRotation);
    else
        setDCMBasedOnPanTilt(&tempDcm, Pan, Tilt);

    // Now create the rotation from camera to nav.
    matrixMultiplyf(&pGeo->gimbalDcm, &tempDcm, &pGeo->cameraDcm);

    // The cameras quaternion and Euler angles
    dcmToQuaternion(&pGeo->cameraDcm, pGeo->cameraQuat);
    pGeo->cameraEuler[AXIS_ROLL]  = dcmRoll(&pGeo->cameraDcm);
    pGeo->cameraEuler[AXIS_PITCH] = dcmPitch(&pGeo->cameraDcm);
    pGeo->cameraEuler[AXIS_YAW]   = dcmYaw(&pGeo->cameraDcm);

    // Slant range is the vector magnitude of the line of sight ECEF vector
    pGeo->slantRange = vector3Lengthf(pGeo->base.losECEF);

    // Gimbal ECEF position + line of sight ECEF vector = ECEF image position
    vector3Sum(pGeo->posECEF, vector3Convertf(pGeo->base.losECEF, pGeo->imagePosECEF), pGeo->imagePosECEF);

    // Convert ECEF image position to LLA
    ecefToLLA(pGeo->imagePosECEF, pGeo->imagePosLLA);

}// ConvertGeolocateTelemetryCore


/*!
 * Given a current image location compute a new location based on a angular
 * deviation in camera frame (i.e. user click) and assuming the altitude of the
 * new location is the same as the image location altitude.
 * \param geo is the geolocate telemetry from the gimbal.
 * \param imagePosLLA is the latitude, longitude, altitude of the current image
 *        location in radians, radians, meters.
 * \param ydev is the angular deviation in radians from the image location in
 *        right camera direction.
 * \param zdev is the angular deviation in radians from the image location in
 *        up camera direction.
 * \param newPosLLA receives the position of the user click
 * \param slantRangeM Slant Range to point in Meters
 */
BOOL offsetImageLocation(const GeolocateTelemetry_t *geo, const double imagePosLLA[NLLA], float ydev, float zdev, double newPosLLA[NLLA], double* slantRangeM)
{
    float range, down;
    float vectorNED[NNED];
    float shift[NECEF];

    // Numerical problems at the poles
    if(geo->llaTrig.cosLat == 0)
        return FALSE;

    // Vector from the gimbal to the image location in NED, notice that Altitue and Down have different signs
    vectorNED[NORTH] = (float)((imagePosLLA[LAT] - geo->base.posLat)*datum_meanRadius);
    vectorNED[EAST]  = (float)((imagePosLLA[LON] - geo->base.posLon)*datum_meanRadius*geo->llaTrig.cosLat);
    vectorNED[DOWN]  = (float)(geo->base.posAlt - imagePosLLA[ALT]);

    // Remember this value
    down = vectorNED[DOWN];

    // Second bail out point, the image altitude must be lower than the gimbal
    // altitude (i.e. down must be positive)
    if(down <= 0)
        return FALSE;

    // Range from gimbal to image position.
    range = vector3Lengthf(vectorNED);

    // Adjust the angular deviations to be deviations in meters
    ydev = tanf(ydev)*range;
    zdev = tanf(zdev)*range;

    // The vector of shifts in *camera* frame. Notice that our
    // definition of Z changed. It was given to us positive up, but
    // native axis for the camera is positive down.
    shift[VECTOR3X] = 0;
    shift[VECTOR3Y] = ydev;
    shift[VECTOR3Z] = -zdev;

    // Rotate this shift from camera frame to NED (body to nav)
    dcmApplyRotation(&geo->cameraDcm, shift, shift);

    // Add this to the vector that goes from gimbal to image,
    // to create a new vector that goes from gimbal to new location.
    vector3Sumf(vectorNED, shift, vectorNED);

    // Last bail out point. The new location altitude must be lower than the
    // gimbal altitude (i.e. down must be positive)
    if(vectorNED[DOWN] <= 0)
        return FALSE;

    // Make the vector longer or shorter until it hits the same altitude as
    // before, i.e. it has the same down component as before
    vector3Scalef(vectorNED, vectorNED, down/vectorNED[DOWN]);

    // Finally compute the new location
    newPosLLA[LAT] = addAngles(geo->base.posLat, vectorNED[NORTH]/datum_meanRadius);
    newPosLLA[LON] = addAngles(geo->base.posLon, vectorNED[EAST]/(datum_meanRadius*geo->llaTrig.cosLat));
    newPosLLA[ALT] = geo->base.posAlt - vectorNED[DOWN];

    // Recalcualte slant range after we get the new location.
    // Range from gimbal to image position.
    range = vector3Lengthf(vectorNED);

    // Return Slant Range in meters
    *slantRangeM = (double)range;

    return TRUE;

}// offsetImageLocation

//! Returns the approximate distance to the horizon based on MSL altitude and latitude.
double distanceToHorizonM(double latRad, double altMSLMeters)
{
    double earth_radius_m = datum_meanRadius; //avg earth radius.

    return earth_radius_m * acos( earth_radius_m / (earth_radius_m + altMSLMeters) );

}// distanceToHorizonM

/*!
 * Assuming the gimbal is over the ocean, compute an image location based on
 * angular deviation in camera frame (i.e. user click).
 *
 * \param geo is the geolocate telemetry from the gimbal.
 * \param deltaYawDeg is the angular deviation in radians from the image location in
 *        right camera direction.
 * \param deltaPitchRad is the angular deviation in radians from the image location in
 *        up camera direction.
 * \param newPosLLA receives the position of the user click. If the click is above the
 *        horizon, a point above the ocean at the distance to horizon will be returned.
 * \param slantRangeM Slant Range to point in Meters
 */
void offsetImageLocationOcean( const GeolocateTelemetry_t *geoloc, float deltaYawRad, float deltaPitchRad, double newPosLLA[NLLA], double* slantRangeM)
{
    // Get the DCM to rotate from the delta position in FOV to camera centerline
    stackAllocateDCM(deltaDcm);
    setDCMBasedOnPanTilt(&deltaDcm, deltaYawRad, deltaPitchRad);

    // Get the total rotation from the delta position to NED
    stackAllocateDCM(dcmDeltaToNED);
    dcmMultiply(&geoloc->cameraDcm, &deltaDcm, &dcmDeltaToNED);

    // The line-of-sight unit vector from the gimbal is the first column (note switch to [N,E,UP]
    double u = -(double)dcmGet(&dcmDeltaToNED,2,0);

    // Use the geocentric radius of the earth at the gimbal
    double geocentric[NLLA] = {geoloc->base.posLat, geoloc->base.posLon, 0};
    geodeticToGeocentric(geocentric, geocentric);

    // The gimbal position is the origin in x,y and z is relative to center of sphere: [0,0,geocentricAlt]
    double wgsradius_m = radiusOfEWCurv(geoloc->base.posLat);
    double gimbal_alt_msl_m = geoloc->base.posAlt - geoloc->base.geoidUndulation;
    double g = wgsradius_m + gimbal_alt_msl_m;

    double ug = u*g;

    // Check for a possible solution
    double q = ug*ug - g*g + SQR(wgsradius_m);
    // if q < 0 no solution
    double range_to_intercept = -1;
    // if q == 0 the solution will be the horizon, numerically this is unlikely to happen
    // but handle it anyway, d is in meters so use a "small" epsilon relative to the radius of earth
    if(fabs(q) < 10.)
        range_to_intercept = -ug;
    // if q > 0 take the smaller (positive result) of the two solutions
    else if( q > 0 )
    {
        q = sqrt(q);
        double s1 = -ug - q;
        double s2 = -ug + q;
        if( s1 > 0 && s2 > 0 )
            range_to_intercept = MIN(s1, s2);
        else if( s1 > 0 )
            range_to_intercept = s1;
        else if( s2 > 0 )
            range_to_intercept = s2;
        else
        {
            // Error: no solution
            // If we are above the horizon, use range to horizon.
            range_to_intercept = distanceToHorizonM( geoloc->base.posLat, gimbal_alt_msl_m );
        }
    }
    else
    {
        // If we are above the horizon, use range to horizon.
        range_to_intercept = distanceToHorizonM( geoloc->base.posLat, gimbal_alt_msl_m );
    }

    *slantRangeM = range_to_intercept;

    // Now that we have the range to the sea (or a point in the sky out at the range to horizon), project a
    // vector from the gimbal along our offset at the given range to get its LLA.
    double GimbalLla[NLLA] = { geoloc->base.posLat, geoloc->base.posLon, geoloc->base.posAlt };

    double GimbalEcef[NECEF], ImgPosECEF[NECEF];
    float LineOfSight[NECEF], Ecef[NECEF];
    stackAllocateDCM(Dcm);
    llaTrig_t Trig;

    // Create a line of sight vector looking north.
    LineOfSight[ECEFX] = range_to_intercept;
    LineOfSight[ECEFY] = 0.0f;
    LineOfSight[ECEFZ] = 0.0f;

    // Now rotate that vector into the nav frame
    setDCMBasedOnEuler(&Dcm, dcmYaw(&dcmDeltaToNED), dcmPitch(&dcmDeltaToNED), dcmRoll(&dcmDeltaToNED));
    dcmApplyRotation(&Dcm, LineOfSight, LineOfSight);

    // Get gimbal pos in ECEF
    llaToECEFandTrig(GimbalLla, GimbalEcef, &Trig);

    // Rotate from the nav frame to ECEF
    nedToECEFtrigf(LineOfSight, Ecef, &Trig);

    // Add the line of sight vector to the gimbal ECEF position to get image pos in
    // ECEF.
    ImgPosECEF[ECEFX] = Ecef[ECEFX] + GimbalEcef[ECEFX];
    ImgPosECEF[ECEFY] = Ecef[ECEFY] + GimbalEcef[ECEFY];
    ImgPosECEF[ECEFZ] = Ecef[ECEFZ] + GimbalEcef[ECEFZ];

    // Convert img ECEF to LLA and store result in parameter from caller.
    ecefToLLA( ImgPosECEF, newPosLLA );

}// offsetImageLocationOcean

/*! Get the terrain intersection of the current line of sight given gimbal geolocate telemetry data.
 *  \param pGeo[in] A pointer to incoming geolocate telemetry data
 *  \param getElevationHAE[in] Pointer to a terrain model lookup function, which should take a lat/lon
 *                             pair (in radians) and return the height above ellipsoid of that point
 *  \param PosLLA[out] Terrain intersection location in the LLA frame
 *  \param pRange[out] Target range in meters to be sent to the gimbal
 *  \return TRUE if a valid intersection was found, otherwise FALSE. Note that if this function
 *          returns FALSE, the data in PosLLA and pRange will still be overwritten with invalid data.
 */
BOOL getTerrainIntersection(const GeolocateTelemetry_t *pGeo, float (*getElevationHAE)(double, double), double PosLLA[NLLA], double *pRange)
{
    double UnitNED[NNED], UnitECEF[NECEF], LineOfSight[NECEF], Step, End;
    float Temp[NNED] = { 1.0f, 0.0f, 0.0f };

    // Coarse and fine line of sight ray step distances, in meters
    static const double StepCoarse = 30.0, StepFine = 1.0;

    // Maximum distance to follow a ray before giving up
    static const double MaxDistance = 15000.0;

    // Rotate a unit line of sight vector by the camera DCM to get a 1-meter NED look vector
    dcmApplyRotation(&pGeo->cameraDcm, Temp, Temp);

    // Convert the unit vector in Temp[NNED] from single to double precision
    vector3Convertf(Temp, UnitNED);

    // Convert the unit vector to ECEF
    nedToECEFtrig(UnitNED, UnitECEF, &pGeo->llaTrig);

    // Start with a step value of StepCoarse and loop until MaxDistance
    Step = StepCoarse;
    End = MaxDistance;

    // Scale the unit ECEF vector to the step length
    vector3Scale(UnitECEF, UnitECEF, Step);

    // Initialize the line of sight vector with the gimbal position
    vector3Copy(pGeo->posECEF, LineOfSight);

    // Loop through LOS ranges
    for (*pRange = Step; *pRange <= End; *pRange += Step)
    {
        double GroundHeight;

        // Increment the ECEF line of sight vector by the Step-sized unit vector
        vector3Sum(LineOfSight, UnitECEF, LineOfSight);

        // Convert the ECEF line of sight position to LLA
        ecefToLLA(LineOfSight, PosLLA);

        // Get the ground HAE
        GroundHeight = getElevationHAE(PosLLA[LAT], PosLLA[LON]);

        // If we're still coarsely stepping
        if (Step != StepFine)
        {
            // Set the step size to the greater of 1% of current range or StepCoarse
            Step = MAX(StepCoarse, *pRange * 0.01f);
        }

        // If the end of this ray is under ground
        if (PosLLA[ALT] <= GroundHeight)
        {
            // Decrease the range by one step
            if (Step != StepFine)
            {
                // Back up one step
                *pRange -= Step;

                // Change to the fine step and loop until the current range
                End = *pRange + Step;
                Step = StepFine;

                // Subtract one unit step from the LOS vector and rescale the unit to the new step distance
                vector3Difference(LineOfSight, UnitECEF, LineOfSight);
                vector3ChangeLength(UnitECEF, UnitECEF, Step);
            }
            // If we're fine stepping, we've found the terrain intersection
            else
            {
                // Clamp the altitude to the ground and tell the caller that the image position is good
                PosLLA[ALT] = GroundHeight;
                return TRUE;
            }
        }
    }

    // No valid image position
    return FALSE;

}// getTerrainIntersection


/*!
 * Get the velocity of the terrain intersection
 * \param buf points to the geolocate buffer
 * \param dt is the desired timer interval in milliseconds
 * \param imageVel receives the velocity of the image in North, East Down, meters
 * \return TRUE if the velocity was computed, else FALSE
 */
BOOL getImageVelocity(const GeolocateBuffer_t* buf, uint32_t dt, float imageVel[NNED])
{
    GeolocateTelemetry_t Old, New;
    int32_t diff;

    if (getGeolocateBuffer(buf, dt, &Old) == FALSE)
        return FALSE;
    else if (getGeolocateBuffer(buf, 0, &New) == FALSE)
        return FALSE;

    // Compute time delta in milliseconds
    diff = New.base.systemTime - Old.base.systemTime;


    // If the newest entry has no range data, don't compute anything. Also skip internal
    //   range estimates because they assume a velocity of zero
    if ((New.base.rangeSource == RANGE_SRC_NONE) || (New.base.rangeSource == RANGE_SRC_INTERNAL))
        return FALSE;
    // Otherwise, delta time is good and the two range sources match
    else if ((diff >= (int32_t)dt) && (Old.base.rangeSource == New.base.rangeSource))
    {
        double DeltaECEF[NECEF], DeltaNED[NNED];

        // Compute the NED distance between the two image positions
        vector3Difference(New.imagePosECEF, Old.imagePosECEF, DeltaECEF);
        ecefToNEDtrig(DeltaECEF, DeltaNED, &New.llaTrig);
        vector3Convert(DeltaNED, imageVel);

        // Now convert to velocity by multiplying by 1000 / dt (in ms)
        vector3Scalef(imageVel, imageVel, 1000.0f / diff);
        return TRUE;
    }

    return FALSE;

}// getImageVelocity


/*!
 * Get a buffered geolocate telemetry struct
 * \param buf points to the geolocate buffer
 * \param dt is the desired timer interval in milliseconds
 * \param imageVel receives the velocity of the gimbal in North, East Down, meters
 * \return TRUE if the velocity was computed, else FALSE
 */
BOOL getGeolocateBuffer(const GeolocateBuffer_t* buf, uint32_t dt, GeolocateTelemetry_t* geo)
{
    int newest, oldest, index;

    if(buf->holding < 2)
        return FALSE;

    // The newest entry is one behind the in pointer
    newest = buf->in - 1;
    if(newest < 0)
        newest += GEOLOCATE_BUFFER_SIZE;

    // If the user is asking for the newest buffer
    if (dt == 0)
    {
        // Copy it out immediately and return TRUE
        copyGeolocateTelemetry(&buf->geobuf[newest], geo);
        return TRUE;
    }

    // The oldest entry (if holding == 1, then oldest and newest are the same)
    oldest = newest - (buf->holding - 1);
    if(oldest < 0)
        oldest += GEOLOCATE_BUFFER_SIZE;

    // Go backwards in time until we match or exceed dt
    index = newest - 1;
    if(index < 0)
        index += GEOLOCATE_BUFFER_SIZE;

    while (index != oldest)
    {
        // The difference in time in milliseconds
        const GeolocateTelemetry_t *pOld = &buf->geobuf[index], *pNew = &buf->geobuf[newest];
        int32_t diff = pNew->base.systemTime - pOld->base.systemTime;

        // If we've got at least dt milliseconds of data
        if (diff >= (int32_t)dt)
        {
            copyGeolocateTelemetry(pOld, geo);
            return TRUE;
        }

        // Go back one more
        if (--index < 0)
            index += GEOLOCATE_BUFFER_SIZE;
    }

    // Couldn't find a buffer at least dt milliseconds old
    return FALSE;

}// getGeolocateBuffer


/*!
 * Push a new geolocate telemetry into a geolocate buffer
 * \param buf is the geolocate buffer to put new data into
 * \param geo is the new data to load, which will be copied into the buffer
 */
void pushGeolocateBuffer(GeolocateBuffer_t* buf, const GeolocateTelemetry_t* geo)
{
    // Copy the data into the buffer, cannot do simple assignment
    copyGeolocateTelemetry(geo, &buf->geobuf[buf->in]);

    // Adjust in pointer, in always points to the next entry to go in, which is
    // also the oldest entry when the buffer is full
    buf->in++;
    if(buf->in >= GEOLOCATE_BUFFER_SIZE)
        buf->in = 0;

    // Count number of entries
    if(buf->holding < GEOLOCATE_BUFFER_SIZE)
        buf->holding++;

}// pushGeolocateBuffer


/*!
 * Copy a geolocate structure, which cannot be done with simple assignment due to the DCM pointers
 * \param source is the source structure whose contents are copied.
 * \param dest receives a copy of the data in source.
 */
void copyGeolocateTelemetry(const GeolocateTelemetry_t* source, GeolocateTelemetry_t* dest)
{
    // Simple assignment here
    (*dest) = (*source);

    // Now fix the pointers, the dest DCM data pointers need to be pointing at the dest data, not the source data
    dest->cameraDcm.data = &dest->cameraDcmdata[0];
    dest->gimbalDcm.data = &dest->gimbalDcmdata[0];
}

