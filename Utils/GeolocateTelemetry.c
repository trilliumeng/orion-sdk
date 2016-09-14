#include "GeolocateTelemetry.h"
#include "earthposition.h"
#include "earthrotation.h"
#include "mathutilities.h"
#include "WGS84.h"

//! Number of days between Jan 6 1980 and Jan 1 2012
#define JAN12012 11683

//! Day number at start of each month for a common year, day and month are zero based
static const uint16_t month_day_norm[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

//! Day number at start of each month for a leap year, day and month are zero based
static const uint16_t month_day_leap[12] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};

//! Determine if a year is a leap year in the Gregorian calendar.
int isLeapYear(uint16_t year);

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
		stackAllocateDCM(tempDcm);
        float Pan, Tilt;

        // Date and time
        computeDateFromWeekAndItow(pGeo->base.gpsWeek, pGeo->base.gpsITOW, &pGeo->Year, &pGeo->Month, &pGeo->Day);
        pGeo->Hour   = (pGeo->base.gpsITOW % (24*60*60*1000))/(60*60*1000);
        pGeo->Minute = (pGeo->base.gpsITOW %    (60*60*1000))/(60*1000);
        pGeo->Second = (pGeo->base.gpsITOW %       (60*1000))/(1000);

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
        Pan  = pGeo->base.pan  - pGeo->base.outputShifts[GIMBAL_AXIS_PAN];
        Tilt = pGeo->base.tilt - pGeo->base.outputShifts[GIMBAL_AXIS_TILT];

		// Rotation from camera to gimbal, note that this only works if pan
		// is over tilt (pan first, then tilt, just like Euler)
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

		return TRUE;
	}
	else
		return FALSE;

}// DecodeGeolocateTelemetry


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
 */
BOOL offsetImageLocation(const GeolocateTelemetry_t *geo, const double imagePosLLA[NLLA], float ydev, float zdev, double newPosLLA[NLLA])
{
    float range, down;
    float vectorNED[NNED];
    float shift[NECEF];

    // Numerical problems at the poles
    if(geo->llaTrig.cosLat == 0)
        return FALSE;

    // Vector from the gimbal to the image location in NED, notice that Altitue and Down have different signs
    vectorNED[NORTH] = (float)(imagePosLLA[LAT] - geo->base.posLat)*datum_meanRadius;
    vectorNED[EAST]  = (float)(imagePosLLA[LON] - geo->base.posLon)*datum_meanRadius*geo->llaTrig.cosLat;
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

    return TRUE;

}// offsetImageLocation


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
    double UnitNED[NNED], UnitECEF[NECEF], Step, End;
    float Temp[NNED] = { 1.0f, 0.0f, 0.0f };

    // Coarse and fine line of sight ray step distances, in meters
    static const double StepCoarse = 30.0, StepFine = 1.0;

    // Maximum distance to follow a ray before giving up
    static const double MaxDistance = 5000.0;

    // Rotate a unit line of sight vector by the camera DCM to get a 1-meter NED look vector
    dcmApplyRotation(&pGeo->cameraDcm, Temp, Temp);

    // Convert the unit vector in Temp[NNED] from single to double precision
    vector3Convertf(Temp, UnitNED);

    // Convert the unit vector to ECEF
    nedToECEFtrig(UnitNED, UnitECEF, &pGeo->llaTrig);

    // Start with a step value of StepCoarse and loop until MaxDistance
    Step = StepCoarse;
    End = MaxDistance;

    // Loop through LOS ranges
    for (*pRange = Step; *pRange <= End; *pRange += Step)
    {
        double LineOfSight[NECEF], GroundHeight;

        // Scale the unit LOS vector to the appropriate range
        vector3Scale(UnitECEF, LineOfSight, *pRange);

        // Now add the gimbal position to the line of sight vector to get ECEF position
        vector3Sum(pGeo->posECEF, LineOfSight, LineOfSight);

        // Convert the ECEF line of sight position to LLA
        ecefToLLA(LineOfSight, PosLLA);

        // Get the ground HAE
        GroundHeight = getElevationHAE(PosLLA[LAT], PosLLA[LON]);

        // If the end of this ray is under ground
        if (PosLLA[ALT] <= GroundHeight)
        {
            // If we're using a coarse step
            if (Step == StepCoarse)
            {
                // Back up one step
                *pRange -= Step;

                // Change to the fine step and loop until the current range
                Step = StepFine;
                End = *pRange + StepCoarse;
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
 * Use GPS time information to compute the Gregorian calendar date.
 * This only works for dates after Jan 1 2012.
 * \param week is the GPS week number
 * \param itow is the GPS time of week in milliseconds
 * \param pyear receives the Gregorian year
 * \param pmonth receives the month of the year, from 1 (January) to 12 (December)
 * \param pday receives the day of the month, from 1 to 31
 */
void computeDateFromWeekAndItow(uint16_t week, uint32_t itow, uint16_t* pyear, uint8_t* pmonth, uint8_t* pday)
{
    // If the date is after 1/1/2012
    if (week >= JAN12012 / 7)
    {
        int isleap;
        uint8_t   month;
        const uint16_t* month_day;
        uint16_t dayinyear;

        // We don't support dates before this year
        uint16_t year = 2012;

        // Number of days since Jan 6 1980.
        uint32_t days = week*7 + itow/86400000;

        // Convert to number of days since jan 1 2012
        days -= JAN12012;

        // The number of days in a year depends on if it is a leap year
        isleap = isLeapYear(year);
        if(isleap)
            dayinyear = 366;
        else
            dayinyear = 365;

        while(days >= dayinyear)
        {
            // subtract off the days
            days -= dayinyear;

            // Go to the next year
            year++;

            isleap = isLeapYear(year);
            if(isleap)
                dayinyear = 366;
            else
                dayinyear = 365;

        }// while finding the year

        // now we know the year
        *pyear = year;

        // The month_day list is leap year dependent
        if(isleap)
            month_day = month_day_leap;
        else
            month_day = month_day_norm;

        // Now we have the year, we need to get the month.
        for(month = 1; month < 12; month++)
        {
            if(days < month_day[month])
                break;
        }

        // i is the zero-based month from 1(feb) to 12(dec+1), whose starting day is after
        // the days, we want the zero based month from 0 to 11 whose starting
        // day is just before or equal to days
        month--;

        // Subtract off the number of days for the month
        days -= month_day[month];

        // Record days and months, notice they are 1-based
        *pday = days+1;
        *pmonth = month+1;
    }
    else
    {
        // Otherwise, party like it's 1980
        *pyear = 1980;
        *pmonth = 1;
        *pday = 6;
    }

}// computeDateFromWeekAndITOW


/*!
 * Compute hours, minutes, and seconds from GPS time of week
 * \param itow is the GPS time of week in milliseconds
 * \param hour receives the hour of the day from 0 to 23
 * \param min receives the minute of the hour from 0 to 59
 * \param second receives the seconds of the minute from 0 to 59
 */
void computeTimeFromItow(uint32_t itow, uint8_t* hour, uint8_t* min, uint8_t* second)
{
	// Milliseconds of the day
	itow = itow % 86400000;

	// Compute hours of the day
	(*hour) = (uint8_t)(itow / (60*60*1000));

	// Subtract off hours
	itow -= (*hour)*60*60*1000;

	// Compute minutes of the hour
	(*min)  = (uint8_t)(itow / (60*1000));

	// Subtract off minutes
	itow -= (*min)*60*1000;

	// Compute seconds of the minute
	(*second)  = (uint8_t)(itow / (1000));

}


/*!
 * Determine if a year is a leap year in the Gregorian calendar.
 * \param year is the year number.
 * \return 0 for a common year (365 days), 1 for a leap year (366 days)
 */
int isLeapYear(uint16_t year)
{
	// Leap year rules:
	// 1) every 4th year is a leap year unless:
	// 2) the year is modulo 100 its not a leap year, unless:
	// 3) the year is module 400 it is a leap year. So:
	// 1896 is leap year,
	// 1900 is not (modulo 100),
	// 1904 is
	// 1996 is
	// 2000 is (modulo 400)
	// 2004 is
	// 2100 is not (module 100)
	// 2400 is (modulo 400)

	if(year & 0x03)
		return 0;	// not modulo 4, not a leap year
	else
	{
		if((year % 100) != 0)
			return 1;	// modulo 4, but not modulo 100, leap year
		else if((year % 400) != 0)
			return 0;	// modul0 100, but not modulo 400, not a leap year
		else
			return 1;	// modulo 400, leap year
	}

}// isLeapYear


/*!
 * Use Gregorian date information to compute GPS style time information. The
 * output is either in GPS time or UTC time, depending on which time reference
 * is used for the inputs. This function only works for dates after Jan 1 2012
 * \param year is the Gregorian calendar year
 * \param month is the month of the year, from 1 (January) to 12 (December)
 * \param day is the day of the month, from 1 to 31
 * \param hours is the hour of the dya, from 0 to 23
 * \param minutes is the minute of the hour, from 0 to 59
 * \param seconds is the seconds of the minute, from 0 to 60 (60 may occur in a leap second jump)
 * \param milliseconds is the milliseconds of the second
 * \param pweek receives the number of weeks since Jan 6 1980 (aka GPS week number)
 * \param pitow receives the time of week in milliseconds
 */
void computeWeekAndItow(uint16_t year, uint8_t month, uint8_t day, uint8_t hours, uint8_t minutes, uint8_t seconds, int16_t milliseconds, uint16_t* pweek, uint32_t* pitow)
{
	// Number of days between Jan 6 1980 and Jan 1 2012
	uint32_t days = JAN12012;
	uint32_t week;
	uint32_t itow;

	// 0 based month
	month--;

	// 0 based day
	day--;

	// Days of the months already in this year
	days += month_day_norm[month];

	// Add in day of the month
	days += day;

	// Has leap day happened?, add a day
	if(isLeapYear(year) && (month>=2))
		days++;

	// Now add up all the years prior to this one.
	// Counting number of days to Jan 1 year 2012
	while(year > 2012)
	{
		year--;

		if(isLeapYear(year))
			days += 366;
		else
			days += 365;

	}// While still years to count down

	// Now that we have the number of days we can compute the week number
	week = days/7;

	// subtract off the days that are accounted for in the week
	days -= week*7;

	// convert the times to milliseconds
	itow = (((days*24 + hours)*60 + minutes)*60 + seconds)*1000 + milliseconds;

	// return the results
	*pweek = (uint16_t)week;
	*pitow = itow;

}// computeWeekAndItow


/*!
 * Test the date conversion logic
 * \return 1 if good, 0 if bad
 */
int testDateConversion(void)
{
	uint16_t week;
	uint32_t itow;
	uint16_t year;
	uint8_t month, day;

	// 2016 is a leap year, and march 12 is after leap day
	computeWeekAndItow(2016, 3, 12, 9, 10, 11, 250, &week, &itow);

	computeDateFromWeekAndItow(week, itow, &year, &month, &day);

	if(year != 2016)
		return 0;
	else if(month != 3)
		return 0;
	else if(day != 12)
		return 0;
	else if(week != 1887)
		return 0;
	else if(itow != (((6*24 + 9)*60 + 10)*60 + 11)*1000 + 250)
		return 0;
	else
		return 1;

}


