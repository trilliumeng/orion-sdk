#ifndef KLVPARSER_H
#define KLVPARSER_H

#include <stdint.h>

typedef enum
{
	KLV_UAS_NULL,// = 0
    KLV_UAS_CHECKSUM,// = 1
    KLV_UAS_TIME_STAMP,// = 2
    KLV_UAS_MISSION_ID,// = 3
    KLV_UAS_TAIL_NUMBER,// = 4
    KLV_UAS_PLATFORM_YAW,// = 5
    KLV_UAS_PLATFORM_PITCH_SHORT,// = 6
    KLV_UAS_PLATFORM_ROLL_SHORT,// = 7
    KLV_UAS_PLATFORM_TAS,// = 8
    KLV_UAS_PLATFORM_IAS,// = 9
    KLV_UAS_PLATFORM_ID,// = 10
    KLV_UAS_SENSOR_ID,// = 11
    KLV_UAS_COORD_SYSTEM,// = 12
    KLV_UAS_SENSOR_LAT,// = 13
    KLV_UAS_SENSOR_LON,// = 14
    KLV_UAS_SENSOR_MSL,// = 15
    KLV_UAS_SENSOR_HFOV,// = 16
    KLV_UAS_SENSOR_VFOV,// = 17
    KLV_UAS_SENSOR_PAN,// = 18
    KLV_UAS_SENSOR_TILT,// = 19
    KLV_UAS_SENSOR_ROLL,// = 20
    KLV_UAS_SLANT_RANGE,// = 21
    KLV_UAS_TARGET_WIDTH,// = 22
    KLV_UAS_IMAGE_LAT,// = 23
    KLV_UAS_IMAGE_LON,// = 24
    KLV_UAS_IMAGE_MSL,// = 25
    KLV_UAS_CORNER1_LAT_OFFSET,// = 26
    KLV_UAS_CORNER1_LON_OFFSET,// = 27
    KLV_UAS_CORNER2_LAT_OFFSET,// = 28
    KLV_UAS_CORNER2_LON_OFFSET,// = 29
    KLV_UAS_CORNER3_LAT_OFFSET,// = 30
    KLV_UAS_CORNER3_LON_OFFSET,// = 31
    KLV_UAS_CORNER4_LAT_OFFSET,// = 32
    KLV_UAS_CORNER4_LON_OFFSET,// = 33
    KLV_UAS_ICING,// = 34
    KLV_UAS_WIND_DIRECTION,// = 35
    KLV_UAS_WIND_SPEED,// = 36
    KLV_UAS_STATIC_PRESSURE,// = 37
    KLV_UAS_DENSITY_ALTITUDE,// = 38
    KLV_UAS_OAT,// = 39
    KLV_UAS_TARGET_LAT,// = 40
    KLV_UAS_TARGET_LON,// = 41
    KLV_UAS_TARGET_MSL,// = 42
    KLV_UAS_TRACK_WIDTH,// = 43
    KLV_UAS_TRACK_HEIGHT,// = 44
    KLV_UAS_TARGET_CE90,// = 45
    KLV_UAS_TARGET_LE90,// = 46
    KLV_UAS_GENERIC_FLAG,// = 47
    KLV_UAS_SECURITY,// = 48
    KLV_UAS_DYNAMIC_PRESSURE,// = 49
    KLV_UAS_ANGLE_OF_ATTACK_SHORT,// = 50
    KLV_UAS_VERTICAL_VELOCITY,// = 51
    KLV_UAS_SIDESLIP_SHORT,// = 52
    KLV_UAS_AIRFIELD_PRESSURE,// = 53
    KLV_UAS_AIRFIELD_ELEVATION,// = 54
    KLV_UAS_HUMIDITY,// = 55
    KLV_UAS_GROUND_SPEED,// = 56
    KLV_UAS_GROUND_RANGE,// = 57
    KLV_UAS_FUEL_REMAINING,// = 58
    KLV_UAS_CALLSIGN,// = 59
    KLV_UAS_WEAPON_LOAD,// = 60
    KLV_UAS_WEAPON_FIRED,// = 61
    KLV_UAS_LASER_PRF_CODE,// = 62
    KLV_UAS_FOV_NAME,// = 63
    KLV_UAS_MAGNETIC_HEADING,// = 64
    KLV_UAS_VERSION,// = 65
    KLV_UAS_TARGET_COVARIANCE,// = 66
    KLV_UAS_ALT_PLATFORM_LAT,// = 67
    KLV_UAS_ALT_PLATFORM_LON,// = 68
    KLV_UAS_ALT_PLATFORM_MSL,// = 69
    KLV_UAS_ALT_PLATFORM_NAME,// = 70
    KLV_UAS_ALT_PLATFORM_HEADING,// = 71
    KLV_UAS_EVENT_START_TIME,// = 72
    KLV_UAS_RVT_LOCAL_SET,// = 73
    KLV_UAS_VMTI,// = 74
    KLV_UAS_SENSOR_HAE,// = 75
    KLV_UAS_ALT_PLATFORM_HAE,// = 76
    KLV_UAS_OPERATIONAL_MODE,// = 77
    KLV_UAS_IMAGE_HAE,// = 78
    KLV_UAS_NORTH_VELOCITY,// = 79
    KLV_UAS_EAST_VELOCITY,// = 80
    KLV_UAS_IMAGE_HORIZON,// = 81
    KLV_UAS_CORNER1_LAT,// = 82
    KLV_UAS_CORNER1_LON,// = 83
    KLV_UAS_CORNER2_LAT,// = 84
    KLV_UAS_CORNER2_LON,// = 85
    KLV_UAS_CORNER3_LAT,// = 86
    KLV_UAS_CORNER3_LON,// = 87
    KLV_UAS_CORNER4_LAT,// = 88
    KLV_UAS_CORNER4_LON,// = 89
    KLV_UAS_PLATFORM_PITCH,// = 90
    KLV_UAS_PLATFORM_ROLL,// = 91
    KLV_UAS_ANGLE_OF_ATTACK,// = 92
    KLV_UAS_SIDESLIP,// = 93
    KLV_UAS_CORE_ID,// = 94
    KLV_UAS_NUM_ELEMENTS
} KlvUasDataElement_t;

typedef enum
{
    KLV_SECURITY_NULL, // = 0
    KLV_SECURITY_CLASSIFICATION, // = 1
    KLV_SECURITY_CLASSIFYING_CODING, // = 2
    KLV_SECURITY_CLASSIFYING_COUNTRY, // = 3
    KLV_SECURITY_HANDLING_INSTRUCTIONS, // = 4
    KLV_SECURITY_CAVEATS, // = 5
    KLV_SECURITY_RELEASING_INSTRUCTIONS, // = 6
    KLV_SECURITY_CLASSIFIED_BY, // = 7
    KLV_SECURITY_DERIVED_FROM, // = 8
    KLV_SECURITY_CLASSIFICATION_REASON, // = 9
    KLV_SECURITY_DECLASSIFICATION_DATE, // = 10
    KLV_SECURITY_CLASSIFICATION_MARKING, // = 11
    KLV_SECURITY_OBJECT_COUNTRY_CODING, // = 12
    KLV_SECURITY_OBJECT_COUNTRY, // = 13
    KLV_SECURITY_CLASSIFICATION_COMMENTS, // = 14
    KLV_SECURITY_NULL1, // = 15
    KLV_SECURITY_NULL2, // = 16
    KLV_SECURITY_NULL3, // = 17
    KLV_SECURITY_NULL4, // = 18
    KLV_SECURITY_NULL5, // = 19
    KLV_SECURITY_NULL6, // = 20
    KLV_SECURITY_NULL7, // = 21
    KLV_SECURITY_VERSION, // = 22
    KLV_SECURITY_CLASSIFYING_CODING_VERSION, // = 23
    KLV_SECURITY_OBJECT_COUNTRY_CODING_VERSION, // = 24
    KLV_SECURITY_NUM_ELEMENTS
} KlvSecurityDataElement_t;

void KlvNewData(const uint8_t *pData, int Length);

double KlvGetValueDouble(KlvUasDataElement_t Element, int *pResult);
int64_t KlvGetValueInt(KlvUasDataElement_t Element, int *pResult);
uint64_t KlvGetValueUInt(KlvUasDataElement_t Element, int *pResult);
const char *KlvGetValueString(KlvUasDataElement_t Element);

void KlvPrintData(void);
void KlvPrintSecurityData(const uint8_t *pData, int passedLength);

#endif // KLVPARSER_H
