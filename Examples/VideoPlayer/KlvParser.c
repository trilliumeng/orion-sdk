#include "KlvParser.h"
#include "KlvTree.h"

#include "Constants.h"

#include <inttypes.h>
#include <stdio.h>

// Define to output data in radians, undefine for degrees
#define RADIANS

#ifdef RADIANS
# define CONVERT_ANGLE(x) radians(x)
#else
# define CONVERT_ANGLE(x) (x)
#endif // RADIANS

typedef enum
{
    KLV_TYPE_UINT,
    KLV_TYPE_INT,
    KLV_TYPE_DOUBLE,
    KLV_TYPE_STRING,
    KLV_TYPE_OTHER
} KlvType_t;

typedef struct
{
    char Name[64];
    KlvType_t Type;
    double Min;
    double Max;
} KlvTagInfo_t;

KlvTagInfo_t TagInfo[KLV_UAS_NUM_ELEMENTS] = {
    { "KLV_UAS_NULL", /* = 0, */                  KLV_TYPE_OTHER },
    { "KLV_UAS_CHECKSUM", /* = 1, */              KLV_TYPE_UINT },
    { "KLV_UAS_TIME_STAMP", /* = 2, */            KLV_TYPE_UINT },
    { "KLV_UAS_MISSION_ID", /* = 3, */            KLV_TYPE_STRING },
    { "KLV_UAS_TAIL_NUMBER", /* = 4, */           KLV_TYPE_STRING },
    { "KLV_UAS_PLATFORM_YAW", /* = 5, */          KLV_TYPE_DOUBLE, CONVERT_ANGLE(0), CONVERT_ANGLE(360) },
    { "KLV_UAS_PLATFORM_PITCH_SHORT", /* = 6 */   KLV_TYPE_DOUBLE, CONVERT_ANGLE(-20), CONVERT_ANGLE(20) },
    { "KLV_UAS_PLATFORM_ROLL_SHORT", /* = 7 */    KLV_TYPE_DOUBLE, CONVERT_ANGLE(-50), CONVERT_ANGLE(50) },
    { "KLV_UAS_PLATFORM_TAS", /* = 8, */          KLV_TYPE_UINT },
    { "KLV_UAS_PLATFORM_IAS", /* = 9, */          KLV_TYPE_UINT },
    { "KLV_UAS_PLATFORM_ID", /* = 10 */           KLV_TYPE_STRING },
    { "KLV_UAS_SENSOR_ID", /* = 11 */             KLV_TYPE_STRING },
    { "KLV_UAS_COORD_SYSTEM", /* = 12 */          KLV_TYPE_STRING },
    { "KLV_UAS_SENSOR_LAT", /* = 13 */            KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_SENSOR_LON", /* = 14 */            KLV_TYPE_DOUBLE, CONVERT_ANGLE(-180), CONVERT_ANGLE(180) },
    { "KLV_UAS_SENSOR_MSL", /* = 15 */            KLV_TYPE_DOUBLE, -900, 19000 },
    { "KLV_UAS_SENSOR_HFOV", /* = 16 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(0), CONVERT_ANGLE(180) },
    { "KLV_UAS_SENSOR_VFOV", /* = 17 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(0), CONVERT_ANGLE(180) },
    { "KLV_UAS_SENSOR_PAN", /* = 18 */            KLV_TYPE_DOUBLE, CONVERT_ANGLE(0), CONVERT_ANGLE(360) },
    { "KLV_UAS_SENSOR_TILT", /* = 19 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(-180), CONVERT_ANGLE(180) },
    { "KLV_UAS_SENSOR_ROLL", /* = 20 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(0), CONVERT_ANGLE(360) },
    { "KLV_UAS_SLANT_RANGE", /* = 21 */           KLV_TYPE_DOUBLE, 0, 5000000 },
    { "KLV_UAS_TARGET_WIDTH", /* = 22 */          KLV_TYPE_DOUBLE, 0, 10000 },
    { "KLV_UAS_IMAGE_LAT", /* = 23 */             KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_IMAGE_LON", /* = 24 */             KLV_TYPE_DOUBLE, CONVERT_ANGLE(-180), CONVERT_ANGLE(180) },
    { "KLV_UAS_IMAGE_MSL", /* = 25 */             KLV_TYPE_DOUBLE, -900, 19000 },
    { "KLV_UAS_CORNER1_LAT_OFFSET", /* = 26 */    KLV_TYPE_DOUBLE, CONVERT_ANGLE(-0.075), CONVERT_ANGLE(0.075) },
    { "KLV_UAS_CORNER1_LON_OFFSET", /* = 27 */    KLV_TYPE_DOUBLE, CONVERT_ANGLE(-0.075), CONVERT_ANGLE(0.075) },
    { "KLV_UAS_CORNER2_LAT_OFFSET", /* = 28 */    KLV_TYPE_DOUBLE, CONVERT_ANGLE(-0.075), CONVERT_ANGLE(0.075) },
    { "KLV_UAS_CORNER2_LON_OFFSET", /* = 29 */    KLV_TYPE_DOUBLE, CONVERT_ANGLE(-0.075), CONVERT_ANGLE(0.075) },
    { "KLV_UAS_CORNER3_LAT_OFFSET", /* = 30 */    KLV_TYPE_DOUBLE, CONVERT_ANGLE(-0.075), CONVERT_ANGLE(0.075) },
    { "KLV_UAS_CORNER3_LON_OFFSET", /* = 31 */    KLV_TYPE_DOUBLE, CONVERT_ANGLE(-0.075), CONVERT_ANGLE(0.075) },
    { "KLV_UAS_CORNER4_LAT_OFFSET", /* = 32 */    KLV_TYPE_DOUBLE, CONVERT_ANGLE(-0.075), CONVERT_ANGLE(0.075) },
    { "KLV_UAS_CORNER4_LON_OFFSET", /* = 33 */    KLV_TYPE_DOUBLE, CONVERT_ANGLE(-0.075), CONVERT_ANGLE(0.075) },
    { "KLV_UAS_ICING", /* = 34 */                 KLV_TYPE_UINT },
    { "KLV_UAS_WIND_DIRECTION", /* = 35 */        KLV_TYPE_DOUBLE, CONVERT_ANGLE(0), CONVERT_ANGLE(360) },
    { "KLV_UAS_WIND_SPEED", /* = 36 */            KLV_TYPE_DOUBLE, 0, 100 },
    { "KLV_UAS_STATIC_PRESSURE", /* = 37 */       KLV_TYPE_DOUBLE, 0, 500000 },
    { "KLV_UAS_DENSITY_ALTITUDE", /* = 38 */      KLV_TYPE_DOUBLE, -900, 19000 },
    { "KLV_UAS_OAT", /* = 39 */                   KLV_TYPE_INT },
    { "KLV_UAS_TARGET_LAT", /* = 40 */            KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_TARGET_LON", /* = 41 */            KLV_TYPE_DOUBLE, CONVERT_ANGLE(-180), CONVERT_ANGLE(180) },
    { "KLV_UAS_TARGET_MSL", /* = 42 */            KLV_TYPE_DOUBLE, -900, 19000 },
    { "KLV_UAS_TRACK_WIDTH", /* = 43 */           KLV_TYPE_UINT },
    { "KLV_UAS_TRACK_HEIGHT", /* = 44 */          KLV_TYPE_UINT },
    { "KLV_UAS_TARGET_CE90", /* = 45 */           KLV_TYPE_UINT, 0, 4096 }, // NOTE: POORLY DEFINED BY SPEC
    { "KLV_UAS_TARGET_LE90", /* = 46 */           KLV_TYPE_UINT, 0, 4096 }, // NOTE: POORLY DEFINED BY SPEC
    { "KLV_UAS_GENERIC_FLAG", /* = 47 */          KLV_TYPE_UINT },
    { "KLV_UAS_SECURITY", /* = 48 */              KLV_TYPE_OTHER },
    { "KLV_UAS_DYNAMIC_PRESSURE", /* = 49 */      KLV_TYPE_DOUBLE, 0, 500000 },
    { "KLV_UAS_ANGLE_OF_ATTACK_SHORT", /* = 50 */ KLV_TYPE_OTHER },
    { "KLV_UAS_VERTICAL_VELOCITY", /* = 51 */     KLV_TYPE_DOUBLE, -180, 180 },
    { "KLV_UAS_SIDESLIP_SHORT", /* = 52 */        KLV_TYPE_DOUBLE, CONVERT_ANGLE(-20), CONVERT_ANGLE(20) },
    { "KLV_UAS_AIRFIELD_PRESSURE", /* = 53 */     KLV_TYPE_DOUBLE, 0, 500000 },
    { "KLV_UAS_AIRFIELD_ELEVATION", /* = 54 */    KLV_TYPE_DOUBLE, -900, 19000 },
    { "KLV_UAS_HUMIDITY", /* = 55 */              KLV_TYPE_DOUBLE, 0, 1 },
    { "KLV_UAS_GROUND_SPEED", /* = 56 */          KLV_TYPE_UINT },
    { "KLV_UAS_GROUND_RANGE", /* = 57 */          KLV_TYPE_DOUBLE, 0, 5000000 },
    { "KLV_UAS_FUEL_REMAINING", /* = 58 */        KLV_TYPE_DOUBLE, 0, 10000 },
    { "KLV_UAS_CALLSIGN", /* = 59 */              KLV_TYPE_STRING },
    { "KLV_UAS_WEAPON_LOAD", /* = 60 */           KLV_TYPE_UINT },
    { "KLV_UAS_WEAPON_FIRED", /* = 61 */          KLV_TYPE_UINT },
    { "KLV_UAS_LASER_PRF_CODE", /* = 62 */        KLV_TYPE_UINT },
    { "KLV_UAS_FOV_NAME", /* = 63 */              KLV_TYPE_STRING },
    { "KLV_UAS_MAGNETIC_HEADING", /* = 64 */      KLV_TYPE_DOUBLE, CONVERT_ANGLE(0), CONVERT_ANGLE(360) },
    { "KLV_UAS_VERSION", /* = 65 */               KLV_TYPE_UINT },
    { "KLV_UAS_TARGET_COVARIANCE", /* = 66 */     KLV_TYPE_OTHER },
    { "KLV_UAS_ALT_PLATFORM_LAT", /* = 67 */      KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_ALT_PLATFORM_LON", /* = 68 */      KLV_TYPE_DOUBLE, CONVERT_ANGLE(-180), CONVERT_ANGLE(180) },
    { "KLV_UAS_ALT_PLATFORM_MSL", /* = 69 */      KLV_TYPE_DOUBLE, -900, 19000 },
    { "KLV_UAS_ALT_PLATFORM_NAME", /* = 70 */     KLV_TYPE_STRING },
    { "KLV_UAS_ALT_PLATFORM_HEADING", /* = 71 */  KLV_TYPE_DOUBLE, CONVERT_ANGLE(0), CONVERT_ANGLE(360) },
    { "KLV_UAS_EVENT_START_TIME", /* = 72 */      KLV_TYPE_UINT },
    { "KLV_UAS_RVT_LOCAL_SET", /* = 73 */         KLV_TYPE_OTHER },
    { "KLV_UAS_VMTI", /* = 74 */                  KLV_TYPE_OTHER },
    { "KLV_UAS_SENSOR_HAE", /* = 75 */            KLV_TYPE_DOUBLE, -900, 19000 },
    { "KLV_UAS_ALT_PLATFORM_HAE", /* = 76 */      KLV_TYPE_DOUBLE, -900, 19000 },
    { "KLV_UAS_OPERATIONAL_MODE", /* = 77 */      KLV_TYPE_UINT },
    { "KLV_UAS_IMAGE_HAE", /* = 78 */             KLV_TYPE_DOUBLE, -900, 19000 },
    { "KLV_UAS_NORTH_VELOCITY", /* = 79 */        KLV_TYPE_DOUBLE, -327, 327 },
    { "KLV_UAS_EAST_VELOCITY", /* = 80 */         KLV_TYPE_DOUBLE, -327, 327 },
    { "KLV_UAS_IMAGE_HORIZON", /* = 81 */         KLV_TYPE_OTHER },
    { "KLV_UAS_CORNER1_LAT", /* = 82 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_CORNER1_LON", /* = 83 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(-180), CONVERT_ANGLE(180) },
    { "KLV_UAS_CORNER2_LAT", /* = 84 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_CORNER2_LON", /* = 85 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(-180), CONVERT_ANGLE(180) },
    { "KLV_UAS_CORNER3_LAT", /* = 86 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_CORNER3_LON", /* = 87 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(-180), CONVERT_ANGLE(180) },
    { "KLV_UAS_CORNER4_LAT", /* = 88 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_CORNER4_LON", /* = 89 */           KLV_TYPE_DOUBLE, CONVERT_ANGLE(-180), CONVERT_ANGLE(180) },
    { "KLV_UAS_PLATFORM_PITCH", /* = 90 */        KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_PLATFORM_ROLL", /* = 91 */         KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_ANGLE_OF_ATTACK", /* = 92 */       KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_SIDESLIP", /* = 93 */              KLV_TYPE_DOUBLE, CONVERT_ANGLE(-90), CONVERT_ANGLE(90) },
    { "KLV_UAS_CORE_ID", /* = 94 */               KLV_TYPE_OTHER },
};

KlvTagInfo_t SecurityTagInfo[KLV_SECURITY_NUM_ELEMENTS] = {
    { "KLV_SECURITY_NULL", /* = 0 */                            KLV_TYPE_OTHER },
    { "KLV_SECURITY_CLASSIFICATION", /* = 1 */                  KLV_TYPE_OTHER },
    { "KLV_SECURITY_CLASSIFYING_CODING", /* = 2 */              KLV_TYPE_OTHER },
    { "KLV_SECURITY_CLASSIFYING_COUNTRY", /* = 3 */             KLV_TYPE_STRING },
    { "KLV_SECURITY_HANDLING_INSTRUCTIONS", /* = 4 */           KLV_TYPE_STRING },
    { "KLV_SECURITY_CAVEATS", /* = 5 */                         KLV_TYPE_STRING },
    { "KLV_SECURITY_RELEASING_INSTRUCTIONS", /* = 6 */          KLV_TYPE_STRING },
    { "KLV_SECURITY_CLASSIFIED_BY", /* = 7 */                   KLV_TYPE_STRING },
    { "KLV_SECURITY_DERIVED_FROM", /* = 8 */                    KLV_TYPE_STRING },
    { "KLV_SECURITY_CLASSIFICATION_REASON", /* = 9 */           KLV_TYPE_STRING },
    { "KLV_SECURITY_DECLASSIFICATION_DATE", /* = 10 */          KLV_TYPE_STRING },
    { "KLV_SECURITY_CLASSIFICATION_MARKING", /* = 11 */         KLV_TYPE_STRING },
    { "KLV_SECURITY_OBJECT_COUNTRY_CODING", /* = 12 */          KLV_TYPE_OTHER },
    { "KLV_SECURITY_OBJECT_COUNTRY", /* = 13 */                 KLV_TYPE_OTHER }, //UTF16
    { "KLV_SECURITY_CLASSIFICATION_COMMENTS", /* = 14 */        KLV_TYPE_STRING },
    { "KLV_SECURITY_NULL1", /* = 15 */                          KLV_TYPE_OTHER },
    { "KLV_SECURITY_NULL2", /* = 16 */                          KLV_TYPE_OTHER },
    { "KLV_SECURITY_NULL3", /* = 17 */                          KLV_TYPE_OTHER },
    { "KLV_SECURITY_NULL4", /* = 18 */                          KLV_TYPE_OTHER },
    { "KLV_SECURITY_NULL5", /* = 19 */                          KLV_TYPE_OTHER },
    { "KLV_SECURITY_NULL6", /* = 20 */                          KLV_TYPE_OTHER },
    { "KLV_SECURITY_NULL7", /* = 21 */                          KLV_TYPE_OTHER },
    { "KLV_SECURITY_VERSION", /* = 22 */                        KLV_TYPE_UINT },
    { "KLV_SECURITY_CLASSIFYING_CODING_VERSION", /* = 23 */     KLV_TYPE_STRING },
    { "KLV_SECURITY_OBJECT_COUNTRY_CODING_VERSION", /* = 24 */  KLV_TYPE_STRING },
};

static uint64_t KlvGetLength(const uint8_t *pData, uint64_t *pIndex)
{
    if (pData[*pIndex] & 0x80)
    {
        int Bytes = pData[(*pIndex)++] & 0x7F;
        uint64_t Length = 0;

        while (Bytes--)
            Length |= (uint64_t)pData[(*pIndex)++] << (Bytes * 8);

        return Length;
    }
    else
        return pData[(*pIndex)++];
}

void KlvNewData(const uint8_t *pData, int Length)
{
    // Get the length of the whole shebang
    uint64_t i = 16, DataLength = KlvGetLength(pData, &i) + i;

    // As long as there's more data to read out
    while (i < DataLength)
    {
        // Grab the key and the length of the tag's data
        uint8_t Key = pData[i++];
        uint64_t KeyLength = KlvGetLength(pData, &i);

        // Clip the length to the number of bytes remaining
        KeyLength = MIN(KeyLength, DataLength - i);

        // Pass the data to the KLV tree
        KlvTreeSetValue(Key, KeyLength, &pData[i]);

        // Now increment the array index by the data size
        i += KeyLength;
    }

}// KlvNewData


double KlvGetValueDouble(KlvUasDataElement_t Element, int *pResult)
{
    // If this is a valid key
    if (Element < KLV_UAS_NUM_ELEMENTS)
    {
        // Decide what to do based on the data type
        switch (TagInfo[Element].Type)
        {
        case KLV_TYPE_DOUBLE:
            return KlvTreeGetValueDouble(Element, TagInfo[Element].Min, TagInfo[Element].Max, pResult);

        case KLV_TYPE_UINT:
            return KlvTreeGetValueUInt(Element, pResult);

        case KLV_TYPE_INT:
            return KlvTreeGetValueInt(Element, pResult);

        default:
            break;
        }
    }

    *pResult = 0;
    return 0;

}// KlvGetValueDouble

int64_t KlvGetValueInt(KlvUasDataElement_t Element, int *pResult)
{
    // If this is a valid key
    if (Element < KLV_UAS_NUM_ELEMENTS)
    {
        // Decide what to do based on the data type
        switch (TagInfo[Element].Type)
        {
        case KLV_TYPE_DOUBLE:
        {
            double Value = KlvTreeGetValueDouble(Element, TagInfo[Element].Min, TagInfo[Element].Max, pResult);

            // Treat the data as invalid if it doesn't fit in an int64_t
            if ((Value > (double)0x7FFFFFFFFFFFFFFFLL) || (Value < (double)0x8000000000000000LL))
                break;
            else if (Value >= 0)
                return Value + 0.5f;
            else
                return Value - 0.5f;
        }

        case KLV_TYPE_UINT:
        {
            uint64_t Value = KlvTreeGetValueUInt(Element, pResult);

            // Treat the data as invalid if it doesn't fit in an int64_t
            if (Value > 0x7FFFFFFFFFFFFFFFULL)
                break;
            else
                return Value;
        }

        case KLV_TYPE_INT:
            return KlvTreeGetValueInt(Element, pResult);

        default:
            break;
        }
    }

    *pResult = 0;
    return 0;

}// KlvGetValueInt

uint64_t KlvGetValueUInt(KlvUasDataElement_t Element, int *pResult)
{
    // If this is a valid key
    if (Element < KLV_UAS_NUM_ELEMENTS)
    {
        // Decide what to do based on the data type
        switch (TagInfo[Element].Type)
        {
        case KLV_TYPE_DOUBLE:
        {
            double Value = KlvTreeGetValueDouble(Element, TagInfo[Element].Min, TagInfo[Element].Max, pResult);

            // Treat the data as invalid if it doesn't fit in a uint64_t
            if ((Value > (double)0xFFFFFFFFFFFFFFFFULL) || (Value < 0))
                break;
            else if (Value > 0)
                return Value + 0.5f;
        }

        case KLV_TYPE_UINT:
            return KlvTreeGetValueUInt(Element, pResult);

        case KLV_TYPE_INT:
        {
            int64_t Value = KlvTreeGetValueInt(Element, pResult);

            // Treat the data as invalid if it doesn't fit in a uint64_t
            if (Value < 0)
                break;
            else
                return Value;
        }

        default:
            break;
        }
    }

    *pResult = 0;
    return 0;

}// KlvGetValueUInt

const char *KlvGetValueString(KlvUasDataElement_t Element)
{
    const char *pValue = 0;

    // If this is a valid key
    if ((Element < KLV_UAS_NUM_ELEMENTS) && (TagInfo[Element].Type == KLV_TYPE_STRING))
        pValue = KlvTreeGetValueString(Element);

    return pValue;

}// KlvGetValueString

const char *KlvGetSecurityValueString(KlvSecurityDataElement_t Element)
{
    const char *pValue = 0;

    // If this is a valid key
    if ((Element < KLV_SECURITY_NUM_ELEMENTS) && (SecurityTagInfo[Element].Type == KLV_TYPE_STRING))
        pValue = KlvTreeGetValueString(Element+KLV_UAS_NUM_ELEMENTS);

    return pValue;

}// KlvGetSecurityValueString

void KlvPrintData(void)
{
    int Result, i;
    uint32_t Length;

    // Loop through all of the different tags we know about
    for (i = 0; i < KLV_UAS_NUM_ELEMENTS; i++)
    {
        // If we found one of these elements
        if (KlvTreeHasKey(i))
        {
            // Grab the length of this tag
            KlvTreeGetValue(i, &Length);

            // Print the key, enumeration name, and length
            printf("Key %3d (%-32s), length: %d, value: ", i, TagInfo[i].Name, Length);

            if(i == 48) {
                uint32_t Length;
                const uint8_t *pData = KlvTreeGetValue((KlvUasDataElement_t)i, &Length);
                KlvPrintSecurityData(pData, Length); continue; 
            }

            // Switch on tag type
            switch (TagInfo[i].Type)
            {
            // Decode this tag as a double and print
            case KLV_TYPE_DOUBLE:
            {
                //check for special Cases
                uint32_t Length, j;
                const uint8_t *pData = KlvTreeGetValue((KlvUasDataElement_t)i, &Length);
                // for (j = 0; j < Length; j++)
                //     printf("(%02x) ", pData[j]);
                if (Length == 2 && (pData[0] == 0x80 && pData[1] == 00)) {
                    printf("KLV_INVALID_0x8000\n");
                } else if (Length == 4 && (pData[0]==0x80 && pData[1] == 0x00 && pData[2] == 0x00 && pData[3] == 0x00)) {
                    printf("KLV_INVALD_0x80000000\n");
                } else {
                    printf("%lf\n", KlvTreeGetValueDouble((KlvUasDataElement_t)i, TagInfo[i].Min, TagInfo[i].Max, &Result));
                }
                break;
            }
            // Decode this tag as an unsigned int and print
            case KLV_TYPE_UINT:
                printf("%" PRIu64 "\n", KlvTreeGetValueUInt((KlvUasDataElement_t)i, &Result));
                break;

            // Decode this tag as a signed int and print
            case KLV_TYPE_INT:
                printf("%" PRId64 "\n", KlvTreeGetValueInt((KlvUasDataElement_t)i, &Result));
                break;

            // Decode this tag as a string and print
            case KLV_TYPE_STRING:
                printf("%s\n", KlvGetValueString((KlvUasDataElement_t)i));
                break;

            // Print anything else as a hexdump
            default:
            {
                uint32_t Length, j;
                const uint8_t *pData = KlvTreeGetValue((KlvUasDataElement_t)i, &Length);

                // 
                for (j = 0; j < Length; j++)
                    printf("%02x ", pData[j]);
                printf("\n");
                break;
            }
            };
        }
    }

}// KlvPrintData

void KlvPrintSecurityData(const uint8_t *pData, int passedLength)
{
    //presumably we are coming from KlvPrintData so add a newLine
    printf("\n");
    int Result, i;
    uint32_t Length;

    // Get the length of the whole shebang
    uint64_t k = 0, DataLength = passedLength;

    // As long as there's more data to read out
    while (k < DataLength)
    {
        // Grab the key and the length of the tag's data
        uint8_t Key = pData[k++] + KLV_UAS_NUM_ELEMENTS; //Offset the key by the KLV_UAS count. as duplicate key entries exist
        uint64_t KeyLength = KlvGetLength(pData, &k);

        // Clip the length to the number of bytes remaining
        KeyLength = MIN(KeyLength, DataLength - k);

        // Pass the data to the KLV tree
        KlvTreeSetValue(Key, KeyLength, &pData[k]);

        // Now increment the array index by the data size
        k += KeyLength;
    }

    // Loop through all of the different tags we know about
    for (i = 0; i < KLV_SECURITY_NUM_ELEMENTS; i++)
    {
        // If we found one of these elements
        if (KlvTreeHasKey(i+KLV_UAS_NUM_ELEMENTS))
        {
            // Grab the length of this tag
            KlvTreeGetValue(i+KLV_UAS_NUM_ELEMENTS, &Length);

            // Print the key, enumeration name, and length
            printf("\tKey %3d (%-45s), length: %d, value: ", i, SecurityTagInfo[i].Name, Length);

            // Switch on tag type
            switch (SecurityTagInfo[i].Type)
            {
            // Decode this tag as a double and print
            case KLV_TYPE_DOUBLE:
            {
                //check for special Cases
                uint32_t Length, j;
                const uint8_t *pData = KlvTreeGetValue((KlvSecurityDataElement_t)i+KLV_UAS_NUM_ELEMENTS, &Length);
                // for (j = 0; j < Length; j++)
                //     printf("(%02x) ", pData[j]);
                if (Length == 2 && (pData[0] == 0x80 && pData[1] == 00)) {
                    printf("KLV_INVALID_0x8000\n");
                } else if (Length == 4 && (pData[0]==0x80 && pData[1] == 0x00 && pData[2] == 0x00 && pData[3] == 0x00)) {
                    printf("KLV_INVALD_0x80000000\n");
                } else {
                    printf("%lf\n", KlvTreeGetValueDouble((KlvSecurityDataElement_t)i+KLV_UAS_NUM_ELEMENTS, SecurityTagInfo[i].Min, SecurityTagInfo[i].Max, &Result));
                }
                break;
            }
            // Decode this tag as an unsigned int and print
            case KLV_TYPE_UINT:
                printf("%" PRIu64 "\n", KlvTreeGetValueUInt((KlvSecurityDataElement_t)i+KLV_UAS_NUM_ELEMENTS, &Result));
                break;

            // Decode this tag as a signed int and print
            case KLV_TYPE_INT:
                printf("%" PRId64 "\n", KlvTreeGetValueInt((KlvSecurityDataElement_t)i+KLV_UAS_NUM_ELEMENTS, &Result));
                break;

            // Decode this tag as a string and print
            case KLV_TYPE_STRING:
                printf("%s\n", KlvGetSecurityValueString((KlvSecurityDataElement_t)i));
                break;

            // Print anything else as a hexdump
            default:
            {
                uint32_t Length, j;
                const uint8_t *pData = KlvTreeGetValue((KlvSecurityDataElement_t)i+KLV_UAS_NUM_ELEMENTS, &Length);

                // 
                for (j = 0; j < Length; j++)
                    printf("%02x ", pData[j]);
                printf("\n");
                break;
            }
            };
        }
    }
}
