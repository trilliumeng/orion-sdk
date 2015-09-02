#include "OrionPacket.h"
#include <string.h>
#include <stdio.h>

// Incoming and outgoing packet structures. Incoming structure *MUST* be persistent
//  between calls to ProcessData.
static OrionPkt_t PktIn, PktOut;

// Process a chunk of incoming data from a bytestream
static void ProcessData(const UInt8 *pData, UInt32 Length);

int main(int argc, char **argv)
{
    float Pos[2] = { 135.0f * PIf / 180.0f, -30.0f * PIf / 180.0f };
    float Vel[2] = { 0, 0 };
    UInt8 Buffer[64];

    // IMPORTANT: Zero out the packet structure before using for decoding
    memset(&PktIn, 0, sizeof(PktIn));

    // This is how you form a packet
    FormOrionEncoderData(&PktOut, Pos, Vel);

    // For now, I fake sending it by copying it straight into the input buffer
    memcpy(Buffer, &PktOut, PktOut.Length + ORION_PKT_OVERHEAD);

    // Process the "incoming data"
    ProcessData(Buffer, PktOut.Length + ORION_PKT_OVERHEAD);

    // Print a trailing line feed so the prompt doesn't overwrite the data
    printf("\n");

    // Done
    return 0;

}// main

void ProcessData(const UInt8  *pData, UInt32 Length)
{
    UInt32 i;

    // For each byte in the receive buffer
    for (i = 0; i < Length; i++)
    {
        // If this byte terminates a valid packet
        if (LookForOrionPacketInByte(&PktIn, pData[i]))
        {
            // Decide what to do based on packet ID
            switch (PktIn.ID)
            {
            // Encoder position/velocity report
            case ORION_PKT_ENCODER_DATA:
            {
                float Pos[NUM_GIMBAL_AXES];
                float Vel[NUM_GIMBAL_AXES];

                // If this encoder data packet contains valid data
                if (DecodeOrionEncoderData(&PktIn, Pos, Vel))
                {
                    // Convert the encoder positions to degrees and print them out
                    printf("Pan: %6.1f, Tilt: %6.1f\r",
                           Pos[GIMBAL_AXIS_PAN]  * 180.0f / PIf,
                           Pos[GIMBAL_AXIS_TILT] * 180.0f / PIf);
                }
                break;
            }

            // Other packets: TBD
            default:
                break;
            };
        }
    }

}// ProcessData
