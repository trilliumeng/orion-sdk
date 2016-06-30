#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "LineOfSight.h"
#include "GeolocateTelemetry.h"
#include "LinuxComm.h"

// #define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT printf
#else
# define DEBUG_PRINT(...)
#endif

static int TriangleContainsPoint(const double A[NLLA], const double B[NLLA], const double C[NLLA], double P[NLLA]);
static int GetTile(const Tile_t *pTile);
static float GetElevation(double TargetLat, double TargetLon);
static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv, int *pLevel);

static OrionPkt_t PktIn, PktOut;
static Vertices_t Vertices;
static Triangles_t Triangles;
static Tile_t CurrentTile = { -1, -1, -1 };
static int CommHandle = -1;

// Elevation tile level of detail - defaults to 15
static int TileLevel = 15;

int main(int argc, char **argv)
{
    // Process the command line arguments
    ProcessArgs(argc, argv, &TileLevel);

    // If we don't have a valid handle yet (i.e. no serial port)
    if (CommHandle < 0)
    {
        // Try to find the gimbal on the network
        CommHandle = LinuxCommOpenNetwork();
    }

    // If we STILL don't have a valid handle
    if (CommHandle < 0)
    {
        // Kill the whole app right now
        KillProcess("Failed to connect to gimbal", -1);
    }

    // Loop forever
    while (1)
    {
        GeolocateTelemetry_t Geo;

        // If we get a geolocate telemetry packet from the gimbal
        if (LinuxCommReceive(CommHandle, &PktIn) && DecodeGeolocateTelemetry(&PktIn, &Geo))
        {
            double TargetLla[NLLA], Range;
            static int Iteration = 0;

            // Throttle lookups back to once every 10 geolocate updates
            if (++Iteration >= 10)
            {
                // Try finding an intersection with the WGS-84 ellipsoid
                if (getTerrainIntersection(&Geo, GetElevation, TargetLla, &Range))
                {
                    // If we got a valid intersection, print it out
                    printf("TARGET LLA: %10.6lf %11.6lf %6.1lf, RANGE: %5.0lf\r",
                           degrees(TargetLla[LAT]),
                           degrees(TargetLla[LON]),
                           TargetLla[ALT],
                           Range);

                    // Send the computed slant range data to the gimbal
                    encodeOrionRangeDataPacket(&PktOut, Range, 1000, RANGE_SRC_OTHER);
                    LinuxCommSend(CommHandle, &PktOut);
                }
                // Otherwise, tell the user that the lookup failed for some reason
                else
                    printf("TARGET LLA: %-43s\r", "INVALID");

                // Zero the throttle timer
                Iteration = 0;
            }

            // Flush the stdout buffers to the terminal
            fflush(stdout);
        }

        // Sleep for 20 ms so as not to hog the entire CPU
        usleep(20000);
    }

    // Finally, be done!
    return 0;
}

// This function just shuts things down consistently with a nice message for the user
static void KillProcess(const char *pMessage, int Value)
{
    // Print out the error message that got us here
    printf("%s\n", pMessage);
    fflush(stdout);

    // Close down the active file descriptors
    LinuxCommClose(CommHandle);

    // Finally exit with the proper return value
    exit(Value);

}// KillProcess

static void ProcessArgs(int argc, char **argv, int *pLevel)
{
    char Error[80];

    // If there are at least two arguments, and the first looks like a serial port
    if ((argc >= 2) && (argv[1][0] == '/'))
    {
        // Try opening the specified serial port
        CommHandle = LinuxCommOpenSerial(argv[1]);

        // Now decrement the number of arguments and push the pointer up one arg
        argc--;
        argv = &argv[1];
    }

    // Use a switch with fall-through to overwrite the default geopoint
    switch (argc)
    {
    case 2: *pLevel = atoi(argv[1]);         // Tile level of detail
    case 1: break;                           // Serial port path
    // If there aren't enough arguments
    default:
        // Kill the application and print the usage info
        sprintf(Error, "USAGE: %s [/dev/ttyXXX] [LoD]", argv[0]);
        KillProcess(Error, -1);
        break;
    };

}// ProcessArgs

static int TriangleContainsPoint(const double A[NLLA], const double B[NLLA], const double C[NLLA], double P[NLLA])
{
    // Compute dot products
    double Dot00 = (C[0] - A[0]) * (C[0] - A[0]) + (C[1] - A[1]) * (C[1] - A[1]); // dot(AC, AC)
    double Dot01 = (C[0] - A[0]) * (B[0] - A[0]) + (C[1] - A[1]) * (B[1] - A[1]); // dot(AC, AB)
    double Dot02 = (C[0] - A[0]) * (P[0] - A[0]) + (C[1] - A[1]) * (P[1] - A[1]); // dot(AC, AP)
    double Dot11 = (B[0] - A[0]) * (B[0] - A[0]) + (B[1] - A[1]) * (B[1] - A[1]); // dot(AB, AB)
    double Dot12 = (B[0] - A[0]) * (P[0] - A[0]) + (B[1] - A[1]) * (P[1] - A[1]); // dot(AB, AP)

    // Compute barycentric coordinates
    double InvDenom = 1.0 / (Dot00 * Dot11 - Dot01 * Dot01);
    double u = (Dot11 * Dot02 - Dot01 * Dot12) * InvDenom;
    double v = (Dot00 * Dot12 - Dot01 * Dot02) * InvDenom;

    // If the point is inside this triangle
    if ((u >= 0) && (v >= 0) && (u + v < 1))
    {
        // Interpolate the altitude and return true
        P[ALT] = u * C[ALT] + v * B[ALT] + (1.0f - (u + v)) * A[ALT];
        return 1;
    }
    // Otherwise return false
    else
        return 0;

}// TriangleContainsPoint

static int GetTile(const Tile_t *pTile)
{
    // If the requested tile is already downloaded and read into RAM
    if ((pTile->Level == CurrentTile.Level) && (pTile->X == CurrentTile.X) && (pTile->Y == CurrentTile.Y))
    {
        // We're done here
        return 1;
    }
    else
    {
        char Cmd[64], File[64];
        FILE *pFile;

        // Pull the appropriate tile from the server and construct the file name string
        sprintf(Cmd, "./get_tile.sh %d %d %d", pTile->Level, pTile->X, pTile->Y);
        system(Cmd);
        sprintf(File, "cache/%d/%d/%d.terrain", pTile->Level, pTile->X, pTile->Y);

        // Open the terrain file
        pFile = fopen(File, "rb");

        // If the file opens successfully
        if (pFile != NULL)
        {
            double CenterLla[NLLA], Scale = PId / (1 << pTile->Level);
            int16_t U = 0, V = 0, H = 0;
            Header_t Header;

            // Delete all heap-allocated storage
            free(Triangles.pIndices);
            free(Vertices.pLla);
            free(Vertices.pH);
            free(Vertices.pV);
            free(Vertices.pU);

            // Read header structure
            fread(&Header, sizeof(Header_t), 1, pFile);

            // Convert the ECEF center point of this tile to LLA
            ecefToLLA((double *)&Header, CenterLla);

            // Set up constants for degrees-to-meters Taylor series expansion
            static const double m1 = 111132.92, m2 = -559.82, m3 = 1.175, m4 = -0.0023;
            static const double p1 = 111412.84, p2 = -93.5,   p3 = 0.118;

            // Read in the vertex count
            fread(&Vertices.Count, sizeof(uint32_t), 1, pFile);

            // Print the tile dimensions in meters, along with the number of points
            DEBUG_PRINT(" Tile width   = %.1fm\n", rad2deg(Scale) * (p1 * cos(CenterLla[LAT])) + (p2 * cos(3 * CenterLla[LAT])) + (p3 * cos(5 * CenterLla[LAT])));
            DEBUG_PRINT(" Tile height  = %.1fm\n", rad2deg(Scale) * (m1 + (m2 * cos(2 * CenterLla[LAT])) + (m3 * cos(4 * CenterLla[LAT])) + (m4 * cos(6 * CenterLla[LAT]))));
            DEBUG_PRINT(" Points       = %d\n", Vertices.Count);

            // Allocate space for vertex data (U/V/H as well as LLA)
            Vertices.pU = (uint16_t *)malloc(Vertices.Count * sizeof(uint16_t));
            Vertices.pV = (uint16_t *)malloc(Vertices.Count * sizeof(uint16_t));
            Vertices.pH = (uint16_t *)malloc(Vertices.Count * sizeof(uint16_t));
            Vertices.pLla = (double *)malloc(Vertices.Count * 3 * sizeof(double));

            // Now read the U/V/H data from the file
            fread(Vertices.pU, sizeof(uint16_t) * Vertices.Count, 1, pFile);
            fread(Vertices.pV, sizeof(uint16_t) * Vertices.Count, 1, pFile);
            fread(Vertices.pH, sizeof(uint16_t) * Vertices.Count, 1, pFile);

            // For each vertex read from the file
            for (int i = 0; i < Vertices.Count; i++)
            {
                // Decode the zigzag data and add the delta to the running raw U/V/H trackers
                U += (Vertices.pU[i] >> 1) ^ (-(Vertices.pU[i] & 1));
                V += (Vertices.pV[i] >> 1) ^ (-(Vertices.pV[i] & 1));
                H += (Vertices.pH[i] >> 1) ^ (-(Vertices.pH[i] & 1));

                // Compute and store the LLA position of this point
                Vertices.pLla[i * NLLA + LAT] = CenterLla[LAT] + (V / 32767.0f - 0.5f) * Scale;
                Vertices.pLla[i * NLLA + LON] = CenterLla[LON] + (U / 32767.0f - 0.5f) * Scale;
                Vertices.pLla[i * NLLA + ALT] = H * (Header.MaxHeight - Header.MinHeight) / 32767.0f + Header.MinHeight;
            }

            // Now read the number of triangles that follow
            fread(&Triangles.Count, sizeof(uint32_t), 1, pFile);

            // Allocate space for and read in the triangles' vertex data
            Triangles.pIndices = (uint16_t *)malloc(Triangles.Count * 3 * sizeof(uint16_t));
            fread(Triangles.pIndices, sizeof(uint16_t) * Triangles.Count * 3, 1, pFile);

            // Close the terrain file now that we've gotten all we need from it
            fclose(pFile);

            // For each vertex in each triangle
            for (int i = 0, MaxIndex = 0; i < Triangles.Count * 3; i++)
            {
                // Decode the "high-water-mark" encoded index of this vertex
                if (Triangles.pIndices[i] == 0)
                    Triangles.pIndices[i] = MaxIndex++;
                else
                    Triangles.pIndices[i] = MaxIndex - Triangles.pIndices[i];
            }

            // Save the current tile information
            CurrentTile = *pTile;

            // Tell the caller that we got the tile
            return 1;
        }
        else
        {
            // Invalidate the current tile
            CurrentTile.Level = -1;
            CurrentTile.X = -1;
            CurrentTile.Y = -1;
        }
    }

    // No data available... or something
    return 0;

}// GetTile

static float GetElevation(double TargetLat, double TargetLon)
{
    double TargetLla[NLLA] = { TargetLat, TargetLon, -10000.0 };
    double Scale = PId / (1 << TileLevel);
    int Result = 0;
    Tile_t Tile;

    // Load up the tile description structure with
    Tile.Level = TileLevel;
    Tile.X = (TargetLon + PId * 1.0f) / Scale;
    Tile.Y = (TargetLat + PId * 0.5f) / Scale;

    // If we can get the elevation tile containing this lat/lon
    if (GetTile(&Tile))
    {
        // For each triangle in the list
        for (int i = 0; i < Triangles.Count; i++)
        {
            // Get pointers to the LLA data of the three vertices in this triangle
            double *pA = &Vertices.pLla[Triangles.pIndices[i * 3 + 0] * NLLA];
            double *pB = &Vertices.pLla[Triangles.pIndices[i * 3 + 1] * NLLA];
            double *pC = &Vertices.pLla[Triangles.pIndices[i * 3 + 2] * NLLA];

            // If the point we're looking for is contained within this triangle
            if (TriangleContainsPoint(pA, pB, pC, TargetLla))
            {
                // No need to continue searching
                break;
            }
        }
    }

    // Return the elevation of this point (or -10000 on failure)
    return TargetLla[ALT];

}// GetElevation
