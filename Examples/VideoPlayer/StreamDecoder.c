#include "StreamDecoder.h"
#include "Constants.h"
#include "FFmpeg.h"

#include <string.h>

static AVFormatContext *pFormatContext = NULL;
static AVCodecContext *pCodecContext = NULL;
static AVCodec *pCodec = NULL;
static AVFrame *pFrame = NULL;
static AVFrame *pFrameCopy = NULL;
static AVPacket Packet;

static uint8_t *pMetaData = NULL;
static uint64_t MetaDataBufferSize = 0;
static uint64_t MetaDataSize = 0;
static uint64_t MetaDataBytes = 0;

static int VideoStream = 0;
static int DataStream = 0;

static void StreamCleanup(void);

int StreamOpen(const char *pUrl)
{
    // FFmpeg startup stuff
    avcodec_register_all();
    av_register_all();
    avformat_network_init();

    // Allocate a new format context
    pFormatContext = avformat_alloc_context();

    // Have avformat_open_input timeout after 5s
    AVDictionary *pOptions = 0;
    av_dict_set(&pOptions, "timeout", "5000000", 0);

    // If the stream doesn't open
    if (avformat_open_input(&pFormatContext, pUrl, NULL, &pOptions) < 0)
    {
        // Clean up the allocated resources (if any...) and exit with a failure code
        StreamCleanup();
        return 0;
    }

    // Dump the transport stream data
    av_dump_format(pFormatContext, 0, pUrl, 0);
    fflush(stdout);

    // If there don't appear to be an valid streams in the transport stream
    if (pFormatContext->nb_streams == 0)
    {
        // Clean up the allocated resources (if any...) and exit with a failure code
        StreamCleanup();
        return 0;
    }

    // Get the stream indices for video and metadata
    VideoStream = av_find_best_stream(pFormatContext, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    DataStream  = av_find_best_stream(pFormatContext, AVMEDIA_TYPE_DATA,  -1, -1, NULL, 0);

    // Set the format context to playing
    av_read_play(pFormatContext);

    // Get a codec pointer based on the video stream's codec ID and allocate a context
    pCodec = avcodec_find_decoder(pFormatContext->streams[VideoStream]->codec->codec_id);
    pCodecContext = avcodec_alloc_context3(pCodec);

    // Open the newly allocated codec context
    avcodec_open2(pCodecContext, pCodec, NULL);

    // Allocate the decode and output frame structures
    pFrame = av_frame_alloc();
    pFrameCopy = av_frame_alloc();

    // Finally, initialize the AVPacket structure
    av_init_packet(&Packet);
    Packet.data = NULL;
    Packet.size = 0;

    // Done - return 1 to indicate success
    return 1;

}// StreamOpen

static void StreamCleanup(void)
{
    // Free the AVFrames
    av_frame_free(&pFrame);
    av_frame_free(&pFrameCopy);

    // If we allocated a codec context
    if (pCodecContext)
    {
        // Close it then free it
        avcodec_close(pCodecContext);
        av_free(pCodecContext);
    }

    // If we allocated a format context
    if (pFormatContext)
    {
        // Pause, close and free it
        av_read_pause(pFormatContext);
        avformat_close_input(&pFormatContext);
        avformat_free_context(pFormatContext);
    }

    // Nullify all the pointers we just messed with
    pCodecContext  = NULL;
    pFrame         = NULL;
    pFormatContext = NULL;

}// StreamCleanup

int StreamProcess(void)
{
    int NewVideo = 0, NewMetaData = 0;

    // As long as we can keep reading packets from the UDP socket
    while (av_read_frame(pFormatContext, &Packet) >= 0)
    {
        // If this packet belongs to the video stream
        if (Packet.stream_index == VideoStream)
        {
            // Pass it to the h.264 decoder
            avcodec_decode_video2(pCodecContext, pFrame, &NewVideo, &Packet);

            // If this packet finished a video frame
            if (NewVideo)
            {
                // If the incoming frame size doesn't match the output frame
                if ((pFrame->width  != pFrameCopy->width) ||
                    (pFrame->height != pFrameCopy->height))
                {
                    // Free the output frame and allocate a new one
                    av_frame_free(&pFrameCopy);
                    pFrameCopy = av_frame_alloc();

                    // Copy all the metadata (width, height, etc.) over
                    //   NOTE: av_frame_copy_props *should* do this but doesn't seem to...
                    memcpy(pFrameCopy, pFrame, sizeof(AVFrame));

                    // Now allocate a data buffer associated with this frame
                    av_frame_get_buffer(pFrameCopy, 0);
                }

                // Copy the image data from the decoder frame to the output frame
                av_frame_copy(pFrameCopy, pFrame);

                // Finally, unref the frame and free the packet
                av_frame_unref(pFrame);
            }
        }
        // If this is the KLV stream data
        else if (Packet.stream_index == DataStream)
        {
            // If we have a full metadata packet in memory, zero out the size and index
            if (MetaDataBytes == MetaDataSize)
                MetaDataBytes = MetaDataSize = 0;

            // If we don't have any metadata buffered up yet and this packet is big enough for a US key and size
            if ((MetaDataBytes == 0) && (Packet.size > 17))
            {
                // UAS LS universal key
                static const uint8_t KlvHeader[16] = {
                    0x06, 0x0E, 0x2B, 0x34, 0x02, 0x0B, 0x01, 0x01,
                    0x0E, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00
                };

                // Try finding the KLV header in this packet
                const uint8_t *pStart = memmem(Packet.data, Packet.size, KlvHeader, 16);
                const uint8_t *pSize = pStart + 16;

                // If we found the header and the size tag is contained in this packet
                if ((pStart != 0) && ((pSize - Packet.data) < Packet.size))
                {
                    // Initialize the header size to US key + 1 size byte and zero KLV tag bytes
                    uint64_t KlvSize = 0, HeaderSize = 17;

                    // If the size is a multi-byte BER-OID size
                    if (pSize[0] & 0x80)
                    {
                        // Get the size of the size (up to )
                        int Bytes = pSize[0] & 0x07, i;

                        // If the entire size field is contained in this packet
                        if (&pSize[Bytes] < &Packet.data[Packet.size])
                        {
                            // Build the size up from the individual bytes
                            for (i = 0; i < Bytes; i++)
                                KlvSize = (KlvSize << 8) | pSize[i + 1];
                        }

                        // Add the additional size bytes to the header size
                        HeaderSize += Bytes;
                    }
                    // Otherwise, just read the size byte straight out of byte 16
                    else
                        KlvSize = pSize[0];

                    // If we got a valid local set size
                    if (KlvSize > 0)
                    {
                        // Compute the maximum bytes to copy out of the packet
                        int MaxBytes = Packet.size - (pStart - Packet.data);
                        int TotalSize = HeaderSize + KlvSize;
                        int BytesToCopy = MIN(MaxBytes, TotalSize);

                        // If our local buffer is too small for the incoming data
                        if (MetaDataBufferSize < TotalSize)
                        {
                            // Reallocate enough space and store the new buffer size
                            pMetaData = (uint8_t *)realloc(pMetaData, TotalSize);
                            MetaDataBufferSize = TotalSize;
                        }

                        // Now copy the new data into the start of the local buffer
                        memcpy(pMetaData, pStart, BytesToCopy);
                        MetaDataSize = TotalSize;
                        MetaDataBytes = BytesToCopy;
                    }
                }
            }
            // Otherwise, if we're mid-packet
            else if (MetaDataBytes < MetaDataSize)
            {
                // Figure out the number of bytes to copy out of this particular packet
                int BytesToCopy = MIN(Packet.size, MetaDataSize - MetaDataBytes);

                // Copy into the local buffer in the right spot and increment the index
                memcpy(&pMetaData[MetaDataBytes], Packet.data, BytesToCopy);
                MetaDataBytes += BytesToCopy;
            }

            // There's new metadata if the size is non-zero and equal to the number of bytes read in
            NewMetaData = (MetaDataSize != 0) && (MetaDataBytes == MetaDataSize);
        }

        // Free the packet data
        av_free_packet(&Packet);

        // Return 1 if both a video frame and KLV packet have been read in
        if (NewVideo && NewMetaData)
            return 1;
    }

    // No new data if we made it here
    return 0;

}// StreamProcess

int StreamGetVideoFrame(uint8_t *pFrameData, int *pWidth, int *pHeight, int MaxBytes)
{
    // If we actually have frame data to copy out and it won't overrun pFrameData
    if ((pFrameCopy->width > 0) && (pFrameCopy->height > 0) && ((pFrame->width * pFrame->height * 3) < MaxBytes))
    {
        // Allocate a context for colorspace conversion and do some data marshaling 
        struct SwsContext *pContext  = sws_getContext(pFrameCopy->width, pFrameCopy->height, pFrameCopy->format,
                                                      pFrameCopy->width, pFrameCopy->height, AV_PIX_FMT_RGB24,
                                                      SWS_FAST_BILINEAR, NULL, NULL, NULL);
        uint8_t *pData[3] = { pFrameData, NULL, NULL };
        int Stride[3] = { pFrameCopy->width * 3, 0, 0 };

        // Copy the frame width/height out to the caller
        *pWidth  = pFrameCopy->width;
        *pHeight = pFrameCopy->height;

        // Now do the actual colorspace conversion
        sws_scale(pContext, (const uint8_t **)pFrameCopy->data, pFrameCopy->linesize, 0, pFrameCopy->height, pData, Stride);
        sws_freeContext(pContext);

        // Tell the caller this function succeeded
        return 1;
    }

    // No such data
    return 0;

}// StreamGetVideoFrame

int StreamGetMetaData(uint8_t *pData, int *pBytes, int MaxBytes)
{
    // If there's a valid metadata buffer and we're not going to overrun pData
    if ((pMetaData != 0) && (MetaDataBytes < MaxBytes))
    {
        // Copy the buffered KLV data into pData and send out the buffer size
        memcpy(pData, pMetaData, MetaDataBytes);
        *pBytes = MetaDataBytes;

        // A return value of 1 signifies that that pData contains a whole KLV UAS data packet
        return 1;
    }

    // Either the caller needs to allocate more space or we haven't seen any metadata yet
    return 0;

}// StreamGetMetaData

