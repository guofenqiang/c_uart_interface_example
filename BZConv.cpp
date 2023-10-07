#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "CRCCheck.h"
#include "BZConv.h"

/*
UAV=Protocol2
Ground=Protocol1
*/
/*My defines*/
/*up: UAV<-Ground*/
#define BZ_UAV_UP_LEN 64
#define BZ_GROUND_UP_LEN 53

/*down: UAV->Ground*/
#define BZ_UAV_DOWN_LEN 128
#define BZ_GROUND_DOWN_LEN 115

/*Protocol2 -> Protocol1*/
BZ_STATUS BZ_UAV2Ground(UINT8 *grdBuffer, const UINT8 *UAVBuffer)
{
    UINT8 crcL, crcH;
    UINT16 crc;
    if (!UAVBuffer || !grdBuffer)
        return BZ_INVALID_ADDRESS;
    crc = GetCRC16((UINT8 *)UAVBuffer, BZ_UAV_DOWN_LEN - 3);
    crcL = crc & 0x00FF;
    crcH = crc >> 8;
    if (UAVBuffer[BZ_UAV_DOWN_LEN - 3] != crcL || UAVBuffer[BZ_UAV_DOWN_LEN - 2] != crcH)
        return BZ_CRC_UNPASSED;
    memcpy(grdBuffer, UAVBuffer, BZ_GROUND_DOWN_LEN - 3);
    crc = GetCRC16(grdBuffer, BZ_GROUND_DOWN_LEN - 3);
    grdBuffer[BZ_GROUND_DOWN_LEN - 3] = crc & 0x00FF;
    grdBuffer[BZ_GROUND_DOWN_LEN - 2] = crc >> 8;
    grdBuffer[BZ_GROUND_DOWN_LEN - 1] = UAVBuffer[BZ_UAV_DOWN_LEN - 1];
    return BZ_OK;
}

/*Protocol1 -> Protocol2*/
BZ_STATUS BZ_Ground2UAV(UINT8 *UAVBuffer, const UINT8 *grdBuffer)
{
//    BZ_STATUS ret;
    UINT8 crcL, crcH;
    UINT16 crc;
    if (!UAVBuffer || !grdBuffer)
        return BZ_INVALID_ADDRESS;
    crc = GetCRC16((UINT8 *)grdBuffer, BZ_GROUND_UP_LEN - 3);
    crcL = crc & 0x00FF;
    crcH = crc >> 8;
    if (grdBuffer[BZ_GROUND_UP_LEN - 3] != crcL || grdBuffer[BZ_GROUND_UP_LEN - 2] != crcH)
        return BZ_CRC_UNPASSED;
    memcpy(UAVBuffer, grdBuffer, BZ_GROUND_UP_LEN - 3);
    memset(UAVBuffer + BZ_GROUND_UP_LEN - 3, 0, BZ_UAV_UP_LEN - BZ_GROUND_UP_LEN);
    crc = GetCRC16(UAVBuffer, BZ_UAV_UP_LEN - 3);
    UAVBuffer[BZ_UAV_UP_LEN - 3] = crc & 0x00FF;
    UAVBuffer[BZ_UAV_UP_LEN - 2] = crc >> 8;
    UAVBuffer[BZ_UAV_UP_LEN - 1] = grdBuffer[BZ_GROUND_UP_LEN - 1];
    return BZ_OK;
}
