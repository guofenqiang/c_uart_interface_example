#ifndef __BZCONV_H__
#define __BZCONV_H__
#include <stdint.h>

typedef uint32_t BZ_STATUS;
typedef uint8_t UINT8;
typedef uint16_t UINT16;

/*Status Type*/
#define BZ_OK 0x00000000
#define BZ_INVALID_ADDRESS 0x00000001
#define BZ_CRC_UNPASSED 0x00000002

BZ_STATUS BZ_UAV2Ground(UINT8 *grdBuffer, const UINT8 *UAVBuffer);
BZ_STATUS BZ_Ground2UAV(UINT8 *UAVBuffer, const UINT8 *grdBuffer);

#endif
