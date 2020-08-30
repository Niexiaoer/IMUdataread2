#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GX5-25.h"

#define STIM_IDEN_BYTE1 0xAF
#define STIM_INIDATA_SIZE 63
#define STIM_ADDDATA_SIZE 8
#define STIM_DATA_SIZE STIM_INIDATA_SIZE+STIM_ADDDATA_SIZE

typedef struct _STIM300_timestamp
{
	u32 second;
	u16 msecond;
}STIM300_timestamp;

typedef struct _STIM300_packet
{
	struct _STIM300_timestamp STIM300_timestamp;
}STIM300_packet;

static u32 table[256];
u32 bitrev(u32 input, int bw);
void crc32_init(u32 poly);
int STIM_is_checksum_valid(u8*, int);
void STIM300_packet_callback(STIM300_packet*, u8 *, u16);
#ifdef __cplusplus
}
#endif