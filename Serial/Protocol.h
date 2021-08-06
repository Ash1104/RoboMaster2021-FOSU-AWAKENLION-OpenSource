//-----------------------------------【协议】--------------------------------------------
// brief：串口相关协议，包括crc8和crc16校验
//------------------------------------------------------------------------------------------------

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "Settings/Settings.h"

#define UP_REG_ID    0xA0  //up layer regional id
#define DN_REG_ID    0xA5  //down layer regional id

#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

#undef NULL
#if defined(__cplusplus)
#define NULL 0
#else
#define NULL ((void *)0)
#endif

#define HEARDER_LED sizeof(frame_header_t)
#define CMD_LEN 2
#define CRC_LEN  2

/** 
  * @brief  frame header structure definition
  */
typedef void * osMutexId;

typedef struct
{
  u8  sof;
  u16 data_length;
  u8  seq;
  u8  crc8;
} frame_header_t;

#define HEADER_LEN   sizeof(frame_header_t)

#define ASSERT(x) do {while(!(x));} while(0)

u8 verify_crc8_check_sum(u8* pchMessage, u16 dwLength);
u8 verify_crc16_check_sum(u8* pchMessage, u32 dwLength);

void append_crc8_check_sum(u8* pchMessage, u16 dwLength);
void append_crc16_check_sum(u8* pchMessage, u32 dwLength);

#endif
