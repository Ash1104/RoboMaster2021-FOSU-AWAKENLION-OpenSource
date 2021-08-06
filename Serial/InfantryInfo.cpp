#include "InfantryInfo.h"

void pc_data_handler(u8 *p_frame, send_pc_t &pc_send_mesg)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  u8 data_length = p_frame[1];//10

  unsigned char cmd_id = p_frame[5] ;//17

  u8 *data_addr = p_frame + HEADER_LEN + CMD_LEN-1; //-1

  //taskENTER_CRITICAL();

  switch (cmd_id)
  {
  case SENTRY_DATA_ID:
    memcpy(&pc_send_mesg.stm32_info_data, data_addr, data_length);
    break;
  }
  return;
}
