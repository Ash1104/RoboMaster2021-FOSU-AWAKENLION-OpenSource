#ifndef __JUDGEMENT_INFO_H__
#define __JUDGEMENT_INFO_H__

#include "Protocol.h"
#ifdef USE_SERIAL
#define JUDGE_FIFO_BUFLEN 500

typedef enum
{
  GAME_INFO_ID       = 0x0001,
  REAL_BLOOD_DATA_ID = 0x0002,
  REAL_SHOOT_DATA_ID = 0x0003,
  REAL_POWER_DATA_ID = 0x0004,
  REAL_FIELD_DATA_ID = 0x0005,
  GAME_RESULT_ID     = 0x0006,
  GAIN_BUFF_ID       = 0x0007,
  POSITION_DATA_ID   = 0x0008,
  
  STU_CUSTOM_DATA_ID = 0x0100,
  ROBOT_TO_CLIENT_ID = 0x0101,
  CLIENT_TO_ROBOT_ID = 0x0102,
} judge_data_id_e;

typedef   struct
{
  u8 valid_flag;
  float x;
  float y;
  float z;
  float yaw;
}__attribute__((packed))  position_t;

typedef   struct
{
  u16   stage_remain_time;
  u8    game_process;
  u8    robotLevel;
  u16   remain_hp;
  u16   max_hp;
} game_robot_state_t;

typedef  struct
{
  u8 armor_type:4;
  u8 hurt_type:4;
} robot_hurt_data_t;

typedef  struct
{
  u8 bulletType;  //弹丸类型
  u8 bullet_freq;
  float   bullet_spd;
	
} real_shoot_t;

typedef  struct
{
 float chassisVolt;    //底盘输出电压
 float chassisCurrent; //底盘输出电流
 float chassisPower;   //底盘输出功率
 float chassisPowerBuffer; //地盘功率缓冲
 u16 shootHeat0;      //17mm枪口热量
 u16 shootHeat1;      //42mm枪口热量
}extPowerHeatData_t;

typedef  struct
{
  u8 card_type;
  u8 card_idx;
} rfid_detect_t;

typedef  struct
{
  u8 winner;
} game_result_t;

typedef  struct
{
	u16 buffMusk;
} get_buff_t;

typedef  struct
{
  float x;  //位置x坐标值
  float y;	//位置y坐标值
  float z;	//位置z坐标值
  float yaw; //枪口朝向角度值
} extGameRobotPos_t;

typedef  struct
{
  float data1;
  float data2;
  float data3;
  u8 mask;  //自定义数据4
} client_show_data_t;

typedef  struct
{
  u8  data[64];
} user_to_server_t;

typedef  struct
{
  u8  data[32];
} server_to_user_t;

typedef struct
{
  game_robot_state_t game_information;
  robot_hurt_data_t    blood_changed_data;
  real_shoot_t              real_shoot_data;
  extPowerHeatData_t  power_heat_data; //
  rfid_detect_t             rfid_data;
  game_result_t            game_result_data;
  get_buff_t                    get_buff_data;
  extGameRobotPos_t  position;        //
  client_show_data_t      client_data;     //用户自定义数据
  server_to_user_t          student_download_data;
} receive_judge_t;//接收来自主控 转发给妙算的信息

#endif // USE_SERIAL

#endif

