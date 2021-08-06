//-----------------------------------【步兵信息结构体】--------------------------------------------
// brief：包括云台控制模式，pitch角度，yaw角度等信息，是控制步兵运动的数据
//------------------------------------------------------------------------------------------------

#ifndef __INFANTRY_INFO_H__
#define __INFANTRY_INFO_H__

#include "Settings/Settings.h"
#include "JudgementInfo.h"
#include "Serial.h"
typedef enum
{
  CHASSIS_DATA_ID     = 0x0010,
  GIMBAL_DATA_ID      = 0x0011,
  SHOOT_TASK_DATA_ID  = 0x0012,
  INFANTRY_ERR_ID     = 0x0013,
  CONFIG_RESPONSE_ID  = 0x0014,
  CALI_RESPONSE_ID    = 0x0015,
  REMOTE_CTRL_INFO_ID = 0x0016,
  BOTTOM_VERSION_ID   = 0x0017,
  SENTRY_DATA_ID      = 0x0018,

  CHASSIS_CTRL_ID     = 0x00A0,
  GIMBAL_CTRL_ID      = 0x00A1,
  SHOOT_CTRL_ID       = 0x00A2,
  ERROR_LEVEL_ID      = 0x00A3,
  INFANTRY_STRUCT_ID  = 0x00A4,
  CALI_GIMBAL_ID      = 0x00A5,
} infantry_data_id_e;

typedef enum
{
  BOTTOM_DEVICE        = 0,
  GIMBAL_GYRO_OFFLINE  = 1,
  CHASSIS_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE   = 3,
  CHASSIS_M2_OFFLINE   = 4,
  CHASSIS_M3_OFFLINE   = 5,
  CHASSIS_M4_OFFLINE   = 6,
  REMOTE_CTRL_OFFLINE  = 7,
  JUDGE_SYS_OFFLINE    = 8,
  PC_SYS_OFFLINE       = 9,
  GIMBAL_YAW_OFFLINE   = 10,
  GIMBAL_PIT_OFFLINE   = 11,
  TRIGGER_MOTO_OFFLINE = 12,
  BULLET_JAM           = 13,
  CHASSIS_CONFIG_ERR   = 14,
  GIMBAL_CONFIG_ERR    = 15,
  ERROR_LIST_LENGTH    = 16,
} err_id_e;

typedef enum
{
  DEVICE_NORMAL = 0,
  ERROR_EXIST   = 1,
  UNKNOWN_STATE = 2,
} bottom_err_e;

typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  GIMBAL_ERROR         = 4,
  CHASSIS_ERROR        = 5,
  HARAWARE_ERROR       = 6,
} err_level_e;

typedef enum
{
  NO_CONFIG      = 0,//无定义
  DEFAULT_CONFIG = 1,//默认定义
  CUSTOM_CONFIG  = 3,//用户定义
} struct_config_e;

/********** the information send to computer ***********/

/**
  * @brief  chassis information
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    u8 ctrl_mode;      /* chassis control mode */
    float   gyro_palstance; /* chassis palstance(degree/s) from gyroscope */
    float   gyro_angle;     /* chassis angle(degree) relative to ground from gyroscope */
    float   ecd_palstance;  /* chassis palstance(degree/s) from chassis motor encoder calculated */
    float   ecd_calc_angle; /* chassis angle(degree) relative to ground from chassis motor encoder calculated */
    s16 x_spd;        /* chassis x-axis move speed(mm/s) from chassis motor encoder calculated */
    s16 y_spd;        /* chassis y-axis move speed(mm/s) from chassis motor encoder calculated */
    s32 x_position;     /* chassis x-axis position(mm) relative to the starting point */
    s32 y_position;     /* chassis y-axis position(mm) relative to the starting point */
}__attribute__((packed))  chassis_info_t;
#else
typedef  struct  //__packed
{
    u8 ctrl_mode;      /* chassis control mode */
    float   gyro_palstance; /* chassis palstance(degree/s) from gyroscope */
    float   gyro_angle;     /* chassis angle(degree) relative to ground from gyroscope */
    float   ecd_palstance;  /* chassis palstance(degree/s) from chassis motor encoder calculated */
    float   ecd_calc_angle; /* chassis angle(degree) relative to ground from chassis motor encoder calculated */
    s16 x_spd;        /* chassis x-axis move speed(mm/s) from chassis motor encoder calculated */
    s16 y_spd;        /* chassis y-axis move speed(mm/s) from chassis motor encoder calculated */
    s32 x_position;     /* chassis x-axis position(mm) relative to the starting point */
    s32 y_position;     /* chassis y-axis position(mm) relative to the starting point */
}  chassis_info_t;
#endif // !USE_WIN

/**
  * @brief  gimbal information
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    u8 ctrl_mode;          /* gimbal control mode */
    float   pit_relative_angle; /* pitch angle(degree) relative to the gimbal center */
    float   yaw_relative_angle; /* yaw angle(degree) relative to the gimbal center */
    float   pit_absolute_angle; /* pitch angle(degree) relative to ground */
    float   yaw_absolute_angle; /* yaw angle(degree) relative to ground */
    float   pit_palstance;      /* pitch axis palstance(degree/s) */
    float   yaw_palstance;      /* yaw axis palstance(degree/s) */
}__attribute__((packed))    gimbal_info_t;
#else
typedef  struct  //__packed
{
    u8 ctrl_mode;          /* gimbal control mode */
    float   pit_relative_angle; /* pitch angle(degree) relative to the gimbal center */
    float   yaw_relative_angle; /* yaw angle(degree) relative to the gimbal center */
    float   pit_absolute_angle; /* pitch angle(degree) relative to ground */
    float   yaw_absolute_angle; /* yaw angle(degree) relative to ground */
    float   pit_palstance;      /* pitch axis palstance(degree/s) */
    float   yaw_palstance;      /* yaw axis palstance(degree/s) */
}  gimbal_info_t;
#endif // !USE_WIN


/**
  * @brief  shoot information
  */
#ifndef USE_WIN
typedef  struct //__packed
{
    s16 remain_bullets;  /* the member of remain bullets */
    s16 shoot_bullets;    /* the member of bullets that have been shot */
    u8 fric_wheel_run;  /* friction run or not */
}__attribute__((packed))  shoot_info_t;
#else
typedef  struct //__packed
{
    s16 remain_bullets;  /* the member of remain bullets */
    s16 shoot_bullets;    /* the member of bullets that have been shot */
    u8 fric_wheel_run;  /* friction run or not */
}  shoot_info_t;
#endif // !USE_WIN


/**
  * @brief  infantry error information
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    bottom_err_e err_sta;                 /* bottom error state */
    bottom_err_e err[ERROR_LIST_LENGTH];  /* device error list */
}__attribute__((packed)) infantry_err_t;
#else
typedef  struct  //__packed
{
    bottom_err_e err_sta;                 /* bottom error state */
    bottom_err_e err[ERROR_LIST_LENGTH];  /* device error list */
} infantry_err_t;
#endif // !USE_WIN


/**
  * @brief  infantry structure config response
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    struct_config_e chassis_config;
    struct_config_e gimbal_config;
}__attribute__((packed))  config_response_t;
#else
typedef  struct  //__packed
{
    struct_config_e chassis_config;
    struct_config_e gimbal_config;
}  config_response_t;
#endif // !USE_WIN


/**
  * @brief  gimbal calibrate response
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    u8 type;     //0x01 success 0x00 fault
    s16 yaw_offset;
    s16 pitch_offset;
}__attribute__((packed))  cali_response_t;
#else
typedef  struct  //__packed
{
    u8 type;     //0x01 success 0x00 fault
    s16 yaw_offset;
    s16 pitch_offset;
}  cali_response_t;
#endif // !USE_WIN


/**
  * @brief  remote control information
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    /* rocker channel information */
    s16 ch1;
    s16 ch2;
    s16 ch3;
    s16 ch4;
    /* left and right lever information */
    u8 sw1;
    u8 sw2;
    /* mouse movement and button information */
    struct  //__packed
    {
        s16 x;
        s16 y;
        s16 z;

        u8 l;
        u8 r;
    } mouse;
    /* keyboard key information */
    union  //__packed
    {
        u16 key_code;
        struct   //__packed
        {
            u16 W : 1;
            u16 S : 1;
            u16 A : 1;
            u16 D : 1;
            u16 SHIFT : 1;
            u16 CTRL : 1;
            u16 Q : 1;
            u16 E : 1;
            u16 R : 1;
            u16 F : 1;
            u16 G : 1;
            u16 Z : 1;
            u16 X : 1;
            u16 C : 1;
            u16 V : 1;
            u16 B : 1;
        }__attribute__((packed))  bit;
    }__attribute__((packed))  kb;
}__attribute__((packed))  rc_info_t;
#else
typedef  struct  //__packed
{
    /* rocker channel information */
    s16 ch1;
    s16 ch2;
    s16 ch3;
    s16 ch4;
    /* left and right lever information */
    u8 sw1;
    u8 sw2;
    /* mouse movement and button information */
    struct  //__packed
    {
        s16 x;
        s16 y;
        s16 z;

        u8 l;
        u8 r;
    } mouse;
    /* keyboard key information */
    union  //__packed
    {
        u16 key_code;
        struct   //__packed
        {
            u16 W : 1;
            u16 S : 1;
            u16 A : 1;
            u16 D : 1;
            u16 SHIFT : 1;
            u16 CTRL : 1;
            u16 Q : 1;
            u16 E : 1;
            u16 R : 1;
            u16 F : 1;
            u16 G : 1;
            u16 Z : 1;
            u16 X : 1;
            u16 C : 1;
            u16 V : 1;
            u16 B : 1;
        }  bit;
    }  kb;
}  rc_info_t;
#endif // !USE_WIN


/**
  * @brief  bottom software version information
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    u8 num[16];
}__attribute__((packed))  version_info_t;
#else
typedef  struct  //__packed
{
    u8 num[16];
}  version_info_t;
#endif // !USE_WIN


/**
  * @brief  stm32 information
  */
#ifndef USE_WIN
#if (ARMS==7)
typedef  struct  //__packed
{
    u8 enemy_color; //敌方颜色：1 red 2 blue
    u8 chassis_pos; //底盘位置：1 基地侧 2 中间 3 断桥侧
    u8 armor_id;    //装甲板：  1 基地侧 2 外侧
    u16 remain_hp;  //剩余血量
    float yaw_relative_angle;  //云台yaw角度：相对底盘的角度（以血条的方向为0度，顺时针递增）
}__attribute__((packed))  stm32_info_t;
#else
typedef  struct  //__packed
{
    u8 main_mode;   //模式：0装甲板 1神符 2空
    u8 enemy_color; //敌方颜色：1 red 2 blue
    u8 is_left;
    u8 run_left;
    float bullet_spd;
    float pitch;
    float yaw;
    float pitch_offset; // pit_offset
    float yaw_offset;  // yaw_offset
//    float distance; // m
//    unsigned char offset_angle;
//    float distance;  // 单位m
//    float pitch_compensate;
//    float yaw_compensate;

    //float pitch_pal;
    //float yaw_pal;

#ifdef USE_OPERATOR_COMPENSATE
    int pitch_add;    //pitch抬升补偿角
    int pitch_reduce;  //pitch下降补偿角
#endif
}__attribute__((packed))  stm32_info_t;
#endif
#else
#if (ARMS==7)
typedef  struct  __packed
{
    u8 enemy_color; //敌方颜色：1 red 2 blue
    u8 chassis_pos; //底盘位置：1 基地侧 2 中间 3 断桥侧
    u8 armor_id;    //装甲板：  1 基地侧 2 外侧
    u16 remain_hp;  //剩余血量
    float yaw_relative_angle;  //云台yaw角度：相对底盘的角度（以血条的方向为0度，顺时针递增）
}__attribute__((packed))  stm32_info_t;
#else
typedef  struct  //__packed
{
    u8 main_mode;   //模式：0装甲板 1神符 2空
    u8 enemy_color; //敌方颜色：1 red 2 blue
}  stm32_info_t;
#endif
#endif // !USE_WIN


/********** the information from computer **********/
#ifndef USE_WIN
typedef  struct  //__packed
{
    s16 x_offset;   /* offset(mm) relative to the x-axis of the chassis center */
    s16 y_offset;   /* offset(mm) relative to the y-axis of the chassis center */
    float   w_spd;    /* rotation speed(degree/s) of chassis */
}__attribute__((packed))  chassis_rotate_t;
#else
typedef  struct  //__packed
{
    s16 x_offset;   /* offset(mm) relative to the x-axis of the chassis center */
    s16 y_offset;   /* offset(mm) relative to the y-axis of the chassis center */
    float   w_spd;    /* rotation speed(degree/s) of chassis */
}  chassis_rotate_t;
#endif // !USE_WIN


/**
  * @brief  chassis control information
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    u8          ctrl_mode; /* chassis control mode */
    s16          x_spd;   /* x-axis move speed(mm/s) of chassis */
    s16          y_spd;   /* y-axis move speed(mm/s) of chassis */
    chassis_rotate_t w_info;    /* rotation control of chassis */
    s16 y_distance;//左为正 右为负(矿石)
    u8  judge;//0没有对准 1对准（矿石）
}__attribute__((packed))  chassis_ctrl_t;
#else
typedef  struct  //__packed
{
    u8          ctrl_mode; /* chassis control mode */
    s16          x_spd;   /* x-axis move speed(mm/s) of chassis */
    s16          y_spd;   /* y-axis move speed(mm/s) of chassis */
    chassis_rotate_t w_info;    /* rotation control of chassis */
}  chassis_ctrl_t;
#endif // !USE_WIN


/**
  * @brief  gimbal control information
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    u32 time;
    float    pit_ref;      /* gimbal pitch reference angle(degree) */
    float    yaw_ref;      /* gimbal yaw reference angle(degree) */

    //u8  ctrl_mode;    /* gimbal control mode */
    //float    tgt_dist;     /* visual target distance */
    //float    x;
    //float    y;
    //float    z;

    u8  visual_valid; /* visual information valid or not */
    u8  buff_shoot;
}__attribute__((packed))   gimbal_ctrl_t;
#else
typedef  struct  //__packed
{
    u32 time;
    float    pit_ref;      /* gimbal pitch reference angle(degree) */
    float    yaw_ref;      /* gimbal yaw reference angle(degree) */

    //u8  ctrl_mode;    /* gimbal control mode */
    //float    tgt_dist;     /* visual target distance */
    //float    x;
    //float    y;
    //float    z;

    u8  visual_valid; /* visual information valid or not */
    u8  buff_shoot;
}  gimbal_ctrl_t;
#endif // !USE_WIN


/**
  * @brief  shoot control information
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    u8  shoot_cmd;      /* single shoot command */
    u8  c_shoot_cmd;    /* continuous shoot command */
    u8  fric_wheel_run; /* friction run or not */
    u16 fric_wheel_spd; /* fricrion wheel speed */
}__attribute__((packed))  shoot_ctrl_t;
#else
typedef  struct  //__packed
{
    u8  shoot_cmd;      /* single shoot command */
    u8  c_shoot_cmd;    /* continuous shoot command */
    u8  fric_wheel_run; /* friction run or not */
    u16 fric_wheel_spd; /* fricrion wheel speed */
}  shoot_ctrl_t;
#endif // !USE_WIN


/**
  * @brief  robot system error level
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    err_level_e err_level; /* the error level is included in err_level_e enumeration */
}__attribute__((packed))  global_err_level_t;
#else
typedef  struct  //__packed
{
    err_level_e err_level; /* the error level is included in err_level_e enumeration */
}  global_err_level_t;
#endif // !USE_WIN


/**
  * @brief  infantry structure configuration information
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    struct_config_e  chassis_config;  /* chassis structure config state */
    u16         wheel_perimeter; /* the perimeter(mm) of wheel */
    u16         wheel_track;     /* wheel track distance(mm) */
    u16         wheel_base;      /* wheelbase distance(mm) */
    struct_config_e  gimbal_config;   /* gimbal structure config state */
    s16          gimbal_x_offset; /* gimbal offset(mm) relative to the x-axis of the chassis center */
    s16          gimbal_y_offset; /* gimbal offset(mm) relative to the y-axis of the chassis center */
}__attribute__((packed))  infantry_structure_t;
#else
typedef  struct  //__packed
{
    struct_config_e  chassis_config;  /* chassis structure config state */
    u16         wheel_perimeter; /* the perimeter(mm) of wheel */
    u16         wheel_track;     /* wheel track distance(mm) */
    u16         wheel_base;      /* wheelbase distance(mm) */
    struct_config_e  gimbal_config;   /* gimbal structure config state */
    s16          gimbal_x_offset; /* gimbal offset(mm) relative to the x-axis of the chassis center */
    s16          gimbal_y_offset; /* gimbal offset(mm) relative to the y-axis of the chassis center */
}  infantry_structure_t;
#endif // !USE_WIN


/**
  * @brief  gimbal calibrate command
  */
#ifndef USE_WIN
typedef  struct  //__packed
{
    u8 type;        /* 0x01 calibrate gimbal center, 0x02 calibrate camera */
}__attribute__((packed))  cali_cmd_t;
#else
typedef  struct  //__packed
{
    u8 type;        /* 0x01 calibrate gimbal center, 0x02 calibrate camera */
}  cali_cmd_t;
#endif // !USE_WIN

/********* variables **********/
/**
  * @brief  the data structure send to pc
  */
#ifndef USE_WIN
typedef struct  //__packed
{
    /* data send */
  //  chassis_info_t    chassis_information;
  //  gimbal_info_t     gimbal_information;
  //  shoot_info_t      shoot_task_information;
  //  infantry_err_t    bottom_error_data;
  //  config_response_t structure_config_data;
  //  cali_response_t   cali_response_data;
  //  rc_info_t         remote_ctrl_data;
    version_info_t    version_info_data;  //stm32信息旧版方式（本为版本信息）
    stm32_info_t     stm32_info_data;       //stm32信息
}__attribute__((packed))  send_pc_t;
#else
typedef struct  //__packed
{
    /* data send */
  //  chassis_info_t    chassis_information;
  //  gimbal_info_t     gimbal_information;
  //  shoot_info_t      shoot_task_information;
  //  infantry_err_t    bottom_error_data;
  //  config_response_t structure_config_data;
  //  cali_response_t   cali_response_data;
  //  rc_info_t         remote_ctrl_data;
    version_info_t    version_info_data;  //stm32信息旧版方式（本为版本信息）
    stm32_info_t     stm32_info_data;       //stm32信息
}  send_pc_t;
#endif // !USE_WIN


/**
  * @brief  the data structure receive from pc
  */
#ifndef USE_WIN
typedef struct  //__packed
{
    /* data receive */
    chassis_ctrl_t       chassis_control_data;
    gimbal_ctrl_t        gimbal_control_data;
    shoot_ctrl_t         shoot_control_data;
    global_err_level_t   global_error_level;
    infantry_structure_t  structure_data;
    cali_cmd_t               cali_cmd_data;
    /* receive to forward */
    client_show_data_t   show_in_client_data;
    user_to_server_t     pc_to_server_data;
}__attribute__((packed))  receive_pc_t;
#else
typedef struct  //__packed
{
    /* data receive */
    chassis_ctrl_t       chassis_control_data;
    gimbal_ctrl_t        gimbal_control_data;
    shoot_ctrl_t         shoot_control_data;
    global_err_level_t   global_error_level;
    infantry_structure_t  structure_data;
    cali_cmd_t               cali_cmd_data;
    /* receive to forward */
    //client_show_data_t   show_in_client_data;
    //user_to_server_t     pc_to_server_data;
}  receive_pc_t;
#endif // !USE_WIN

typedef union
{
    struct
    {
        unsigned char low_byte;
        unsigned char mlow_byte;
        unsigned char mhigh_byte;
        unsigned char high_byte;
    }float_byte;

   float  value;
}FLAOT_UNION;

void pc_data_handler(u8 *p_frame, send_pc_t &pc_send_mesg);

#endif  // __INFANTRY_INFO_H__
