//-----------------------------------【打包】--------------------------------------------
// brief：将控制数据的结构图转成字符型数据，以便串口发送
//------------------------------------------------------------------------------------------------

#ifndef PACKDATA_H
#define PACKDATA_H

#include "InfantryInfo.h"
#include "Settings/Settings.h"

class PackData
{
public:
    PackData();

    void processing(Serial *serial);
    void process(Serial *serial);

    /**
     * @brief  简介
     * @param  第一个参数
     * @return 返回值
     * @autor  参与开发的人员
     * @date   2018-
     */
    receive_pc_t *setPc2StmMesg()
    {
        return &pc2stm_mesg;
    }
private:
    /**
     * @brief  简介
     * @param  第一个参数
     * @return 返回值
     * @autor  参与开发的人员
     * @date   2018-
     */
    u8* packData(u16 cmd_id, u8 *p_data, u16 len,u8 *tx_buf);

private:
    receive_pc_t  pc2stm_mesg;
    u8 tx_buf[PACK_DATA_BUFFSIZE];
};

//-----------------------------------【解包】--------------------------------------------
// brief：接收主控板的信号，包括控制模式（比如人工操作/开启自动辅助瞄准/神符击打，由操作手控制），敌方的颜色
// 1. 从设备中读数据，加入缓冲区队列
// 2. 寻找头标志，分为一段段数据块
// 3. 解析每段数据块的信息
//------------------------------------------------------------------------------------------------
class UnpackData
{
public:
    UnpackData();

    void processing(Serial *serial);
    void process(Serial *serial);

    send_pc_t *getStm2PcMesg()
    {
        return &stm2pc_mesg;
    }

private:
    /**
     * @brief  分块
     * @param  第一个参数
     * @return 返回值
     * @autor  参与开发的人员
     * @date   2018-
     */
    void blockData(u8 *data, u16 &size);

    /**
     * @brief  解析
     * @param  第一个参数
     * @return 返回值
     * @autor  参与开发的人员
     * @date   2018-
     */
    void unpackData(u8 *data, int size);
private:
    send_pc_t     stm2pc_mesg;
    u8 buf[UNPACK_DATA_BUFFSIZE];
};

#endif // PACKDATA_H
