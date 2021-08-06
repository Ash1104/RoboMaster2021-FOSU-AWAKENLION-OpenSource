//-----------------------------------【串口】--------------------------------------------
// brief：打开串口，配置串口，发送数据
//------------------------------------------------------------------------------------------------

#ifndef SERIAL_H
#define SERIAL_H

#include "Settings/Settings.h"
#include "Protocol.h"

#define UNPACK_DATA_BUFFSIZE 200           //接收自主控板信息的缓冲区大小
#define PACK_DATA_BUFFSIZE   150            //发送到主控板信息的缓冲区大小

class Serial
{
public:
	Serial();
	Serial(DeviceParam port_param);

    int openPort(const char * dev_name);

    int configurePort(int fd);

    /**
     * @brief  读取缓冲区
     * @author 参与开发人员
     * @date   2019-
     */
	bool reading(unsigned char * data, u16 &size);

    /**
     * @brief  writing
     * @author 参与开发人员
     * @date   2019-
     */
	bool writing(unsigned char * data, int size);

	/**
	 * @brief  打开设备
	 * @param  第一个参数
	 * @return 返回值
     * @author 梁尧森
	 * @date   2019.3.23
	 */
	int openDevice();

	/**
	 * @brief  关闭设备
	 * @param  第一个参数
	 * @return 返回值
     * @author 梁尧森
	 * @date   2019.3.23
	 */
	bool closeDevice();

	/**
	 * @brief  配置设备
	 * @param  第一个参数
	 * @return 返回值
     * @author 梁尧森
	 * @date   2019.3.23
	 */
	bool setDevice();

private:
	/**
	 * @brief  设置波特率
	 * @param  第一个参数
	 * @return 返回值
     * @author 梁尧森
	 * @date   2019.3.23
	 */
	bool setBaudRate(int baud_rate);

	/**
     *@brief  设置串口数据位，停止位和效验位
	 *@param  databits 类型  int  数据位   取值为 7 或者8*
	 *@param  stopbits 类型  int  停止位   取值为 1 或者2*
	 *@param  parity   类型  int  效验类型 取值为 N,E,O,,S
	 */
	bool setParity(int databits, int stopbits, int parity);

private:
	int fd;
	DeviceParam port_param;
};

#endif // SERIAL_H
