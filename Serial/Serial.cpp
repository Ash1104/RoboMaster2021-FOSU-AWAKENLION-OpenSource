#include "Serial.h"

#ifdef USE_SERIAL

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>     //Unix标准函数定义
#include <fcntl.h>      //文件控制定义
#include <termios.h>    //PPSIX终端控制定义
#include <errno.h>      //错误号定义

#include <sys/ioctl.h>  /* BSD and Linux */
#include <sys/stat.h>

#include <stropts.h>    /* XSI STREAMS */

int rate_arr[] = { B921600, B576000, B460800,
                   B230400, B115200, B57600,
                   B38400, B19200, B9600,
                   B4800, B2400, B1800,
                   B1200, B600, B300,
                   B200, B150, B134,
                   B110, B75, B50,
                 };

int name_arr[] = { 921600, 576000, 460800,
                   230400, 115200, 57600,
                   38400, 19200, 9600,
                   4800, 2400, 1800,
                   1200, 600, 300,
                   200, 150, 134,
                   110, 75, 50,
                 };

Serial::Serial()
{

}

Serial::Serial(DeviceParam port_param)
{
    this->port_param = port_param;
}

bool Serial::reading(unsigned char *data, u16 &size)
{
    //unsigned char buf[UNPACK_DATA_BUFFSIZE] = {0};

    ioctl(fd, FIONREAD, &size);
    if(size >1 && size < UNPACK_DATA_BUFFSIZE)
        size = read(fd, data, size);         //一次把数据放进缓冲区
    else if(size >= UNPACK_DATA_BUFFSIZE)
        size = read(fd, data, UNPACK_DATA_BUFFSIZE);
    else
        return false;
    return true;
}

int Serial::openPort(const char * dev_name){
    /*int fd; */// file description for the serial port
    fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1){ // if open is unsucessful
        printf("串口打开失败\n");
    }
    else{
        fcntl(fd, F_SETFL, 0);
        printf("串口打开完成\n");
    }
    configurePort(fd);
    return(fd);
}

int Serial::configurePort(int fd){                      // configure the port
    struct termios port_settings;               // structure to store the port settings in
    cfsetispeed(&port_settings, B115200);       // set baud rates
    cfsetospeed(&port_settings, B115200);

    cfmakeraw(&port_settings);                  //RAW原始模式
    port_settings.c_cflag &= ~PARENB;           // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;

    tcflush(fd,TCIFLUSH);
//    port_settings.c_cc[VTIME] = 0; /* set overtime _s */
//    port_settings.c_cc[VMIN] = 32; /* Update the options and do it NOW */

    tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port
    return(fd);
}

bool Serial::writing(unsigned char * data, int size)
{
    if (size == write(fd, data, size))
    {
        return true;
    }
    return false;
}

int Serial::openDevice()
{
    fd = open(port_param.dev_name, O_RDWR | O_NONBLOCK);
    if(-1 == fd)
    {
        perror("Can't Open Serial Port");
        return -1;
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
        printf("port is open.\n");
    }

    setDevice();
    return fd;
}

bool Serial::closeDevice()
{
    close(fd);
}

bool Serial::setDevice()
{
    if(setBaudRate(port_param.baud_rate))
        if(setParity(port_param.databits, port_param.stopbits, port_param.parity))
            return true;
    return false;
}

bool Serial::setBaudRate(int baud_rate)
{
    int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for(int i = 0;i < sizeof(rate_arr) / sizeof(int); i++)
    {
        if(baud_rate == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, rate_arr[i]);
            cfsetospeed(&Opt, rate_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if(status != 0)
            {
                perror("tcsetattr fd1");
                return false;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
    return true;
}

bool Serial::setParity(int databits, int stopbits, int parity)
{
    struct termios options;
    if(tcgetattr( fd,&options) != 0)
    {
        perror("SetupSerial 1");
        return false;
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) /*set number of data bits*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n"); return false;
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;    /* Clear parity enable */
            options.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); /* Set odd check*/
            options.c_iflag |= INPCK;             /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;      /* Enable parity */
            options.c_cflag &= ~PARODD;     /* set even check*/
            options.c_iflag |= INPCK;       /* Disnable parity checking */
            break;
        case 'S':
        case 's':  /*as no parity*/
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return false;
        }
    /* Set stop bit*/
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
           break;
        default:
             fprintf(stderr,"Unsupported stop bits\n");
             return false;
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 150; /* set overtime 15s */
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return false;
    }
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    options.c_oflag  &= ~OPOST;   /*Output*/
    return true;
}

#endif // USE_SERIAL
