
/**********************************************************
 Copyright (C), 2001-2011, Sumavision Tech. Co., Ltd.
 FileName: serial.c
 Author/Version/Date: rd5/V1.0/2010.4.12
 Description:     ���������������ɱ��ϲ���õ�
                             open,ioctl�Լ�read ,write����
 Version:         
 History:         
 *********************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <termios.h>

#include "uart_serial.h"

#define MAX_SER_DEVNUM 3

serialAttr serialVar[MAX_SER_DEVNUM] = {{0}, {0},{0}};

#define SERIAL_DBG
#define DEV_MK_FD(t, d)	(((t) << 16) | ((d) + 1))
#ifdef SERIAL_DBG
#define __DBG(fmt, args...) fprintf(stdout,"Debug: " fmt, ## args)
#else
#define __DBG(fmt, args...)
#endif

#define RPT_ERR (1) // error, system error
#define RPT_WRN (2) // warning, maybe wrong, maybe OK
#define RPT_INF (3) // important information
#define RPT_DBG (4) // debug information


static int rpt_lvl = RPT_WRN; /* report level: ERR, WRN, INF, DBG */

/* report micro */
#define RPT(lvl, ...) \
    do { \
        if(lvl <= rpt_lvl) { \
            switch(lvl) { \
                case RPT_ERR: \
                    fprintf(stderr, "\"%s\" line %d [err]: ", __FILE__, __LINE__); \
                    break; \
                case RPT_WRN: \
                    fprintf(stderr, "\"%s\" line %d [wrn]: ", __FILE__, __LINE__); \
                    break; \
                case RPT_INF: \
                    fprintf(stderr, "\"%s\" line %d [inf]: ", __FILE__, __LINE__); \
                    break; \
                case RPT_DBG: \
                    fprintf(stderr, "\"%s\" line %d [dbg]: ", __FILE__, __LINE__); \
                    break; \
                default: \
                    fprintf(stderr, "\"%s\" line %d [???]: ", __FILE__, __LINE__); \
                    break; \
                } \
                fprintf(stderr, __VA_ARGS__); \
                fprintf(stderr, "\n"); \
        } \
    } while(0)



/**********************************************************
 Function:	serialInit
 Description: ��ʼ������Ϊrawģʽ����ջ���
 Input: 	fd :����������
 	                  
 Output:
 Return:         0����-1��
 Others:         
**********************************************************/
int serialSetRaw(unsigned int h)
{
    int fd = serialVar[h].serialFd;
    struct termios	newtio;

    if(tcgetattr (fd ,&newtio) != 0 ){
        __DBG("get attr1 failed...\n");
        return -1;
    }

    /*���ó�Ϊraw mode*/
    
    newtio.c_lflag  &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
    newtio.c_oflag  &= ~OPOST; 
    /*disable flow control*/
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
    newtio.c_iflag &= ~(PARMRK | ISTRIP | IGNPAR);
    newtio.c_iflag &= ~(IGNBRK | BRKINT | INLCR | IGNCR | ICRNL);
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 9;

    tcflush( fd , FLUSHMETHOD);
    if ((tcsetattr (fd, SERIALSETOPTION, &newtio) )!=0){
        __DBG("get attr2 failed...\n");
        return -1;
    }

    return 0;
}

/**********************************************************
 Function:	serialSetSpeed
 Description: ���ô��ڵ������������ (������)
 Input: 	fd :����������
 	speed:��������
 	                  
 Output:
 Return:        
 Others:         
**********************************************************/
int  serialSetSpeed(unsigned int h,int speed )
{
    int fd = serialVar[h].serialFd;
    struct termios	newtio;

    if(fd < 1){
        return -1;
    }

    if( tcgetattr (fd ,&newtio) != 0 ){
        __DBG("serialSetSpeed@tcgetattr failed\n");
        return -1;
    }
    
    switch(speed){
        case  300:
        cfsetispeed(&newtio, B300);
        cfsetospeed(&newtio, B300);
        break;
    case 600:
        cfsetispeed(&newtio, B600);
        cfsetospeed(&newtio, B600);
        break;
    case 1200:
        cfsetispeed(&newtio, B1200);
        cfsetospeed(&newtio, B1200);
        break;
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 38400:
        cfsetispeed(&newtio, B38400);
        cfsetospeed(&newtio, B38400);
        break;
    case 57600:
        cfsetispeed(&newtio, B57600);
        cfsetospeed(&newtio, B57600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;	
    default: 
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;	
    }
    
    newtio.c_cflag |= (CLOCAL | CREAD);

    tcflush( fd , FLUSHMETHOD);
    
    if ( (tcsetattr (fd, SERIALSETOPTION,&newtio) )!=0){
        __DBG("serialSetSpeed@tcsetattr failed...\n");
        return -1;
    }

    return 0;
}


/**********************************************************
 Function:	serialSetBits
 Description: ���ô��ڵ�����λ
 Input: 	fd :����������
 	bit: ����λ��, ������Ϊ5\6\7\8λ 	                  
 Output:
 Return:        
 Others:         
**********************************************************/
int  serialSetBits(unsigned int h, int bit)
{
    int fd = serialVar[h].serialFd;

	struct termios	newtio ;

    if(fd < 1){
        return -1;
    }

    if( tcgetattr (fd ,&newtio) != 0 ){
        return -1;
    }

    newtio.c_cflag &= ~CSIZE;

    switch( bit ){
        case 5:
            newtio.c_cflag |= CS5;
            break;
        case 6:
            newtio.c_cflag |= CS6;
            break;
        case  7:
            newtio.c_cflag |= CS7;
            break;
        case  8:
            newtio.c_cflag |= CS8;
            break;
        default: 
            return -1;	
    }

    tcflush( fd , FLUSHMETHOD);
    if ( (tcsetattr (fd,SERIALSETOPTION,&newtio) )!=0){
        perror(" serial set");
        return -1;
    }

    return 0;
}

/**********************************************************
 Function:	serialSetEvent
 Description: ���ô��ڵ���żУ��
 Input: 	fd :����������
 	event:��żУ���־
 	    0��У��,1��У��, 2żУ��              
 Output:
 Return:        
 Others:         
**********************************************************/
int  serialSetEvent(unsigned int h, char event)
{

    int fd = serialVar[h].serialFd;

    struct termios newtio;

    if(fd < 1){
        return -1;
    }

    if(tcgetattr (fd ,&newtio) != 0){
        return -1;
    }

    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~PARODD;
    newtio.c_iflag &= ~INPCK;
    newtio.c_iflag &= ~ISTRIP;
    switch( event ){
        case 'o':
        case 'O':
        case 1:
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP );
            break;			
        case 'e':
        case 'E':
        case 2:
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP );
            break;			
        case 'n':
        case 'N':
        case 0:
            break;
        default:
            return -1;
    }

    tcflush( fd , FLUSHMETHOD);
    if ((tcsetattr (fd, SERIALSETOPTION, &newtio))!=0){
        return -1;
    }

    return 0;
}

/**********************************************************
 Function:	serialSetStop
 Description: ���ô��ڵ�ֹͣλ
 Input: 	fd :����������
 	stop:ֹͣλ1����2λ
 Output:
 Return:        
 Others:         
**********************************************************/
int serialSetStop(unsigned int h, int stop)
{

    int fd = serialVar[h].serialFd;

    struct termios newtio ;

    if(fd < 1){
        return -1;
    }

    if( tcgetattr (fd ,&newtio) != 0 ){
        return -1;
    }
	
    switch(stop){
        case 1:
            newtio.c_cflag &=~CSTOPB;
            break;
        case 2:
            newtio.c_cflag |= CSTOPB;
            break;
        default:
            return -1;
            break;
    }
	
    tcflush( fd , FLUSHMETHOD);
    if ((tcsetattr (fd,SERIALSETOPTION,&newtio))!=0){
        return -1;
    }

    return 0;
}

/**********************************************************
 Function:	serialOpen
 Description: �򿪲��ҳ�����ʼ������
 Input: 	h : ���ں�
 		                  
 Output:
 Return:         0����-1��
 Others:         ���ڴ򿪺�ᱻ����Ϊraw ģʽ
**********************************************************/
int serialOpen(unsigned int type, int idx, char * ttyname)
{
    //printf("serialOpen INside@@@ %d  %d \n", type, idx);
    RPT(RPT_DBG, "serialOpen INside@@@ %d  %d \n", type, idx);
    serialAttr* varPtr = &serialVar[idx];
    char path[64];
    int fd;
    
    if(varPtr->usage == SL_USE_CONSL)
        return -1;	
    
    if(varPtr->opened)
        return DEV_MK_FD(type, idx);
    
    sprintf(path,"/dev/%s%d", ttyname, idx);

    fd = open(path, O_RDWR |O_NOCTTY |O_NDELAY );
    //printf("serialOpen path %s  type %d   fd %d \n", path, type, fd);
    RPT(RPT_DBG, "serialOpen path %s  type %d   fd %d \n", path, type, fd);
    if(fd <= 0){
        perror("open tty failed\n");
        return -1;
    }else{
        varPtr->serialFd = fd;
        varPtr->opened = 1;
        if(serialSetRaw(idx)){
            perror("set tty to raw mode failed\n");
            return -1;
        }
        return DEV_MK_FD(type, idx);	
    }
}

/**********************************************************
 Function:	serialWrite
 Description: ����д����
 Input: 	h:��serialOpen���صľ����buf ������
 	len:Ԥ�ڵĳ���
 Output:
 Return:        ʧ�ܷ���-1���ɹ�����ʵ��д��ĳ���
 Others:         
**********************************************************/
int serialWrite(unsigned int h , char *buf, int len)
{
    int fd;
    int writelen;
    //printf("serialWrite   fd  %d  h %d opened %d\n", serialVar[h].serialFd, h, serialVar[h].opened);
	RPT(RPT_DBG, "serialWrite   fd  %d  h %d opened %d\n", serialVar[h].serialFd, h, serialVar[h].opened);

    if(serialVar[h].opened == 0)
        return -1;

    fd = serialVar[h].serialFd;
    if(fd <= 0)
        return -1;

    if ( buf == NULL )
        return -1;
		
    writelen = write (fd, buf ,len ) ;
	//tcflush(fd, FLUSHMETHOD);//lixin added;	
    if(writelen < 0) perror("serialWrite@error\n");
    return writelen;
}

/**********************************************************
 Function:	serialRead
 Description: ���ڶ�����
 Input: 	h:��serialOpen���صľ����buf ������
 	len:Ԥ�ڵĳ���
 Output:
 Return:        ʧ�ܷ���-1���ɹ�����ʵ�ʶ�ȡ�ĳ���
 Others:         
**********************************************************/
int  serialRead(unsigned int h, char *buf,int len)
{

    int  readlen;
    int fd;

    if(h > MAX_SER_DEVNUM-1){
        return -1;
    }

    if(serialVar[h].opened == 0)
        return -1;

    fd = serialVar[h].serialFd;
	if(fd <= 0)
		return -1;
	
    fcntl(fd, F_SETFL, 0);
    readlen = read(fd, buf, len);
    if(readlen < 0) perror("serialRead@error\n");
    return readlen;
}

/**********************************************************
 Function:	serialClose
 Description: ���ڹرպ���
 Input: 	h:��serialOpen���صľ����
 Output:
 Return:       
 Others:         
**********************************************************/
int serialClose(unsigned int h)
{
    int fd;
    serialAttr* sPtr;

    if(h > MAX_SER_DEVNUM-1){
        return -1;
    }

    fd = serialVar[h].serialFd;
    sPtr = &serialVar[h];
    
    if(fd > 0)
        close(fd);
    
    sPtr->serialFd = -1;
    sPtr->opened = 0;
    //printf("tty%d closed\n",h);
	RPT(RPT_DBG, "tty%d closed\n",h);

	return 0;
}


/**********************************************************
 Function:	serialIoctl
 Description: ���ڹ��ߺ������������ò����ʵȲ���
 Input: 	h:��serialOpen���صľ����cmd������
 		arg �������
 		                  
 Output:
 Return:         0����-1��
 Others:         �μ�serial.h
**********************************************************/
int serialIoctl(unsigned int h, unsigned int cmd, unsigned int arg)
{

    switch(cmd){
        /*����������*/
        case SL_BAUD_SET:
            if(serialSetSpeed(h, arg)){
                return -1;
            }
            break;
        /*����λ����*/
        case SL_DBIT_SET:
            if(serialSetBits(h, arg)){
                 return -1;
            }
            break;
        /*��żУ��λ*/
        case SL_PAR_SET:
            if(serialSetEvent(h, arg)){
                return -1;
            }
            break;
        /*ֹͣλ����*/
        case SL_STB_SET:
            if(serialSetStop(h, arg)){
                return -1;
            }
            break;
        /*���ڵ���;*/
        case SL_USE_SET:
            serialVar[h].usage = arg;
            break;
        case SL_MODE_SET:  
            break;
        case SL_CLOSE_IO:
            serialClose(h);
            break;
        default: 
            return -1;
    }
	return 0;
}

serialAttr * serialGet(unsigned int h)
{
    return (&serialVar[h]);
}


#ifdef SERIAL_DBG
void __DEGPRINT(unsigned int h)
{
    int fd;
    int rate;
    struct termios options; 

    fd = serialVar[h].serialFd;
    if(fd <= 0)
        return;

    if  ( tcgetattr( fd,&options)  !=  0) { 
		perror("SetupSerial");     
		return;  
	}
    rate = cfgetispeed(&options);

    __DBG("=======DBG=======\n");

    __DBG("BaudRate:%d ",rate);
    if(rate == B115200){
        __DBG("115200\n");
    }
    __DBG("\n");

    __DBG("parityEn:%d\n",(options.c_cflag & PARENB));
    __DBG("parity  :%d\n",(options.c_cflag & PARODD));
    __DBG("InpackEn:%d\n",(options.c_iflag & INPCK));
    __DBG("ISTRIPEn:%d\n",(options.c_iflag & ISTRIP));

    __DBG("\n");
    __DBG("dataWid :%d ",(options.c_cflag & CSIZE));
    if((options.c_cflag & CSIZE) == CS5){
        __DBG("CS5\n");
    }else if((options.c_cflag & CSIZE) == CS6){
        __DBG("CS6\n");
    }else if((options.c_cflag & CSIZE) == CS7){
        __DBG("CS7\n");
    }else if((options.c_cflag & CSIZE) == CS8){
        __DBG("CS8\n");
    }else{
        __DBG("None\n");
    }

    __DBG("stopBit :%d ",(options.c_cflag & CSTOPB));
    if((options.c_cflag & CSTOPB)){
        __DBG("2Bits\n");
    }else{
        __DBG("1Bits\n");
    }
    __DBG("=======END=======\n");
}

#endif


