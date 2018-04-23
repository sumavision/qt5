#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <sys/time.h>
#include <sched.h>
#include <time.h> 
#include <errno.h>


#include "uart_serial.h"

#ifndef false
#define false 0
#endif

#ifndef true
#define true 1
#endif

#ifndef NULL
#define NULL 0
#endif

#define RFCOMM_BUFF_MAX (10*1024)

#define MAX_LEN_SERIAL_NAME 16

/* report level and macro */
#define RPT_ERR (1) /* error, system error */
#define RPT_WRN (2) /* warning, maybe wrong, maybe OK */
#define RPT_INF (3) /* important information */
#define RPT_DBG (4) /* debug information */

#ifndef S_SPLINT_S /* FIXME */
#define RPTERR(fmt, ...) if(RPT_ERR <= rpt_lvl) fprintf(stderr, "%s: %s: %d: err: " fmt "\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__)
#define RPTWRN(fmt, ...) if(RPT_WRN <= rpt_lvl) fprintf(stderr, "%s: %s: %d: wrn: " fmt "\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__)
#define RPTINF(fmt, ...) if(RPT_INF <= rpt_lvl) fprintf(stderr, "%s: %s: %d: inf: " fmt "\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__)
#define RPTDBG(fmt, ...) if(RPT_DBG <= rpt_lvl) fprintf(stderr, "%s: %s: %d: dbg: " fmt "\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__)
static int rpt_lvl = RPT_DBG; /* report level: ERR, WRN, INF, DBG */
#else
#define RPTERR(fmt...)
#define RPTWRN(fmt...)
#define RPTINF(fmt...)
#define RPTDBG(fmt...)
#endif

typedef struct _rfcomm_trans_static_params
{
	/*now only support "ttyO" or "ttyS"*/
	char serial_name[MAX_LEN_SERIAL_NAME]; 
}rfcomm_trans_static_params;


typedef struct _rfcomm_trans_dynamic_params
{
	int baud_rate;
	int serial_id;
}rfcomm_trans_dynamic_params;


typedef struct
{
	rfcomm_trans_dynamic_params params;
	
    void *RecvBuff;
	void *SndBuff;

	char serial_name[MAX_LEN_SERIAL_NAME]; 

	int is_first;
} rfcomm_trans_obj;


typedef void * rfcomm_trans_handle;

/* sleep in ms */
void suma_mssleep(int const time_in_ms)
{   
    struct timespec time;
    struct timespec time_buf;

    int ret = -1;
    time.tv_sec = (time_in_ms / 1000);
    time.tv_nsec = (1000 * 1000 * (time_in_ms % 1000));

    time_buf = time;
    while(1 == 1) /* lint warning modified */
    {
        time = time_buf;
        ret = nanosleep(&time, &time_buf);
        if((ret < 0)&&(errno == EINTR))
            continue;
        else
            break;
    }
    return;
}


static int rfcomm_trans_uart_ioctrl_set(int h, int baud_rate)
{
    int ret = 0;
    /*38400 baud rate*/
    ret = serialIoctl(h, SL_BAUD_SET, baud_rate);
    /*8 data bits*/
    ret |= serialIoctl(h, SL_DBIT_SET, 8);
    /*no parity*/
    ret |= serialIoctl(h, SL_PAR_SET, 0);
    /*1 stop bits*/
    ret |= serialIoctl(h, SL_STB_SET, 1);

    return ret;
}

static int rfcomm_trans_uart_open(rfcomm_trans_handle handle)
{
	rfcomm_trans_obj *obj = NULL;
	rfcomm_trans_dynamic_params *params = NULL;

	char *serial_name = NULL;
	
    int serial_id = 0;
	int baud_rate = 0;
		
	serialAttr *s = NULL;

    if (NULL == handle)
	{
	    RPTERR( "trans_uart_open NULL pointer!\n");		
        return -1;
	}

	obj = (rfcomm_trans_obj*)handle;
	
	params = &(obj->params);
	serial_id = params->serial_id;
	baud_rate = params->baud_rate;

	serial_name = obj->serial_name;
    s = serialGet(serial_id);
    if(!s->opened)
	{
        if(serialOpen(0, serial_id, serial_name) < 0)
		{
            RPTERR("serialOpen@rfcomm failed..%s%d\n",serial_name,serial_id);
            return -1;
        }

		if(rfcomm_trans_uart_ioctrl_set(serial_id, baud_rate))
		{
            RPTERR("set RFCOMM 0 failed...\n");
            return -1;
        }
		RPTWRN("serial Open @RFCOMM %s%d %d success..\n",serial_name,serial_id,baud_rate);
    }
    return 0;
}

static int rfcomm_trans_uart_read_ready(int serial_fd, uint32_t msTimeOut)
{
	fd_set fd;
    struct timeval tv;

	int ret = 0;

    if(serial_fd < 2)
	{
		return(-1);
    }

    tv.tv_sec = msTimeOut/1000;
    tv.tv_usec = (msTimeOut%1000)*1000;

    FD_ZERO(&fd);
	FD_SET(serial_fd, &fd);

	ret = select(serial_fd + 1, &fd, NULL, NULL, &tv);

	return ret;
}

static int rfcomm_trans_snd_data(rfcomm_trans_obj *obj,char *buf_ptr,int buf_size)
{
	rfcomm_trans_dynamic_params *params = NULL;

	if(NULL == obj)
    {
        RPTERR("NULL pointer!");
		return -1;
    }

	params = &(obj->params);

    if (obj->is_first == false)
    {
        return 0;
    }

	serialWrite(params->serial_id,(char*)buf_ptr,buf_size);
	RPTWRN("rfcomm write data :[buff]%s,[len]%d",buf_ptr,buf_size);
	
	return 0;
}

static int rfcomm_trans_read_data(rfcomm_trans_obj *obj)  
{  
	int ret = 0;
	int rd_len = 0;
	int len = 0;

	uint8_t sync_buf;

	char *pRecv = NULL;
	
	serialAttr *s = NULL;

	rfcomm_trans_dynamic_params *params = NULL;
     
	if(NULL == obj)
    {
        RPTERR("NULL pointer!");
		return -1;
    }

	pRecv = obj->RecvBuff;
	params = &(obj->params);
	s = serialGet(params->serial_id);
    memset(pRecv,0,RFCOMM_BUFF_MAX);
	while (rfcomm_trans_uart_read_ready(s->serialFd,1) > 0 && rd_len < RFCOMM_BUFF_MAX)
	{
		RPTWRN( "serial port@serialRead start...\n");
        len = serialRead(params->serial_id, pRecv,RFCOMM_BUFF_MAX - rd_len);
		if(len < 0)
		{
			RPTWRN( "GetUcmpMsg@serialRead error...\n");
			return -1;
		} 
		
		if (len == 0)
		{
			RPTWRN( "GetUcmpMsg@serialRead error...\n");
			suma_mssleep(100);
			continue;
		}

		pRecv += len;
		rd_len += len;
	}

    if (rd_len <= 0)
    {
        return -1;
    }
    pRecv = obj->RecvBuff;
    //RPTWRN( "RFCOMM recv data: [data= %s] [len= %d]:\n",pRecv,rd_len);
    int i = 0;
    pRecv = obj->RecvBuff;
    for (i = 0;i < rd_len;i++)
    {
        RPTWRN( "%x ",pRecv[i]);
        if (i%8 == 0)
        {
            //RPTWRN( "\n ");
        }
    }
	
    return 0;  
}  

static int rfcomm_trans_process(rfcomm_trans_handle handle)
{
	rfcomm_trans_obj *obj = NULL;

	char buff[] = "sumavision ... ...";

	int ret = 0;

	if (NULL == handle)
	{
	    RPTERR( "rfcomm_trans_process NULL pointer!\n");		
        return -1;
	}

	obj = (rfcomm_trans_obj*)handle;

	//ret = rfcomm_trans_snd_data(obj,buff,sizeof(buff));
	if (ret < 0)
	{
		RPTERR( "rfcomm_trans_snd_data error! [buff]%s,[size]%d\n",buff,(int)sizeof(buff));	
		return -1;
	}
	ret = rfcomm_trans_read_data(obj);

    obj->is_first = false;

	return 0;
}


rfcomm_trans_handle rfcomm_trans_creat(rfcomm_trans_static_params *static_params,
	                                              rfcomm_trans_dynamic_params *dynamic_params)
{
	rfcomm_trans_obj *obj = NULL;
	rfcomm_trans_dynamic_params * params = NULL;

    obj = malloc(sizeof(*obj));
    if(NULL == obj || NULL == static_params)
    {
        RPTERR(" tcp_client_create NULL pointer");
        return NULL;
    }

    params = &(obj->params);
    memcpy(params, dynamic_params, sizeof(rfcomm_trans_dynamic_params));
    
    strcpy(obj->serial_name,static_params->serial_name);

	obj->is_first = true;

	obj->RecvBuff = malloc(RFCOMM_BUFF_MAX);
    if(NULL == obj->RecvBuff)
    {
        RPTWRN("Malloc RecvBuff failed!");
		return NULL;
    }

	obj->SndBuff = malloc(RFCOMM_BUFF_MAX);
    if(NULL == obj->SndBuff)
    {
        RPTWRN("Malloc SndBuff failed!");
		return NULL;
    }
	
    return obj;
}

int main (int argc, char *argv[])
{
	rfcomm_trans_static_params rfcomm_static_params;
	rfcomm_trans_dynamic_params rfcomm_dynamic_params;

	rfcomm_trans_handle rfcomm_handle = NULL;

	int ret = 0;

	
	strncpy(rfcomm_static_params.serial_name, "ttyO",MAX_LEN_SERIAL_NAME-1);

	rfcomm_dynamic_params.serial_id = 2;
	rfcomm_dynamic_params.baud_rate = 9600;

	rfcomm_handle = rfcomm_trans_creat(&rfcomm_static_params,&rfcomm_dynamic_params);
	ret = rfcomm_trans_uart_open(rfcomm_handle);
	if (ret < 0)
	{
		RPTERR("bluetooth_trans_uart uart open failed.");
		return 0;
    }

	while (true)
	{
		ret = rfcomm_trans_process( rfcomm_handle);
		if (ret < 0)
		{
			RPTERR("rfcomm_trans_process failed.");
			suma_mssleep(100);
			continue;
        }
	}
	
}
