#include "gpio.h"
#include "clock.h"
#include "stdio.h"
#include "log.h"
//#include "2.13bwr.h"
//#include "bw_data.h"
//#include "red_data.h"
//#include "bagua.h"
//#include "AiPi.h"
//#include "test.h"
//#include "xs.h"

/*宏定义*/
#define Source_Pixel  128    
#define Gate_Pixel    250  

#define EPD_W21_RST_0 hal_gpio_write(P20, 0)
#define EPD_W21_RST_1 hal_gpio_write(P20, 1)

#define isEPD_W21_BUSY (hal_gpio_read(P18))

#define delay_us WaitUs
#define delay_ms WaitMs

#define PIC 	0
#define BLACK 1
#define WHITE 2
#define RED 	3

/*函数声明*/
void EPD_GPIO_init(void);
void EPD_GPIO_LP(void);
//void GPIO_Configuration(void);			//GPIO初始化
void EPD_Reset_and_Init(void);			//EPD复位及初始化
void EPD_refresh(void);					//驱动刷新函数
void EPD_sleep(void);					//睡眠函数
void lcd_chkstatus(void);				//忙碌检测函数
void EPD_Display(uint8_t NUM, const uint8_t *BW_datas,const uint8_t *RED_datas);		//刷屏函数
void EPD_W21_WriteCMD(unsigned char command);
void EPD_W21_WriteDATA(unsigned char data);
unsigned char EPD_read(void);
void EPD_3color(void);
void EPD_reset_refresh(char *data);

extern int axk_send_notify(uint8_t *buf, uint8_t len);
extern void reset_Transmit_timer(uint32_t Ms);
extern void stop_Transmit_timer(void);


//uint8_t BW_datas[] = "BW_datas";
//uint8_t RED_datas[] = "RED_datas";
extern unsigned char BW_datas[];
extern unsigned char RED_datas[];
//unsigned char BW_datas[200];
//unsigned char RED_datas[200];
/*********************************************************************************************
示例流程
复位 --> 初始化 -->刷新图片 --> 睡眠（退出睡眠需要重新复位初始化，如不需要频繁复位初始化可以将睡眠去除）
图片数据大小为Source_Pixel*Gate_Pixel/8byte，即1bit表示1个像素点；
*********************************************************************************************/
int EPF_test(void)
{	 
	uint8_t data = 0;
	
	EPD_GPIO_init();
	EPD_Reset_and_Init();
	EPD_W21_WriteCMD(0x2f);	//status bit read，发送目标寄存器地址
	data = EPD_read();
	LOG_DEBUG("read data:0x%x\r\n", data);
//	EPD_Reset_and_Init();
//	while(1){
//		LOG_DEBUG("EPD_3color_test\r\n");
//		EPD_3color();
//		delay_ms(20000);
//	}
	while(1)
	{	
		LOG_DEBUG("EPF_PIC_test\r\n");
		EPD_Reset_and_Init();                	 //复位及初始化				 
		EPD_Display(PIC,BW_datas,RED_datas);	 //PIC:三色图, BW_datas:黑白数据,RED_datas:红色数据 
		LOG_DEBUG("refresh end\r\n");
		delay_ms(10000);
		//KEY_SW(5);
		LOG_DEBUG("red\r\n");
		EPD_Reset_and_Init();
		EPD_Display(RED,BW_datas,RED_datas);	//RED:全红
		delay_ms(10000);
		//KEY_SW(5);
		LOG_DEBUG("BLACK\r\n");
		EPD_Reset_and_Init();
		EPD_Display(BLACK,BW_datas,RED_datas); //BLACK:全黑
		delay_ms(10000);
		//KEY_SW(5);
		LOG_DEBUG("WHITE\r\n");
		EPD_Reset_and_Init();
		EPD_Display(WHITE,BW_datas,RED_datas); //WHITE:全白
		delay_ms(10000);
		//KEY_SW(5);
	}
}


//相关宏定义
#define CMD_START_TRANSMIT_BW 0x10
#define CMD_START_TRANSMIT_RW 0x11
#define CMD_TRANSMIT_PHOTO 0x20
#define CMD_TRANSMIT_DONE 0x21
#define CMD_TRANSMIT_STOP 0x22
#define CMD_REFRESH_PHOTO 0x30

#define RESPON_OK	0
#define RESPON_PHOTO_TRANSMIT_OK	1
#define RESPON_FRAME_LEN_ERROR		2
#define RESPON_STATUS_ERROR				3
#define RESPON_PHOTO_BUF_FULL			4
#define RESPON_CMD_ERROR					5
#define RESPON_PAYLOAD_LEN_ERROR	6
#define RESPON_FRAME_CHECK_ERROR 	7
#define RESPON_TRANSMIT_TIMEOUT		8


#define TRANSMIT_IDLE 0
#define TRANSMITING_BW 1
#define TRANSMITING_RW 2

#define PHOTO_MAX_SIZE 4000	//Source_Pixel*Gate_Pixel/8
#define FRAME_MAC_PAYLOAD_LEN 200

#define TRANSMIT_TIMROUT_MS	8000
//图传结构体
typedef struct{
	uint8_t status;
	uint8_t *pdata;
	uint16_t data_len;
	uint16_t writen_len;
	
	
} photo_transmit_t;
static photo_transmit_t photo_transmit;



//api
//校验
unsigned char MakeCheckSum8(unsigned char *pData, unsigned short len)
{
    unsigned char ucCheckSum = 0;
    unsigned char ucNum;
    unsigned char *pucDat = (unsigned char *)pData;

    // 以1byte为单位依次相加
    for(ucNum=0; ucNum<len; ucNum++){
        ucCheckSum += pucDat[ucNum];
    }

    // 二进制求反码，并加1，得到校验和
    ucCheckSum = ~ucCheckSum;
    ucCheckSum++;

    return ucCheckSum;
}

unsigned char CheckData8(unsigned char *pData, unsigned short len)
{
    unsigned char ucCheckSum = 0;
    unsigned char ucNum;
    unsigned char *pucDat = (unsigned char *)pData;

    for(ucNum=0; ucNum<len ;ucNum++){
        ucCheckSum += pucDat[ucNum];
    }

    return ucCheckSum;
}
//图片缓冲区覆盖，黑白和红白
//返回0成功, 1 buf满
uint8_t write_photo_buf(uint8_t *pData, uint16_t dataLen, uint16_t write_total_len)
{
	uint16_t WriteLen = 0;
	
	if(write_total_len >= PHOTO_MAX_SIZE){
		return 1;
	}
	if(write_total_len + dataLen > PHOTO_MAX_SIZE){
		WriteLen = PHOTO_MAX_SIZE - write_total_len;
	}
	else{
		WriteLen = dataLen;
	}
	if(photo_transmit.status == TRANSMITING_RW){
		memcpy(&RED_datas[write_total_len], pData, WriteLen);
	}
	else{
		memcpy(&BW_datas[write_total_len], pData, WriteLen);
	}
	
	return 0;
}
//写屏图片数据寄存器，黑白和红白
//返回0成功, 1 buf满
uint8_t writeData2EPD(uint8_t *pData, uint16_t dataLen, uint16_t writen_len)
{
	uint16_t T0WriteLen = 0;
	
	if(writen_len >= PHOTO_MAX_SIZE){
		return 1;
	}
	if(writen_len + dataLen > PHOTO_MAX_SIZE){
		T0WriteLen = PHOTO_MAX_SIZE - writen_len;
	}
	else{
		T0WriteLen = dataLen;
	}
	if(photo_transmit.status == TRANSMITING_RW){
		for(int i=0;i<T0WriteLen;i++){		
			EPD_W21_WriteDATA(pData[i]);//BW data
		}
		
	}
	else{
		for(int i=0;i<T0WriteLen;i++){		
			EPD_W21_WriteDATA(pData[i]);//BW data
		}
	}
	
	return 0;
}

//发送应答
static uint8_t FrameBuf[10] = {0};
static int send_respon(uint8_t cmd, uint8_t result)
{
//	uint8_t FrameBuf[10] = {0};
	
	//组应答帧
	FrameBuf[0] = cmd;
	FrameBuf[3] = result;
	FrameBuf[4] = 1;	//负载长度
	FrameBuf[5] = 0;
	FrameBuf[1] = 7;	//帧总长度
	FrameBuf[2] = 0;
	
	FrameBuf[6] = MakeCheckSum8(FrameBuf, 6);	//校验和
	//发送
	return axk_send_notify(FrameBuf, FrameBuf[1]);
}
void test_copy(void)
{
	AT_LOG("BW_datas old10:");
	for(int i=0; i<10; i++){
		AT_LOG("%02x ", BW_datas[i]);
	}
	AT_LOG("\r\n");
	for(int i=0; i<10; i++){
		memcpy(BW_datas, "1234567890", 9);
	}
	AT_LOG("BW_datas new10:");
	for(int i=0; i<10; i++){
		AT_LOG("%02x ", BW_datas[i]);
	}
	AT_LOG("\r\n");
	delay_ms(1000);
}

void TransmitTimeOut(void)
{
	memset(&photo_transmit, 0, sizeof(photo_transmit));
	send_respon(0x20 ,RESPON_TRANSMIT_TIMEOUT);
}
//墨水屏初始化
int EPD_init(void)
{
	uint8_t data = 0;
	
	EPD_GPIO_init();
	EPD_Reset_and_Init();
	EPD_W21_WriteCMD(0x2f);	//status bit read，发送目标寄存器地址
	data = EPD_read();
	AT_LOG("read data:0x%x\r\n", data);
	EPD_sleep();
	EPD_GPIO_LP();
}
//蓝牙数据接收处理
int EPD_cmd_process(uint8_t *data, uint32_t size)
{
	uint8_t CheckSum = 0;
	uint16_t frame_len = 0;
	
	for(int i=0; i<size; i++){
		AT_LOG("%02x", data[i]);
	}
	AT_LOG("\r\n");
	
//	CheckSum = MakeCheckSum8(data, size-1);
//	if(CheckSum != data[size-1]){
//		//发送帧校验失败回应
//		send_respon(data[0] ,RESPON_FRAME_CHECK_ERROR);
//		return 7;
//	}
	frame_len = data[1] + (data[2]<<8);
	if(size != frame_len){
		//发送帧长度错误回应
		send_respon(data[0] ,RESPON_FRAME_LEN_ERROR);
		return 2;
	}
	switch(data[0]){
		case CMD_START_TRANSMIT_BW: 
				memset(&photo_transmit, 0, sizeof(photo_transmit));
				photo_transmit.status = TRANSMITING_BW;
				reset_Transmit_timer(TRANSMIT_TIMROUT_MS);
//				EPD_GPIO_init();
//				EPD_Reset_and_Init();
//				EPD_W21_WriteCMD(0x24);		//BW datas  		
			break;
		case CMD_START_TRANSMIT_RW:	
				memset(&photo_transmit, 0, sizeof(photo_transmit));
				photo_transmit.status = TRANSMITING_RW;
				reset_Transmit_timer(TRANSMIT_TIMROUT_MS);
//				EPD_Reset_and_Init();
//				EPD_W21_WriteCMD(0x26);			//new or Red data 	
			break;
		case CMD_TRANSMIT_PHOTO: 
			if(photo_transmit.status == TRANSMIT_IDLE){
				//发送状态错误回应
				send_respon(data[0] ,RESPON_STATUS_ERROR);
				return 3;
			}
			photo_transmit.pdata = &data[3];
			photo_transmit.data_len = data[size-3] + (data[size-2]<<8);
			if(photo_transmit.data_len > FRAME_MAC_PAYLOAD_LEN){
				send_respon(data[0] ,RESPON_PAYLOAD_LEN_ERROR);
				return RESPON_PAYLOAD_LEN_ERROR;
			}
			if(write_photo_buf(photo_transmit.pdata, photo_transmit.data_len, photo_transmit.writen_len) == 0){
//			if(writeData2EPD(photo_transmit.pdata, photo_transmit.data_len, photo_transmit.writen_len) == 0){
				photo_transmit.writen_len += photo_transmit.data_len;
				if(photo_transmit.writen_len >= PHOTO_MAX_SIZE){
					stop_Transmit_timer();
//					EPD_refresh();	//Refresh，驱动刷屏
//					EPD_sleep();		//Sleep 如果有进入睡眠，下次刷新需要复位初始化	 
					photo_transmit.status = TRANSMIT_IDLE;
					send_respon(data[0] ,RESPON_PHOTO_TRANSMIT_OK);
					return RESPON_PHOTO_TRANSMIT_OK;
				}
			}			
			else{
				stop_Transmit_timer();
				//发送图片缓冲区满回应
				send_respon(data[0] ,RESPON_PHOTO_BUF_FULL);
				photo_transmit.status = TRANSMIT_IDLE;
				return 4;
			}
			photo_transmit.pdata = NULL;
			reset_Transmit_timer(TRANSMIT_TIMROUT_MS);
			break;
		case CMD_TRANSMIT_DONE:
//			EPD_refresh();	//Refresh，驱动刷屏
//			EPD_sleep();		//Sleep 如果有进入睡眠，下次刷新需要复位初始化	 
			stop_Transmit_timer();
			photo_transmit.status = TRANSMIT_IDLE;
			send_respon(data[0] ,RESPON_PHOTO_TRANSMIT_OK);
			return RESPON_PHOTO_TRANSMIT_OK;
			break;
		case CMD_TRANSMIT_STOP: 
				stop_Transmit_timer();
				memset(&photo_transmit, 0, sizeof(photo_transmit));
				photo_transmit.status = TRANSMIT_IDLE;
			break;
		case CMD_REFRESH_PHOTO: 
			EPD_reset_refresh("5");
//			EPD_GPIO_init();
//			EPD_refresh();	//Refresh，驱动刷屏
//			EPD_sleep();		//Sleep 如果有进入睡眠，下次刷新需要复位初始化	 
			//屏断电，io 低功耗配置
			break;
		case 0x69:
			test_copy();
//			EPD_reset_refresh("6");
			break;
		case 0x66:
			EPD_reset_refresh("5");
			break;
		
		default: 
			send_respon(data[0] ,RESPON_CMD_ERROR);
			return 5;
		
	}
	
	send_respon(data[0] ,RESPON_OK);
	return 0;
}
void EPD_reset_refresh(char *data)
{
		EPD_GPIO_init();
	
	if(data[0] == '1'){
		EPD_Reset_and_Init();
		EPD_Display(RED,BW_datas,RED_datas);	//RED:全红
		LOG_DEBUG("refresh red end\r\n");
	}
	else if(data[0] == '2'){
		EPD_Reset_and_Init();
		EPD_Display(WHITE,BW_datas,RED_datas); //WHITE:全白
		LOG_DEBUG("refresh white end\r\n");
	}
	else if(data[0] == '3'){
		EPD_Reset_and_Init();
		EPD_Display(BLACK,BW_datas,RED_datas); //BLACK:全黑
		LOG_DEBUG("refresh black end\r\n");
	}
	else if(data[0] == '4'){
		EPD_Reset_and_Init();
		EPD_3color();
		LOG_DEBUG("refresh 3color end\r\n");
	}
	else if(data[0] == '5'){
		EPD_Reset_and_Init();                	 //复位及初始化				 
		EPD_Display(PIC,BW_datas,RED_datas);	 //PIC:三色图, BW_datas:黑白数据,RED_datas:红色数据 
		LOG_DEBUG("refresh end\r\n");
	}
//	else if(data[0] == '6'){
//		EPD_Reset_and_Init();                	 //复位及初始化				 
//		EPD_Display(PIC,BW_datas,gImage_AiPi);	 //PIC:三色图, BW_datas:黑白数据,RED_datas:红色数据 
//		LOG_DEBUG("refresh end\r\n");
//	}
	else{
		LOG_DEBUG("cmd error\r\n");
	}
	
	
//	if(memcmp(data, "red", 3) == 0){
//		EPD_Reset_and_Init();
//		EPD_Display(RED,BW_datas,RED_datas);	//RED:全红
//		LOG_DEBUG("refresh red end\r\n");
//	}
//	else if(memcmp(data, "white", 5) == 0){
//		EPD_Reset_and_Init();
//		EPD_Display(WHITE,BW_datas,RED_datas); //WHITE:全白
//		LOG_DEBUG("refresh white end\r\n");
//	}
//	else if(memcmp(data, "black", 5) == 0){
//		EPD_Reset_and_Init();
//		EPD_Display(BLACK,BW_datas,RED_datas); //BLACK:全黑
//		LOG_DEBUG("refresh black end\r\n");
//	}
//	else if(memcmp(data, "3color", 6) == 0){
//		EPD_Reset_and_Init();
//		EPD_3color();
//		LOG_DEBUG("refresh 3color end\r\n");
//	}
//	else{
//		LOG_DEBUG("cmd error\r\n");
//	}

	
}


/*GPIO初始化*/
void EPD_GPIO_init(void)
{
	//SPI
	hal_gpio_pin_init(P32, GPIO_OUTPUT);	//CS
	hal_gpio_pin_init(P33, GPIO_OUTPUT);	//SCL
	hal_gpio_pin_init(P34, GPIO_OUTPUT);	//MOSI
	
	hal_gpio_pin_init(P26, GPIO_OUTPUT);	//DC
	hal_gpio_pin_init(P20, GPIO_OUTPUT);	//reset
	hal_gpio_pin_init(P18, GPIO_INPUT);		//busy
	
	hal_gpio_pin_init(P11, GPIO_OUTPUT);	//vcc en
	hal_gpio_write(P11, 0);
}

void EPD_GPIO_LP(void)
{
//		hal_gpio_pull_set(GPIO_P32, GPIO_PULL_DOWN);
//		hal_gpio_pull_set(GPIO_P33, GPIO_PULL_DOWN);
//		hal_gpio_pull_set(GPIO_P34, GPIO_PULL_DOWN);
//		hal_gpio_pull_set(P26, GPIO_PULL_DOWN);
//		hal_gpio_pull_set(P20, GPIO_PULL_DOWN);
//		hal_gpio_pull_set(P18, GPIO_PULL_DOWN);
	
	hal_gpio_write(P32, 0);
	hal_gpio_write(P33, 0);
	hal_gpio_write(P34, 0);
	hal_gpio_write(P26, 0);
	hal_gpio_write(P20, 0);
	hal_gpio_write(P18, 0);
	hal_gpio_pull_set(P11, GPIO_PULL_UP_S);
	hal_gpio_write(P11, 1);
}


/*EPD复位及初始化*/
void EPD_Reset_and_Init(void)
{
	EPD_W21_RST_0;					// Module reset
	delay_ms(100);					//At least 10ms delay 
	EPD_W21_RST_1;
	delay_ms(100);					//At least 10ms delay 
	lcd_chkstatus();
	LOG_DEBUG("EPD_Reset_and_Init,lcd_chkstatus\r\n");
	EPD_W21_WriteCMD (0x12); //SW RESET
	
	lcd_chkstatus();
//	LOG_DEBUG("EPD_Reset_and_Init,lcd_chkstatus2\r\n");
	EPD_W21_WriteCMD (0x01); //驱动输出控制      
	EPD_W21_WriteDATA(0xF9);
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(0x00);

	EPD_W21_WriteCMD (0x11); //数据输入模式（扫描数据模式）      
	EPD_W21_WriteDATA(0x01);

	EPD_W21_WriteCMD (0x44); //设RAM X 的开始和结束方向      
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(0x0F); //0x0F-->(15+1)*8=128

	EPD_W21_WriteCMD (0x45); //设RAM Y 的开始和结束方向          
	EPD_W21_WriteDATA(0xF9); //0xF9-->(249+1)=250
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(0x00); 

	EPD_W21_WriteCMD (0x3C); //边框波形 BorderWavefrom
	EPD_W21_WriteDATA(0x05);	
	
	EPD_W21_WriteCMD (0x18); //温度感应器控制
	EPD_W21_WriteDATA(0x80); //80：IC内部感应器   48：外部感应器	
	
	EPD_W21_WriteCMD (0x22);
	EPD_W21_WriteDATA(0xB1); //B1

	EPD_W21_WriteCMD (0x21); 
	EPD_W21_WriteDATA(0x00);//(0x80); //第一个数字(0或8反红色数据) 第二个数字(0或8反黑白数据)
	EPD_W21_WriteDATA(0x80);
	
	EPD_W21_WriteCMD (0x4E); //设RAM X的初始地址
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteCMD (0x4F); //设RAM Y的初始地址 
	EPD_W21_WriteDATA(0xF9); 
	EPD_W21_WriteDATA(0x00);
	
	EPD_W21_WriteCMD (0x20);
	lcd_chkstatus();
}



/********************************************
函数功能:刷屏函数
函数参数:NUM: PIC，刷图片     BLACK，刷纯黑色
				WHITE，刷纯白色  RED，刷纯红色
				BW_datas:图片的黑白图层的取模数据
				RED_datas:图片的红色图层的取模数据
********************************************/
void EPD_3color(void)
{
	uint32_t i;
	
	EPD_W21_WriteCMD(0x24);
	for(i=0; i<4000/3; i++){
		EPD_W21_WriteDATA(0x00);
	}
	for(i=0; i<4000/3; i++){
		EPD_W21_WriteDATA(0xff);
	}
	for(i=0; i<4000/3; i++){
		EPD_W21_WriteDATA(0x00);
	}
	
	EPD_W21_WriteCMD(0x26);
	for(i=0; i<4000/3; i++){
		EPD_W21_WriteDATA(0x00);
	}
	for(i=0; i<4000/3; i++){
		EPD_W21_WriteDATA(0x00);
	}
	for(i=0; i<4000/3; i++){
		EPD_W21_WriteDATA(0xff);
	}
	EPD_refresh();
}
void EPD_Display(uint8_t NUM, const uint8_t *BW_datas,const uint8_t *RED_datas)
{
	unsigned int i;	
	/*送黑白图层数据*/
	EPD_W21_WriteCMD(0x24);		//BW datas  							
	for(i=0;i<Source_Pixel*Gate_Pixel/8;i++)
	{            
		switch (NUM)
		{
			case PIC:
			EPD_W21_WriteDATA(*BW_datas);//BW data
			BW_datas++;
			break;
			
			case BLACK:
			EPD_W21_WriteDATA(0x00);
			break;
			
			case WHITE:
			EPD_W21_WriteDATA(0xFF);
			break;
			
			case RED:
			EPD_W21_WriteDATA(0x00);
			break;
		}
	} 
	/*送红色图层数据*/
	EPD_W21_WriteCMD(0x26);			//new or Red data 
	for(i=0;i<Source_Pixel*Gate_Pixel/8;i++)
	{        
		switch (NUM)
		{
			case PIC:
			EPD_W21_WriteDATA(*RED_datas);//RED data
			RED_datas++;
			break;
			
			case BLACK:
			EPD_W21_WriteDATA(0xFF);
			break;
			
			case WHITE:
			EPD_W21_WriteDATA(0xFF);
			break;
			
			case RED:
			EPD_W21_WriteDATA(0x00);
			break;
		}		 
	}
	EPD_refresh();	//Refresh，驱动刷屏
	EPD_sleep();		//Sleep 如果有进入睡眠，下次刷新需要复位初始化	 
	EPD_GPIO_LP();	
}


/*EPD开始刷屏函数*/
void EPD_refresh(void)
{											
	EPD_W21_WriteCMD (0x22);  	
	EPD_W21_WriteDATA(0xCf);//(0xC7);
	EPD_W21_WriteCMD (0x20);				
	lcd_chkstatus();		
	AT_LOG("func:EPD_refresh, done\r\n");
}	


/*进入睡眠函数，退出睡眠需要硬件复位，
复位后需要重新对屏进行初始化*/
void EPD_sleep(void)
{
		EPD_W21_WriteCMD (0x10);  	//deep sleep
		EPD_W21_WriteDATA(0x01);
}


/*忙碌检测函数，读取busy PIN脚电平判断屏是否处于繁忙状态*/
void lcd_chkstatus(void)
{
	uint16_t timeOut = 0;
	
  while(1)
  {	 //=1 BUSY
     if(isEPD_W21_BUSY==0) {
			 break;
		 }
		 if(timeOut++ > 25000){
			 AT_LOG("func:lcd_chkstatus, timeOut\r\n");
			 break;
		 }
		 delay_ms(1);
  }                    
}

/*************************软件模拟SPI***********************************************************/

#define EPD_W21_MOSI_0	hal_gpio_write(P34, 0)
#define EPD_W21_MOSI_1	hal_gpio_write(P34, 1)

#define EPD_W21_CLK_0	hal_gpio_write(P33, 0)
#define EPD_W21_CLK_1	hal_gpio_write(P33, 1)

#define EPD_W21_CS_0	hal_gpio_write(P32, 0)
#define EPD_W21_CS_1	hal_gpio_write(P32, 1)

#define EPD_W21_DC_0	hal_gpio_write(P26, 0)
#define EPD_W21_DC_1	hal_gpio_write(P26, 1)

//SPI发送一字节函数
void SPI_Write(unsigned char value)                                    
{                                                           
	unsigned char i;
	
	for(i=0; i<8; i++)   
	{
		EPD_W21_CLK_0;
	
		if(value & 0x80)
			EPD_W21_MOSI_1;
		else
			EPD_W21_MOSI_0;		
		value = (value << 1);
		EPD_W21_CLK_1;
			
	}
}
/*SPI写命令函数*/
void EPD_W21_WriteCMD(unsigned char command)
{
	
  	EPD_W21_CS_0;                   
	EPD_W21_DC_0;		// command write
	SPI_Write(command);
	EPD_W21_CS_1;
	EPD_W21_DC_1;		//
}
/*SPI写数据函数*/
void EPD_W21_WriteDATA(unsigned char data)
{
	
	EPD_W21_MOSI_0;
  	EPD_W21_CS_0;                   
	EPD_W21_DC_1;		// data write
	SPI_Write(data);
	EPD_W21_CS_1;
	EPD_W21_DC_1;		//
	EPD_W21_MOSI_0;
}

#define SDA_IN()  hal_gpio_pin_init(P34, GPIO_INPUT)
#define SDA_OUT() hal_gpio_pin_init(P34, GPIO_OUTPUT)
#define READ_SDA  hal_gpio_read(P34)
/********************************************************************
读取示例：
	u8 data = 0;
	EPD_W21_WriteCMD(0x2f);	//status bit read，发送目标寄存器地址
	data = EPD_read();	//接收回读数据

如使用库函数不能正常读取，检查时序是否相符，尤其DC引脚的电平变化
********************************************************************/
unsigned char EPD_read(void)
{
	unsigned char i;
	unsigned char DATA_BUF;   

	EPD_W21_CS_0; 
  delay_us(5);  
	SDA_IN();				//数据脚设置为输入，不同平台不同引脚配置不同，可封装函数或宏
	EPD_W21_DC_1;		// data read
  delay_us(5);
	EPD_W21_CLK_0;
	delay_us(5);
  for(i=0;i<8;i++ )
	{
		DATA_BUF=DATA_BUF<<1;
		DATA_BUF|=READ_SDA;
		EPD_W21_CLK_1;
		delay_us(5);			 
		EPD_W21_CLK_0;	
    delay_us(5);			 
  }					 
  delay_us(5);		 
	EPD_W21_CS_1;
	EPD_W21_DC_1;			
	SDA_OUT();		//读取结束，数据脚设置成输出			
  return DATA_BUF;				
}

