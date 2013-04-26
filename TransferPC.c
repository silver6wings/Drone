#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"     // Device Headerfile and Examples Include File

extern float fAccX_IG500N;  		// 加速度
extern float fAccY_IG500N;  		// 加速度
extern float fAccZ_IG500N;  		// 加速度

extern float fMagX_IG500N;        // 磁强计
extern float fMagY_IG500N;        // 磁强计
extern float fMagZ_IG500N;        // 磁强计

extern float fGyrX_IG500N;        // 角速度
extern float fGyrY_IG500N;        // 角速度
extern float fGyrZ_IG500N;        // 角速度

extern float fPitch_IG500N;        // 俯仰角
extern float fRoll_IG500N;	    	// 滚转角
extern float fYaw_IG500N;			// 航向角
extern float fYaw;
extern int32 i32NavVn_IG500N;        //导航仪
extern int32 i32NavVe_IG500N;
extern int32 i32NavVd_IG500N;
extern int32 i32NavH_IG500N;

extern int32 i32BaroAltitude_IG500N;  //高度计

extern long double dGpsLatitude_IG500N;  //GPS
extern long double dGpslongitude_IG500N;
extern long double dGpsHeight_IG500N;

extern float fVecX_IG500N;         // 速度
extern float fVecY_IG500N;	        // 速度
extern float fVecZ_IG500N;		    // 速度


extern Uint32 u32PitchDuty;
extern Uint32 u32RollDuty;
extern Uint32 u32YawDuty;

extern Uint16 u16UtcTimeHour;
extern Uint16 u16UtcTimeMinute;
extern Uint16 u16UtcTimeSecond;

extern Uint16 u16SatelliteNum;
extern Uint16 u16GpsFixInformation;

void  TELE_FrameA (BYTE Buf[]);
void  TELE_FrameB (BYTE Buf[]);
void  TELE_FrameC (BYTE Buf[]);
void  MakeCheckSum (BYTE Buf[], BYTE len);
//******************************************************************

void  TELE_TaskTx (void)
{
    int   i;
    BYTE  Buf[36];
    TELE_FrameA(Buf);
	for(i=0; i<36; i++)
    {
       while (ScicRegs.SCICTL2.bit.TXRDY != 1) {}
 	   ScicRegs.SCITXBUF=Buf[i];     // Send data
	}
	
	TELE_FrameB(Buf);
	for(i=0; i<36; i++)
    {
       while (ScicRegs.SCICTL2.bit.TXRDY != 1) {}
 	   ScicRegs.SCITXBUF=Buf[i];     // Send data
	}
	
	TELE_FrameC(Buf);
	for(i=0; i<36; i++)
    {
       while (ScicRegs.SCICTL2.bit.TXRDY != 1) {}
 	   ScicRegs.SCITXBUF=Buf[i];     // Send data
	}
	
  
}

void  TELE_FrameA (BYTE Buf[])
{
	union  { BYTE B[4]; short D[2]; WORD W[2]; long DW; } src;
	int i;
//	Uint32 m;
//	m=0x00AA00BB;
    Buf[0]   = 0xEB;//帧头
    Buf[1]   = 0x90;
    Buf[2]   = 0x55;                                                                  
    Buf[3]   = 'A';                                                                                                                  /*[任务遥控指令]*/
    src.D[0] = (short)(fPitch_IG500N*10.0f);   Buf[4]=src.W[0]>>8;  Buf[5]=src.W[0];  
    src.D[0] = (short)(fRoll_IG500N*10.0f);    Buf[6]=src.W[0]>>8;  Buf[7]=src.W[0];
    src.D[0] = (short)(fYaw*10.0f);            Buf[8]=src.W[0]>>8;  Buf[9]=src.W[0];
    
//    src.DW = m;     Buf[8]=src.B[1]>>8;  Buf[9]=src.B[1];Buf[10]=src.B[0]>>8;Buf[11]=src.B[0];
    
    src.D[0] = (short)(fGyrX_IG500N*10.0f);    Buf[10]=src.W[0]>>8;  Buf[11]=src.W[0];                    
    src.D[0] = (short)(fGyrY_IG500N*10.0f);    Buf[12]=src.W[0]>>8;  Buf[13]=src.W[0];
    src.D[0] = (short)(fGyrZ_IG500N*10.0f);    Buf[14]=src.W[0]>>8;  Buf[15]=src.W[0];
    
    src.D[0] = (short)(fAccX_IG500N*10.0f);    Buf[16]=src.W[0]>>8;  Buf[17]=src.W[0];
    src.D[0] = (short)(fAccY_IG500N*10.0f);    Buf[18]=src.W[0]>>8;  Buf[19]=src.W[0];
    src.D[0] = (short)(fAccZ_IG500N*10.0f);    Buf[20]=src.W[0]>>8;  Buf[21]=src.W[0];
    
    //舵量
    for(i=22;i<=30;i++) Buf[i] = 0;
  
    src.B[0] =  (BYTE)(u16UtcTimeHour);            Buf[31]=src.B[0];
    src.B[0] =  (BYTE)(u16UtcTimeMinute);          Buf[32]=src.B[0];
    src.B[0] =  (BYTE)(u16UtcTimeSecond);          Buf[33]=src.B[0];
    Buf[34] = 0;                                                
    MakeCheckSum(Buf, 36);
}

void  TELE_FrameB (BYTE Buf[])
{
	union  { BYTE B[4]; short D[2]; WORD W[2]; long DW; } src;
	int i;
    Buf[0]   = 0xEB;//帧头
    Buf[1]   = 0x90;
    Buf[2]   = 0x55; 
    Buf[3]   = 'B';
   
    src.DW   = (long)(dGpslongitude_IG500N*1e7);    Buf[4]=src.B[1]>>8;  Buf[5]=src.B[1];   /*[经度]*/
                                                    Buf[6]=src.B[0]>>8;  Buf[7]=src.B[0];   
    src.DW   = (long)(dGpsLatitude_IG500N*1e7);     Buf[8]=src.B[1]>>8;  Buf[9]=src.B[1];   /*[纬度]*/
                                                    Buf[10]=src.B[0]>>8; Buf[11]=src.B[0];       
	src.B[0] = (BYTE)((sqrt((i32NavVd_IG500N)^2+(i32NavVe_IG500N)^2+(i32NavVn_IG500N)^2))/20);
													Buf[12]=src.B[0]; //地速 	
	for(i=13;i<=14;i++) Buf[i] = 0;//地速方向
	
	src.W[0] = (WORD)(dGpsHeight_IG500N*10.0);     Buf[15]=src.B[0]>>8;  Buf[16]=src.B[0];   /*[高]*/
    src.D[0] = (short)(i32NavVd_IG500N/10);        Buf[17]=src.B[0]>>8;  Buf[18]=src.B[0];   //Gps升降速度                                      
	
	src.B[0] = (BYTE)(u16SatelliteNum);             Buf[19]=src.B[0];
	src.B[0] = (BYTE)(u16GpsFixInformation);        Buf[20]=src.B[0];
	
	src.D[0] = (short)(i32NavVe_IG500N/10);        Buf[21]=src.B[0]>>8;  Buf[22]=src.B[0]; 
	src.D[0] = (short)(i32NavVn_IG500N/10);        Buf[23]=src.B[0]>>8;  Buf[24]=src.B[0]; 
	
	
	for(i=25;i<=29;i++) Buf[i] = 0;

	Buf[30]  = 0;//备用
	
	src.B[0] =  (BYTE)(u16UtcTimeHour);            Buf[31]=src.B[0];
    src.B[0] =  (BYTE)(u16UtcTimeMinute);          Buf[32]=src.B[0];
    src.B[0] =  (BYTE)(u16UtcTimeSecond);          Buf[33]=src.B[0];
    
    Buf[34] = 0;                                                
    MakeCheckSum(Buf, 36);
}

void  TELE_FrameC(BYTE Buf[])
{
	union  { BYTE B[4]; short D[2]; WORD W[2]; long DW; } src;
	int i;
    Buf[0]   = 0xEB;//帧头
    Buf[1]   = 0x90;
    Buf[2]   = 0x55; 
    Buf[3]   = 'C';
    
    for(i=4;i<=14;i++) Buf[i] = 0;
    
    src.W[0] = (WORD)(i32BaroAltitude_IG500N*2);     Buf[15]=src.B[0]>>8;  Buf[16]=src.B[0];
	Buf[17] = 0; Buf[18] = 0;//指示空速
	Buf[19] = 0; Buf[20] = 0;//真空速
	Buf[21] = 0; Buf[22] = 0;
	Buf[23] = 0; Buf[24] = 0;
	Buf[25] = 0; Buf[26] = 0;
	Buf[27] = 0; Buf[28] = 0; Buf[29] = 0; Buf[30] = 0;
	
	src.B[0] =  (BYTE)(u16UtcTimeHour);            Buf[31]=src.B[0];
    src.B[0] =  (BYTE)(u16UtcTimeMinute);          Buf[32]=src.B[0];
    src.B[0] =  (BYTE)(u16UtcTimeSecond);          Buf[33]=src.B[0];
    
    Buf[34] = 0;                                                
    MakeCheckSum(Buf, 36);//sizeof(Buf) = 2  why??

}

void  MakeCheckSum (BYTE Buf[], BYTE len)
{
   unsigned int  sum, idx;

    sum=0;
    for(idx=0;idx<(len-1);idx++)
        sum+=Buf[idx];

    Buf[len-1] = 256 - sum%256; 
}



