
#include "msp430x22x4.h"
#include "TI_USCI_I2C_master.h"
#include "USCI_UART_master.h"
#include "math.h"
#include "MyMath.h"
#include "MahonyAHRS.h"
//#include "MadgwickAHRS.h"

#define ADJ_NUM 20
#define PI 3.14159265
char comma[] = ",";
//sensor register
unsigned char adxl_config[18] = { 0x31,0x0B,0x2C,0x1B,0X2E,0x00,0x38,0x00,0x2D, 0x08,0x1E,0xFF,0x1F,0x01,0x20,0x09,0x2D,0x00 };//set 0x1E,0xFC,0x1F,0x04,0x20,0x09
unsigned char array2 [3] = { 0x32,0x00,0x2C};//acc address
unsigned char itg3200[10]={0x15,0x07,0x16,0x1E,0x17,0x00,0x3E,0x01,0x3E,0x80};//set
unsigned char array3 [3] = { 0x1D,0x00,0x15};//gyo address
//send data
unsigned char DATA[20];

//for adjust and calculate
int accset[3]={0,0,0};
int gyroset[3]={0,0,0};//initial
int accValue[3]={0,0,0};
int gyroValue[3]={0,0,0};
//control
unsigned char DeviceChoose=0x03;
unsigned char SampleVelo=0x22;

//initial sensor value
unsigned char needAdjust=0x00;//left=gyro right=acc 1=startjust 0=not need adjust

//calculate sample times
int adxlsampletimes=0;
int itgsampletimes=0;
int totaltimestamp=0;

//compute angle
int acclast[3]={0,0,0};
int lasttime=0;
float RwAcc[3]={0.01f,-0.32f,0.49f};         // 通过加速度传感器把重力加速度投影在x/y/z三轴上
float Gyro[3]={0.00f,0.00f,0.00f};          // 陀螺仪读取
float dest[3]={0.0f,0.0f,0.0f};
float Awz[3]={0.0f,0.0f,0.0f};           // XZ/ YZ平面和Z轴（度）R的投影之间的角度
float RwGyro[3]={0.0f,0.0f,0.0f};        // 重新读取陀螺仪
float RwEst[3]={0.0f,0.0f,0.0f};
float interval = 0.0f;
float tmpf[3]={0.0f,0.0f,0.0f};
float wGyro=10;
int signRzGyro=1;
char firstSample=1;
int w = 0;
extern float a1,a2,a3;
//timer A control
volatile unsigned char DataFlag=0x00;//right means device sample， left means send data
volatile int dataSend=12;
int timerA_value=0;
volatile unsigned char receiveFlag;


void reset_device()
{
  while ( TI_USCI_I2C_notready() );    
  TI_USCI_I2C_transmitinit(0x53,0x14);  //reset adxl
  while ( TI_USCI_I2C_notready() );         
  TI_USCI_I2C_transmit(2,&adxl_config[16],1);      
  while ( TI_USCI_I2C_notready() );        
  
  TI_USCI_I2C_transmitinit(0x68,0x14); //reset itg3200
  while ( TI_USCI_I2C_notready() );         
  TI_USCI_I2C_transmit(2,&itg3200[8],1);       
  while ( TI_USCI_I2C_notready() );         
}

void init_adxl345()
{

  TI_USCI_I2C_transmitinit(0x53,0x14);  
  while ( TI_USCI_I2C_notready() );       
  TI_USCI_I2C_transmit(2,&adxl_config[0],1);      
  while ( TI_USCI_I2C_notready() );       
  
  TI_USCI_I2C_transmitinit(0x53,0x14);  
  while ( TI_USCI_I2C_notready() );        
  TI_USCI_I2C_transmit(2,&adxl_config[2],1);      
  while ( TI_USCI_I2C_notready() );       
  
  TI_USCI_I2C_transmitinit(0x53,0x14); 
  while ( TI_USCI_I2C_notready() );        
  TI_USCI_I2C_transmit(2,&adxl_config[4],1);     
  while ( TI_USCI_I2C_notready() );       
  
  TI_USCI_I2C_transmitinit(0x53,0x14);  
  while ( TI_USCI_I2C_notready() );         
  TI_USCI_I2C_transmit(2,&adxl_config[6],1);       
  while ( TI_USCI_I2C_notready() );        
  
  TI_USCI_I2C_transmitinit(0x53,0x14);   
  while ( TI_USCI_I2C_notready() );         
  TI_USCI_I2C_transmit(2,&adxl_config[8],1);     
  while ( TI_USCI_I2C_notready() );       
  
  TI_USCI_I2C_transmitinit(0x53,0x14);  // init transmitting with 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  TI_USCI_I2C_transmit(2,&adxl_config[10],1);       // start transmitting 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  
  TI_USCI_I2C_transmitinit(0x53,0x14);  // init transmitting with 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  TI_USCI_I2C_transmit(2,&adxl_config[12],1);       // start transmitting 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  
  TI_USCI_I2C_transmitinit(0x53,0x14);  // init transmitting with 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  TI_USCI_I2C_transmit(2,&adxl_config[14],1);       // start transmitting 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
}

void init_itg3200()
{
  TI_USCI_I2C_transmitinit(0x68,0x14);  // init transmitting with 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  TI_USCI_I2C_transmit(2,&itg3200[0],1);       // start transmitting 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  
  TI_USCI_I2C_transmitinit(0x68,0x14);  // init transmitting with 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  TI_USCI_I2C_transmit(2,&itg3200[2],1);       // start transmitting 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  
  TI_USCI_I2C_transmitinit(0x68,0x14);  // init transmitting with 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  TI_USCI_I2C_transmit(2,&itg3200[4],1);       // start transmitting 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  
  TI_USCI_I2C_transmitinit(0x68,0x14);  // init transmitting with 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
  TI_USCI_I2C_transmit(2,&itg3200[6],1);       // start transmitting 
  while ( TI_USCI_I2C_notready() );         // wait for bus to be free
}

void accSample()
{
  TI_USCI_I2C_transmitinit(0x53,0x14);
  while ( TI_USCI_I2C_notready() );
  TI_USCI_I2C_transmit(1,&array2[0],1);       
  while ( TI_USCI_I2C_notready() );         
  TI_USCI_I2C_receiveinit(0x53,0x14);    
  while ( TI_USCI_I2C_notready() );     
  TI_USCI_I2C_receive(6,DATA);
  while ( TI_USCI_I2C_notready() );  
  if (needAdjust&0x01)
  {
    accValue[0]=DATA[1]<<8|DATA[0];
    accValue[1]=DATA[3]<<8|DATA[2];
    accValue[2]=DATA[5]<<8|DATA[4];
  }else{
    accValue[0]=(DATA[1]<<8|DATA[0])-accset[0];
    accValue[1]=(DATA[3]<<8|DATA[2])-accset[1];
    accValue[2]=(DATA[5]<<8|DATA[4])-accset[2]; 
    RwAcc[0] = (accValue[0]) / 256.0;
    RwAcc[1] = (accValue[1]) / 256.0;
    RwAcc[2] = (accValue[2]) / 256.0;
  }
}

void gyroSample()
{
  TI_USCI_I2C_transmitinit(0x68,0x14);
  while ( TI_USCI_I2C_notready() );
  TI_USCI_I2C_transmit(1,&array3[0],1); 
  while ( TI_USCI_I2C_notready() );       
  TI_USCI_I2C_receiveinit(0x68,0x14);   
  while ( TI_USCI_I2C_notready() );      
  TI_USCI_I2C_receive(6,&DATA[6]);
  while ( TI_USCI_I2C_notready() );     
  if (needAdjust&0x10)
  {
    gyroValue[0]=DATA[6]<<8|DATA[7];
    gyroValue[1]=DATA[8]<<8|DATA[9];
    gyroValue[2]=DATA[10]<<8|DATA[11];
  }else
  {
    gyroValue[1]=(DATA[6]<<8|DATA[7])-gyroset[0];
    gyroValue[0]=(DATA[8]<<8|DATA[9])-gyroset[1];
    gyroValue[2]=(DATA[10]<<8|DATA[11])-gyroset[2];
    Gyro[0] = gyroValue[0] / 14.375;
    Gyro[1] = gyroValue[1] / 14.375;
    Gyro[2] = gyroValue[2] / 14.375;
    tmpf[0] += *(Gyro)*interval;
    tmpf[1] += *(Gyro+1)*interval;
    tmpf[2] += *(Gyro+2)*interval;
  }
}

void init_device()
{ 
  USCI_UART_init(); //init bluetooth module
  
  if(DeviceChoose&0x01)
    init_adxl345();
  if(DeviceChoose&0x02)
    init_itg3200();
}  

void init_gyrovalue()
{
  gyroset[0]=0;
  gyroset[1]=0;
  gyroset[2]=0;
}

void send_data()
{
  DATA[0]=accValue[0]&0x0ff;
  DATA[1]=accValue[0]>>8;
  DATA[2]=accValue[1]&0x0ff;
  DATA[3]=accValue[1]>>8;
  DATA[4]=accValue[2]&0x0ff;
  DATA[5]=accValue[2]>>8;
  int temp;
  temp=(int)(RwEst[0]*30000);
  DATA[6]=temp>>8;
  DATA[7]=temp&0x0ff;
  temp=(int)(RwEst[1]*30000);
  DATA[8]=temp>>8;
  DATA[9]=temp&0x0ff;
  temp=(int)(tmpf[2]*30000);
  DATA[10]=temp>>8;
  DATA[11]=temp&0x0ff;
  DATA[12]=totaltimestamp&0x0ff;
  DATA[13]=totaltimestamp>>8;
  UART_STR_send(DATA);
  if (totaltimestamp%100==0)
    P1OUT^=0x01;
}

//void send_data()
//{
//  DATA[0]=accValue[0]&0x0ff;
//  DATA[1]=accValue[0]>>8;
//  DATA[2]=accValue[1]&0x0ff;
//  DATA[3]=accValue[1]>>8;
//  DATA[4]=accValue[2]&0x0ff;
//  DATA[5]=accValue[2]>>8;
//  int temp;
//  temp=(int)(a1*30000);
//  DATA[6]=temp>>8;
//  DATA[7]=temp&0x0ff;
//  temp=(int)(a2*30000);
//  DATA[8]=temp>>8;
//  DATA[9]=temp&0x0ff;
//  temp=(int)(a3*30000);
//  DATA[10]=temp>>8;
//  DATA[11]=temp&0x0ff;
//  DATA[12]=totaltimestamp&0x0ff;
//  DATA[13]=totaltimestamp>>8;
//  UART_STR_send(DATA);
//  if (totaltimestamp%100==0)
//    P1OUT^=0x01;
//}

void adjustacc()
{
  if(needAdjust&0x01)
  {
    while((DataFlag&0x01)!=0x01);
    accSample();
    DataFlag&=0xFE;
    if (adxlsampletimes<=100){
      adxlsampletimes++;
    }
    else if(adxlsampletimes<ADJ_NUM+100)
    {
      adxlsampletimes++;
      accset[0]+=accValue[0];
      accset[1]+=accValue[1];
      accset[2]+=accValue[2];
    }
    else if (adxlsampletimes==ADJ_NUM+100)
    {
      accset[0]+=accValue[0];
      accset[1]+=accValue[1];
      accset[2]+=accValue[2];
      accset[0]/=ADJ_NUM;
      accset[1]/=ADJ_NUM;
      accset[2]=abs(accset[2]/ADJ_NUM)-250;
      RwAcc[0] = (accValue[0]-accset[0]) / 256.0;
      RwAcc[1] = (accValue[1]-accset[1]) / 256.0;
      RwAcc[2] = (accValue[2]-accset[2]) / 256.0;
      needAdjust&=0xFE;
      adxlsampletimes=0;
    }
  }
}

void adjustgyro()
{
  if(needAdjust&0x10)
  {
    while((DataFlag&0x02)!=0x02);
    gyroSample();
    DataFlag&=0xFD;
    if (itgsampletimes<=100){
      itgsampletimes++;
    }
    else if (itgsampletimes<ADJ_NUM+100)
    {
      itgsampletimes++;
      gyroset[0]+=gyroValue[0];
      gyroset[1]+=gyroValue[1];
      gyroset[2]+=gyroValue[2];
    }else if (itgsampletimes==ADJ_NUM+100)
    {
      gyroset[0]+=gyroValue[0];
      gyroset[1]+=gyroValue[1];
      gyroset[2]+=gyroValue[2];
      gyroset[0]/=ADJ_NUM;
      gyroset[1]/=ADJ_NUM;
      gyroset[2]/=ADJ_NUM;
      needAdjust&=0xEF;
      itgsampletimes=0;
      P1OUT^=0X02;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////


void getInclination()
{ 
  normalize3DVec(RwAcc);
  if (firstSample==1) { 
    firstSample--;
    *RwEst = *RwAcc;    
    *(RwEst+1) = *(RwAcc+1);    
    *(RwEst+2) = *(RwAcc+2);    
    *acclast=*accValue;
    *(acclast+1)=*(accValue+1);
    *(acclast+2)=*(accValue+2);
  }
  else {
    wGyro=getWgyro(accValue,accset);
    *acclast=*accValue;
    *(acclast+1)=*(accValue+1);
    *(acclast+2)=*(accValue+2);
    //need a better algorithm
    if ( *(RwEst+2) < 0.01 && *(RwEst+2) > -0.01) {
      *dest = *RwEst;
      *(dest+1) = *(RwEst+1);
      *(dest+2) = *(RwEst+2);
      //使用awy
      
    }
    else {
      //tmpf = *Gyro*interval;                             
      *(Awz) = my_atan(*(RwEst), *(RwEst+2))+tmpf[0];      
      //tmpf = *(Gyro+1)*interval;                             
      *(Awz+1) = my_atan(*(RwEst+1), *(RwEst+2))+tmpf[1];     
      *tmpf=0;
      *(tmpf+1)=0; 
      signRzGyro = ( *(RwEst+2) >=0 ) ? 1 : -1;
      float sin1=my_sin( *(Awz+1));
      float cos1=my_cos( *(Awz+1));
      float sin0=my_sin(*(Awz));
      float cos0=my_cos(*(Awz));
      *(dest) = sin0*invSqrt( 1 + my_pow(cos0) * my_pow(sin1/cos1) );
      *(dest+1) = sin1*invSqrt( 1 + my_pow(cos1) * my_pow(sin0/cos0) );
      *(dest+2) = signRzGyro*sqrt(1 - my_pow(dest[0]) - my_pow(dest[1]));
    }

    *(RwEst) = (*(RwAcc)+wGyro*(*(dest)))/(1+wGyro);
    *(RwEst+1) = (*(RwAcc+1)+wGyro*(*(dest+1)))/(1+wGyro);
    *(RwEst+2) = (*(RwAcc+2)+wGyro*(*(dest+2)))/(1+wGyro);

    normalize3DVec(RwEst);
    
  }
}



///////////////////////////////////////////////////////////////////////////////////////



int main( void )
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer to prevent time out reset
  P1DIR |= 0xFF;                            // P1.0 output
  P1OUT &= 0xFD;
  P3DIR |= 0x0F;
  P3OUT |= 0x06;
  P3SEL |= 0x06;
  
  BCSCTL1 = CALBC1_8MHZ; //DCO 8mhz max
  DCOCTL = CALDCO_8MHZ; 
  
  _EINT(); //enable int
  
  P1OUT |= 0x01; //light 
  
  reset_device(); //make device off
  
  init_device();  
  
  init_gyrovalue(); //initial gyro reset value
  
  receiveFlag=1;   //initial sensor
  
  TACCR0 = 5000;  //timerA control
  TACTL = TASSEL_2 + MC_1; 
  TACCTL0 |= CCIE; 

  DATA[14]=0;     //Data[14] is node number
  interval=0.01* PI / (180*(SampleVelo>>4)); //calculate interval

//  while(1){
//    MahonyAHRSupdateIMU(Gyro[0], Gyro[1], Gyro[2], RwAcc[0], RwAcc[1], RwAcc[2]);
//    Gyro[0]=0;
//    Gyro[1]=0;
//    Gyro[2]=0;
//
//  }
  
  while (1)
  {
    //send or adjust data
    if (needAdjust&0x11)
    {
      adjustgyro();
      adjustacc();
    }else if (DataFlag&0x10)
    {
      //MahonyAHRSupdateIMU(Gyro[0], Gyro[1], Gyro[2], RwAcc[0], RwAcc[1], RwAcc[2]);
      getInclination();
      send_data();
      DataFlag&=0xEF;
    }
    //get Data
    if (DataFlag&0x01)
    {
      accSample();
      DataFlag&=0xFE;
    }
    if (DataFlag&0x02)
    {
      gyroSample();
      DataFlag&=0xFD;
    }
    //get Uart Value
    if (receiveFlag==2)
    {
      reset_device();
      TACCTL0=~CCIE;
      timerA_value=0;
      receiveFlag=0;
      _BIS_SR(LPM1);
    }else if(receiveFlag==1)
    {
      
      firstSample=1;
      init_device();
      needAdjust|=0x11;
      TACCTL0 = CCIE;
      receiveFlag=0;
      
    }    
  } 
}
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{  
  timerA_value++;
  if(timerA_value%(16/(SampleVelo&0x0f))==0)
    DataFlag|=0x01;
  if(timerA_value%(16/(SampleVelo>>4))==0)
    DataFlag|=0x02;
  if(timerA_value==dataSend)
  {
    totaltimestamp++;
    
    DataFlag|=0x10;
    timerA_value=0;
  }
}  


