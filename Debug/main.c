
#include "msp430x22x4.h"
#include "TI_USCI_I2C_master.h"
#include "USCI_UART_master.h"
#include "math.h"


#define ADJ_NUM 20
#define PI 3.14159265
char comma[] = ",";
//sensor register
unsigned char adxl_config[18] = { 0x31,0x0B,0x2C,0x1A,0X2E,0x00,0x38,0x00,0x2D, 0x08,0x1E,0xFF,0x1F,0x01,0x20,0x09,0x2D,0x00 };//set 0x1E,0xFC,0x1F,0x04,0x20,0x09
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
unsigned char SampleVelo=0x11;

//initial sensor value
unsigned char needAdjust=0x00;//left=gyro right=acc 1=startjust 0=not need adjust

//calculate sample times
int adxlsampletimes=0;
int itgsampletimes=0;
int totaltimestamp=0;

//compute angle
int lasttime=0;
float RwAcc[5]={0.01f,0.01f,0.99f};         // 通过加速度传感器把重力加速度投影在x/y/z三轴上
float Gyro[3]={0.01f,0.01f,0.01f};          // 陀螺仪读取
double dest[3]={0.0f,0.0f,0.0f};
double Awz[3]={0.0f,0.0f,0.0f};           // XZ/ YZ平面和Z轴（度）R的投影之间的角度
double RwGyro[3]={0.0f,0.0f,0.0f};        // 重新读取陀螺仪
float RwEst[3]={0.0f,0.0f,0.0f};
float interval = 0.0f;
int signRzGyro=1;
char firstSample=1;
int w = 0;

//timer A control
volatile unsigned char DataFlag=0x00;//right means device sample， left means send data
volatile int dataSend=16;
int timerA_value=0;
volatile unsigned char receiveFlag;


void reset_device()
{
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
    accValue[0]=DATA[1]<<8|DATA[0];
    accValue[1]=DATA[3]<<8|DATA[2];
    accValue[2]=DATA[5]<<8|DATA[4]; 
    RwAcc[0] = accValue[0] / 256.0;
    RwAcc[1] = accValue[1] / 256.0;
    RwAcc[2] = accValue[2] / 256.0;
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
    gyroValue[0]=(DATA[6]<<8|DATA[7])-gyroset[0];
    gyroValue[1]=(DATA[8]<<8|DATA[9])-gyroset[1];
    gyroValue[2]=(DATA[10]<<8|DATA[11])-gyroset[2];
    /*DATA[6]=gyroValue[0]>>8;
    DATA[7]=gyroValue[0]&0x0ff;
    DATA[8]=gyroValue[1]>>8;
    DATA[9]=gyroValue[1]&0x0ff;
    DATA[10]=gyroValue[2]>>8;
    DATA[11]=gyroValue[2]&0x0ff;*/
    Gyro[0] = gyroValue[0] / 14.375;
    Gyro[1] = gyroValue[1] / 14.375;
    Gyro[2] = gyroValue[2] / 14.375;
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
  int temp;
  /*temp=(int)(RwEst[0]*30000);
  DATA[6]=temp>>8;
  DATA[7]=temp&0x0ff;
  temp=(int)(RwEst[1]*30000);
  DATA[8]=temp>>8;
  DATA[9]=temp&0x0ff;
  temp=(int)(RwEst[2]*30000);
  DATA[10]=temp>>8;
  DATA[11]=temp&0x0ff;*/

  
  DATA[12]=totaltimestamp&0x0ff;
  DATA[13]=totaltimestamp>>8;
  DATA[14]=0;
  UART_STR_send(DATA);
  if (totaltimestamp%100==0)
    P1OUT^=0x01;
}

void adjustacc()
{
  if(needAdjust&0x01)
  {
    if(adxlsampletimes==ADJ_NUM)
    {
      _NOP();
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
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void normalize3DVec(float *  vector) {
  float R;
  R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
  vector[0] /= R;
  vector[1] /= R;  
  vector[2] /= R;
}

float squared(float x) {
  return x*x;
}

void getInclination()
{
  float tmpf = 0.0f;
  normalize3DVec(RwAcc);
  interval=10.0/(SampleVelo>>4);
  if (firstSample==1) { 
    firstSample--;
    
    for (w=0;w<=2;w++) {
      RwEst[w] = RwAcc[w];    
    }
  }
  else {
    if ( RwEst[2] < 0.01&&RwEst[2]>-0.01) {
      for (w=0;w<=2;w++) {
        dest[w] = RwEst[w];
      }
    }
    else {
      for (w=0;w<=1;w++) {
        tmpf = Gyro[w];                        
        tmpf *= interval/1000.0f;                     
        Awz[w] = atan2(RwEst[w], RwEst[2]);   
        Awz[w] += tmpf * PI / 180;           
      }
      
      signRzGyro = ( RwAcc[2] >=0 ) ? 1 : -1;
      
      dest[0] = sin(Awz[0])/sqrt( 1 + squared(cos(Awz[0])) * squared(tan(Awz[1])) );
      dest[1] = sin(Awz[1])/sqrt( 1 + squared(cos(Awz[1])) * squared(tan(Awz[0])) );
      //dest[0]=1*invSqrt(1+1/(squared(tan(Awz[0]))*squared(cos(Awz[1]))));
      //dest[1]=1*invSqrt(1+1/(squared(tan(Awz[1]))*squared(cos(Awz[0]))));
      dest[2] = signRzGyro*sqrt(1 - squared(dest[0]) - squared(dest[1]));
    }
    
    for (w=0;w<=2;w++) {
      RwEst[w] = (RwAcc[w]+10*dest[w])/11;
    }
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
  
  BCSCTL1 = CALBC1_8MHZ; //DCO 16mhz max
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
  
  while(1){
    getInclination();
  }

  while (1)
  {
    //send or adjust data
    if (needAdjust&0x11)
    {
      adjustgyro();
      //adjustacc();
    }else if (DataFlag&0x10)
    {
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
      getInclination();
      //MahonyAHRSupdateIMU(Gyro[0],Gyro[1],Gyro[2],RwAcc[0],RwAcc[1],RwAcc[2]);
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
      needAdjust|=0x10;
      TACCTL0 = CCIE;
      receiveFlag=0;
    }
    
    
  } 
}
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{  
  timerA_value++;
  if(timerA_value==(16/(SampleVelo&0x0f)))
    DataFlag|=0x01;
  if(timerA_value==(16/(SampleVelo>>4)))
    DataFlag|=0x02;
  if(timerA_value==dataSend)
  {
    totaltimestamp++;
    timerA_value=0;
    DataFlag|=0x10;
  }
}  


