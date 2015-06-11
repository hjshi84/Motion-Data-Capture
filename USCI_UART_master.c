
#include "msp430x22x4.h"
#include "USCI_UART_master.h"



void USCI_UART_init() 
{
  P3SEL |= 0x30;                             // P3.4,5 = USCI_A0 TXD/RXD
  UCA0CTL1|=UCSWRST;
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 0x22;//0xA0;//0x04; //0X08;//                       // 8MHz
  UCA0BR1 =  0x00;  //0x01;//0x00;//0x00;//                           // 1MHz 
  UCA0MCTL = 0x06;//0x0C;//0x3B;//0xB1;////UCBRS2 + UCBRS0;               // Modulation UCBRSx = 5
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
  // IE2 &= ~UCA0TXIE;
}

short UART_send(char value)
{ // sends a char through UART
  
  UCA0TXBUF = value;
  while (!(IFG2&UCA0TXIFG));
  IFG2&=~UCA0TXIFG;
  return 0;
}

short UART_STR_send(unsigned char *strin)
{ 
  // sends a string through UART
  unsigned char i=0;
  UART_send(0x41); // start of string
  while(i<15)
  { 
    UART_send(*strin);
    *strin++;
    i++;
  }
  UART_send(0x5A); // end of string
  return 0;
}



