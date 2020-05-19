#include "DATA.h"

uint8 Meeting_Flag=0;

void DATA_init()
{
    uart_init(uart0,57600);     //初始化串口(VCAN_PORT 是工程里配置为printf函数输出端口，故已经进行初始化)
   // uart_rx_irq_en (uart1);                                 //开串口接收中断

	
	uart_init(uart4,9600);     //初始化串口(VCAN_PORT 是工程里配置为printf函数输出端口，故已经进行初始化)
	uart_rx_irq_en (uart4);                                 //开串口接收中

}



void vcan_sendware(void *wareaddr, uint32_t waresize)
{
#define CMD_WARE     3
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

    uart_putbuff(uart0, cmdf, sizeof(cmdf));    //先发送前命令
    uart_putbuff(uart0, (uint8_t *)wareaddr, waresize);    //发送数据
    uart_putbuff(uart0, cmdr, sizeof(cmdr));    //发送后命令
}



/****接受数据*****/
//void Twocar_Train_RX()
//{
//  unsigned char data;
//  if(uart_query (uart1) != 0)   //接收数据寄存器满
//  {
//    //用户需要处理接收数据
//    uart_getchar (uart1, &data);                    //无限等待接受1个字节
//    if(data == 0xfd)//b车已到会车区
//    {
//	Meeting_Flag=1;
//    }
////        if(data == 0x01)//b车已到会车区
////    {
////         stop_Flag=0; 
////    }
//  }
//}
//
//
//
///****发送数据*****/
//
//void Twocar_Train_TX()
//{
//  //unsigne/d char ch,value;
//  
//  uart_putchar(uart1,0xfd);//中间会车我先到
//}




//-------------------------------------------------------------------------------------------------------------------
//@brief     双车通信
//@param      NULL
//@return     uint8
//@since      v1.0
//Sample usage:
//  @note       L_Find,R_Find
//    -------------------------------------------------------------------------------------------------------------
char A_Start_Ok,A_Reach_Break,A_Reach_End,A_Finish_End,B_Start_Ok,B_Reach_Break,B_Reach_End,B_Finish_End;
uint8 TX_BUFF,RX_BUFF;
//void Twocar_Train_TX()
//{
//  TX_BUFF[0] = 0x52;
//  TX_BUFF[1] = 0x0;
//  TX_BUFF[2] = 0x0;
//  TX_BUFF[3] = 0x0;
//  TX_BUFF[4] = 0x0;
//  TX_BUFF[5] = 0x52;
//  
//  if(A_Start_Ok == 1)
//    TX_BUFF[1] = 0x19;             //A表示我的车，B表示另外一辆车
//  if(A_Start_Ok == 0)
//    TX_BUFF[1] = 0x0;
//  
//  if(A_Reach_Break==1)
//    TX_BUFF[2] = 0x29;
//  if(A_Reach_Break==0)
//    TX_BUFF[2] = 0x0;
//  
//  if(A_Finish_End==1)
//    TX_BUFF[4] = 0x49;
//  if(A_Finish_End==1)
//    TX_BUFF[4] = 0x0;
//  
//  uart_putbuff(uart1,TX_BUFF,sizeof(TX_BUFF));
//}
//
//void Twocar_Train_RX()
//{ 
//  uint8 RX;
//  static int Record=0;
//  
//  if(uart_query(uart1))   //接收数据寄存器满
//  {
//    uart_getchar (uart1, &RX);                    //无限等待接受1个字节
//    RX_BUFF[Record] = RX;
//    if(RX_BUFF[0] == 0x52)
//      Record++;
//    else
//      Record = 0;
//    if(Record == 6)
//    {
//      Record = 0;
//      
//      if(RX_BUFF[0] == 0x52 && RX_BUFF[5] == 0x52)
//      {
//        if(RX_BUFF[1] == 0x19)
//          B_Start_Ok = 1;
//        if(RX_BUFF[1] == 0x0)
//          B_Start_Ok = 0;
//        
//        if(RX_BUFF[2] == 0x29)
//          B_Reach_Break = 1;
//        if(RX_BUFF[2] == 0x0)
//          B_Reach_Break = 0;
//        
//        if(RX_BUFF[3] == 0x39)
//          B_Reach_End = 1;
//        if(RX_BUFF[3] == 0x0)
//          B_Reach_End = 0;
//        
//        if(RX_BUFF[4] == 0x49)
//          B_Finish_End = 1;
//        if(RX_BUFF[4] == 0x0)
//          B_Finish_End = 0;
//      }
//    }
//    
//  }
//}


void Twocar_Train_TX()
{
  TX_BUFF = 0;
  
  if(A_Start_Ok == 1)
    TX_BUFF = 0x19;             //A表示我的车，B表示另外一辆车

  if(A_Reach_Break==1)
    TX_BUFF = 0x29;
  
  if(A_Finish_End==1)
    TX_BUFF = 0x39;

  uart_putchar(uart0,TX_BUFF);
}

void Twocar_Train_RX()
{ 
  if(uart_query(uart0) != 0)   //接收数据寄存器满
  {
    uart_getchar (uart0, &RX_BUFF);                    //无限等待接受1个字节
    
    if(RX_BUFF == 0x19)
      B_Start_Ok = 1;
    
    if(RX_BUFF == 0x29)
      B_Reach_Break = 1;
    
    if(RX_BUFF == 0x39)
      B_Reach_End = 1;
    
    if(RX_BUFF == 0x49)
      B_Finish_End = 1;

  }
}


