#include "DATA.h"

uint8 Meeting_Flag=0;

void DATA_init()
{
    uart_init(uart0,57600);     //��ʼ������(VCAN_PORT �ǹ���������Ϊprintf��������˿ڣ����Ѿ����г�ʼ��)
   // uart_rx_irq_en (uart1);                                 //�����ڽ����ж�

	
	uart_init(uart4,9600);     //��ʼ������(VCAN_PORT �ǹ���������Ϊprintf��������˿ڣ����Ѿ����г�ʼ��)
	uart_rx_irq_en (uart4);                                 //�����ڽ�����

}



void vcan_sendware(void *wareaddr, uint32_t waresize)
{
#define CMD_WARE     3
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //���ڵ��� ʹ�õ�ǰ����
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //���ڵ��� ʹ�õĺ�����

    uart_putbuff(uart0, cmdf, sizeof(cmdf));    //�ȷ���ǰ����
    uart_putbuff(uart0, (uint8_t *)wareaddr, waresize);    //��������
    uart_putbuff(uart0, cmdr, sizeof(cmdr));    //���ͺ�����
}



/****��������*****/
//void Twocar_Train_RX()
//{
//  unsigned char data;
//  if(uart_query (uart1) != 0)   //�������ݼĴ�����
//  {
//    //�û���Ҫ�����������
//    uart_getchar (uart1, &data);                    //���޵ȴ�����1���ֽ�
//    if(data == 0xfd)//b���ѵ��ᳵ��
//    {
//	Meeting_Flag=1;
//    }
////        if(data == 0x01)//b���ѵ��ᳵ��
////    {
////         stop_Flag=0; 
////    }
//  }
//}
//
//
//
///****��������*****/
//
//void Twocar_Train_TX()
//{
//  //unsigne/d char ch,value;
//  
//  uart_putchar(uart1,0xfd);//�м�ᳵ���ȵ�
//}




//-------------------------------------------------------------------------------------------------------------------
//@brief     ˫��ͨ��
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
//    TX_BUFF[1] = 0x19;             //A��ʾ�ҵĳ���B��ʾ����һ����
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
//  if(uart_query(uart1))   //�������ݼĴ�����
//  {
//    uart_getchar (uart1, &RX);                    //���޵ȴ�����1���ֽ�
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
    TX_BUFF = 0x19;             //A��ʾ�ҵĳ���B��ʾ����һ����

  if(A_Reach_Break==1)
    TX_BUFF = 0x29;
  
  if(A_Finish_End==1)
    TX_BUFF = 0x39;

  uart_putchar(uart0,TX_BUFF);
}

void Twocar_Train_RX()
{ 
  if(uart_query(uart0) != 0)   //�������ݼĴ�����
  {
    uart_getchar (uart0, &RX_BUFF);                    //���޵ȴ�����1���ֽ�
    
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


