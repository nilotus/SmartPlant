#ifndef __B_LUX_V20_H
#define __B_LUX_V20_H

/*--------------------------头文件引用--------------------------------*/
#include "DataType.h"
//#include "stm8s.h"

/*-----------------------------结构体定义---------------------------------*/

/*-----------------------------宏定义---------------------------------*/
#define         B_LUX_SCL0      (0x01<<2)
#define         B_LUX_SDA0      (0x01<<3)

#define     __I     volatile const   /*!< defines 'read only' permissions     */
#define     __O     volatile         /*!< defines 'write only' permissions    */
#define     __IO    volatile         /*!< defines 'read / write' permissions  */

/*!< Signed integer types  */
typedef   signed char     int8_t;
typedef   signed short    int16_t;
typedef   signed long     int32_t;

/*!< Unsigned integer types  */
typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned long     uint32_t;


typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef struct GPIO_struct
{
  __IO uint8_t ODR; /*!< Output Data Register */
  __IO uint8_t IDR; /*!< Input Data Register */
  __IO uint8_t DDR; /*!< Data Direction Register */
  __IO uint8_t CR1; /*!< Configuration Register 1 */
  __IO uint8_t CR2; /*!< Configuration Register 2 */
}
GPIO_TypeDef;

#define GPIOD_BaseAddress       0x500F
#define GPIOD ((GPIO_TypeDef *) GPIOD_BaseAddress)

//引脚定义
#define B_LUX_SCL0_O    GPIOD->DDR    |=  B_LUX_SCL0;  GPIOD->CR1 |= B_LUX_SCL0         //推挽输出
#define B_LUX_SCL0_H    GPIOD->ODR    |=  B_LUX_SCL0
#define B_LUX_SCL0_L    GPIOD->ODR    &=  ~B_LUX_SCL0

#define B_LUX_SDA0_O    GPIOD->DDR    |=  B_LUX_SDA0;  GPIOD->CR1 |= B_LUX_SDA0     
#define B_LUX_SDA0_H    GPIOD->ODR    |=  B_LUX_SDA0
#define B_LUX_SDA0_L    GPIOD->ODR    &=  ~B_LUX_SDA0

#define B_LUX_SDA0_I    GPIOD->DDR    &=  ~B_LUX_SDA0;  GPIOD->CR1 &= ~B_LUX_SDA0       //悬浮输入
#define B_LUX_SDA0_DAT  ( (GPIOD->IDR>>3) & 0x01)

#define	B_LUX_SlaveAddress	  0x46                                          //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

/*-----------------------------函数声明-------------------------------*/
vid B_LUX_delay_nms(uint16 k);
vid B_LUX_Init(vid);

vid  B_LUX_Single_Write(uint8 REG_Address);                    //单个写入数据
uint8 B_LUX_Single_Read(uint8 REG_Address);                     //单个读取内部寄存器数据
vid  B_LUX_Multiple_read(vid);                               //连续的读取内部寄存器数据
//------------------------------------
vid B_LUX_Delay5us(vid);
vid B_LUX_Delay5ms(vid);
vid B_LUX_Start(vid);                          //起始信号
vid B_LUX_Stop(vid);                           //停止信号
vid B_LUX_SendACK(uint8 ack);                  //应答ACK
uint8  B_LUX_RecvACK(vid);                     //读ack
vid B_LUX_SendByte(uint8 dat);                 //IIC单个字节写
uint8 B_LUX_RecvByte(vid);                     //IIC单个字节读

vid B_LUX_GetLux(uint32 *vLux);
#endif
