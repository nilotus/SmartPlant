#ifndef __B_LUX_V20_H
#define __B_LUX_V20_H

/*--------------------------ͷ�ļ�����--------------------------------*/
#include "DataType.h"
//#include "stm8s.h"

/*-----------------------------�ṹ�嶨��---------------------------------*/

/*-----------------------------�궨��---------------------------------*/
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

//���Ŷ���
#define B_LUX_SCL0_O    GPIOD->DDR    |=  B_LUX_SCL0;  GPIOD->CR1 |= B_LUX_SCL0         //�������
#define B_LUX_SCL0_H    GPIOD->ODR    |=  B_LUX_SCL0
#define B_LUX_SCL0_L    GPIOD->ODR    &=  ~B_LUX_SCL0

#define B_LUX_SDA0_O    GPIOD->DDR    |=  B_LUX_SDA0;  GPIOD->CR1 |= B_LUX_SDA0     
#define B_LUX_SDA0_H    GPIOD->ODR    |=  B_LUX_SDA0
#define B_LUX_SDA0_L    GPIOD->ODR    &=  ~B_LUX_SDA0

#define B_LUX_SDA0_I    GPIOD->DDR    &=  ~B_LUX_SDA0;  GPIOD->CR1 &= ~B_LUX_SDA0       //��������
#define B_LUX_SDA0_DAT  ( (GPIOD->IDR>>3) & 0x01)

#define	B_LUX_SlaveAddress	  0x46                                          //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�

/*-----------------------------��������-------------------------------*/
vid B_LUX_delay_nms(uint16 k);
vid B_LUX_Init(vid);

vid  B_LUX_Single_Write(uint8 REG_Address);                    //����д������
uint8 B_LUX_Single_Read(uint8 REG_Address);                     //������ȡ�ڲ��Ĵ�������
vid  B_LUX_Multiple_read(vid);                               //�����Ķ�ȡ�ڲ��Ĵ�������
//------------------------------------
vid B_LUX_Delay5us(vid);
vid B_LUX_Delay5ms(vid);
vid B_LUX_Start(vid);                          //��ʼ�ź�
vid B_LUX_Stop(vid);                           //ֹͣ�ź�
vid B_LUX_SendACK(uint8 ack);                  //Ӧ��ACK
uint8  B_LUX_RecvACK(vid);                     //��ack
vid B_LUX_SendByte(uint8 dat);                 //IIC�����ֽ�д
uint8 B_LUX_RecvByte(vid);                     //IIC�����ֽڶ�

vid B_LUX_GetLux(uint32 *vLux);
#endif
