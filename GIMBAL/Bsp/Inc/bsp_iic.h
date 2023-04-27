//
// Created by Yuanbin on 22-10-2.
//

#ifndef BSP_IIC_H
#define BSP_IIC_H

#include "bsp_gpio.h"

/*
 * PB9输入模式
 * */
#define SDA_IN()    do { \
                        GPIOC->MODER&=~(3<<(9*2)); \
                        GPIOC->MODER|=0<<9*2;      \
                      }while(0U);
/*
 * PB9输出模式
 * */
#define SDA_OUT()   do { \
                        GPIOC->MODER&=~(3<<(9*2)); \
                        GPIOC->MODER|=1<<9*2;      \
                      }while(0U) //
// SCL
#define IIC_SCL    PAout(8)
// SDA
#define IIC_SDA    PCout(9)
// READSDA
#define READ_SDA   PCin(9)

extern void IIC_Init(void);
extern void IIC_Start(void);
extern void IIC_Stop(void);
extern uint8_t IIC_Wait_Ack(void);
extern void IIC_Ack(void);
extern void IIC_NAck(void);
extern void IIC_Send_Byte(uint8_t txd);
extern uint8_t IIC_Read_Byte(unsigned char ack);

#endif //BSP_IIC_H
