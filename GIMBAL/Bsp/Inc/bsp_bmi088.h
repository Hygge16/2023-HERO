//
// Created by Yuanbin on 22-10-3.
//

#ifndef BSP_BMI088_H
#define BSP_BMI088_H

#include "stdint.h"

#define BMI088_USE_SPI 


#if defined(BMI088_USE_SPI)

extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_Read_Write_Byte(uint8_t Txdata);

#endif

#endif //BSP_BMI088_H



