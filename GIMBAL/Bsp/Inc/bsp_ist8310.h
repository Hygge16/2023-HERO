//
// Created by Yuanbin on 22-10-3.
//

#ifndef BSP_IST8310_H
#define BSP_IST8310_H

#include "stdint.h"


extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_RST_H(void);
extern void ist8310_RST_L(void);

#endif //BSP_IST8310_H
