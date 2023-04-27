//
// Created by Yuanbin on 22-10-3.
//

#include "bsp_ist8310.h"
#include "bsp_iic.h"

/*IST8310的I2C地址,这里需要注意的是，在硬件IIC的协议下，只接收7位或者10位的地址，因此这里地址写入IIC时，需要左移一位或者右移两位，这里我使用的是左移一位，计算量可以稍微小一些
在IIC初始化时，结合原理图sda和scl都有一个上拉电阻4.7k的，因此可以直接使用硬件IIC配到400K，按道理4.7K的上拉电阻应该可以配1M，你们可以尝试一下  
初始化时需要将其左移好的数据直接写入进去，也可以写10位地址，但是不建议修改，因为一旦修改，在ist8310的写和读寄存器中就得修改
如果在调试过程中卡死，云台任务不工作，可能就是ist8310的iic有问题，查找对应的问题就可以了，常见就是死在iic检查的地方*/
#define IST8310_IIC_ADDRESS 0x0E  

//读ist8310单个字节
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
	uint8_t res;
    IIC_Start(); 
	IIC_Send_Byte(0x1C);
	IIC_Wait_Ack();		
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
    IIC_Start();
	IIC_Send_Byte(0x1D);
    IIC_Wait_Ack();		
	res=IIC_Read_Byte(0);
    IIC_Stop();
	return res;	
}

//读ist8310多个字节
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
   	IIC_Start(); 
	IIC_Send_Byte(0x1C);
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();		 	
	}
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
    IIC_Start();
	IIC_Send_Byte(0x1D);
    IIC_Wait_Ack();	
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);
		else *buf=IIC_Read_Byte(1);	
		len--;
		buf++; 
	}    
    IIC_Stop();	
}

//写ist8310单个字节
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
  IIC_Start(); 
	IIC_Send_Byte(0x1C);
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();		 	
	}
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
	IIC_Send_Byte(data);
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();	 	 
	}		 
    IIC_Stop();	 

}

//写ist8310多个字节
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    //ist8310不用写多个字节，暂时没有编写使用
}

void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
}

void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
}
