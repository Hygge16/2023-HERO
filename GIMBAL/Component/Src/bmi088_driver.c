//
// Created by Yuanbin on 23-3-20.
//

#include "bmi088_driver.h"
#include "bsp_bmi088.h"
#include "BMI088_reg.h"
#include "haltick.h"
#include "stdbool.h"
#include "pid.h"

static float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;  //3倍采样  ACCEL
static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;  //2000字节长度  GYRO
mpu_data_t mpu_data;

static void BMI088_Offset_Cali(void);

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }

#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_Read_Write_Byte((reg) | 0x80);   \
        BMI088_Read_Write_Byte(0x55);           \
        (data) = BMI088_Read_Write_Byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }

#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_Read_Write_Byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR}, //打开加速计以及状态机错误位
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},  //暂停模式以及对应的状态机
    {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},  //加速寄存器以及对应的状态机
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},  //加速计设定范围以及对应状态机
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR}, //配置输入、输出引脚INT1以及对应状态机
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}  //中断映射引脚以及对应状态机
};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, //角速率以及分辨率，对应状态机
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR}, //数据传输速率与带宽设置，对应状态机
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},   //电源模式选择寄存器，对应状态机
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},   //数据中断触发寄存器，对应状态机
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},  //中断引脚触发寄存器，对应状态机
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}   //中断映射寄存器，对应状态机
};
bool bmi088_accel_self_test(void)
{

    int16_t self_test_accel[2][3];

    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    volatile uint8_t res = 0;

    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
        {
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR}

        };

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset  bmi088 accel sensor and wait for > 50ms
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        delay_ms(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        delay_ms(BMI088_LONG_DELAY_TIME);

        // read response accel
        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
    }

    //set self test off
    BMI088_accel_write_single_reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_SELF_TEST, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF))
    {
        return BMI088_ACC_SELF_TEST_ERROR;
    }

    //reset the accel sensor
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680))
    {
        return BMI088_SELF_TEST_ACCEL_ERROR;
    }

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_ERROR;
}
bool bmi088_gyro_self_test(void)
{
    uint8_t res = 0;
    uint8_t retry = 0;
    //check commiunication is normal
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    delay_ms(BMI088_LONG_DELAY_TIME);

    do
    {

        BMI088_gyro_read_single_reg(BMI088_GYRO_SELF_TEST, res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    if (res & BMI088_GYRO_BIST_FAIL)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    return BMI088_NO_ERROR;
}

bool bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //检查通讯
		BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);  //读取加速计ID地址
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);         //等待150us，加速计配置成功
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);   //再来一遍，检查是否正常
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //加速计软件复位
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);  //进行软件复位，BMI088传感器进行复位，对寄存器ACC_SOFTRESET,地址为0x7E写入0xB6，，进行软件复位
    delay_ms(BMI088_LONG_DELAY_TIME);//软件复位等待时间手册里说明是1ms，这里给出80，如果对于系统比较复杂的，建议10ms或者20ms就行。

    // 同样的，软件复位以后，在检查一遍通信是否正常
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 检测 "who am I"这个寄存器是否正确
    if (res != BMI088_ACC_CHIP_ID_VALUE)  //如果读0x00寄存器的值，返回的值不是0x1E，则没有复位成功
    {
        return BMI088_NO_SENSOR;
    }

    //接下来就是配置传感器的基本信息
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
			/*依次像传感器的内部写入配置寄存器值，依次是：
			  [0][0]  BMI088_ACC_PWR_CTRL 0x7D  //开启或者关闭加速计，底下为其应该设定的值
			  [0][1]  BMI088_ACC_ENABLE_ACC_ON 0x04    //打开加速计
			  [1][0]  BMI088_ACC_PWR_CONF 0x7C  //将加速计切换到暂停模式，这种模式下，数据采集停止
			  [1][1]  BMI088_ACC_PWR_ACTIVE_MODE 0x00   //开始
			  [2][0]  BMI088_ACC_CONF 0x40    //加速计配置寄存器
			  [2][1]  BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)  //正常采样 | BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)   //800hz |  BMI088_ACC_CONF_MUST_Set 0x80  最终值0xBA  10111010
			  [3][0]  BMI088_ACC_RANGE 0x41   //加速计范围设定寄存器
			  [3][1]  BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)   //+-3g
			  [4][0]  BMI088_INT1_IO_CTRL 0x53   //配置输入、输出引脚INT1
			  [4][1]  BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)  //INT配置为输出引脚 | BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)  //推挽输出 | BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)  //下拉
			  [5][0]  BMI088_INT_MAP_DATA 0x58  //中断映射到INT1,INT2上，之后可以通过外部中断进行触发
			  [5][1]  BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)  //映射到INT1，之后可有外部中断触发INT1进行工作
				*/
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]); 
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);  //等待150ms

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res); //读取所有配置寄存器的ID号是否正确
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])  //将读取到配置寄存器的值与我们设定的值比较
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];  //不想等则对应反馈相应的错误
        }
    }
    return BMI088_NO_ERROR;  //无，则没有错误，标志着初始化成功
}

bool bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);  // 读取陀螺仪的ID地址
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);  //延时150ms
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //设置陀螺传感器
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);  //将传感器重置
    delay_ms(BMI088_LONG_DELAY_TIME);
    //复位后检查通信是否正常
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);  //重置以后再次读取ID号
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_GYRO_CHIP_ID_VALUE)  //判断ID号是否正确
    {
        return BMI088_NO_SENSOR;   //不正确反馈重置错误
    }

    //设置陀螺传感器配置并检查
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        /*依次像传感器的内部写入配置寄存器值，依次是：
				[0][0]  BMI088_GYRO_RANGE 0x0F  //角速率范围以及分辨率
			  [0][1]  BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)  //+-2000°/s
			  [1][0]  BMI088_GYRO_BANDWIDTH 0x10   //设置速率数据滤波器带宽和输出速率
			  [1][1]  BMI088_GYRO_2000_532_HZ ，数据传输速率为2kHZ,带宽为532hz
			  [2][0]  BMI088_GYRO_LPM1 0x11  //电源模式的选择寄存器
			  [2][1]  BMI088_GYRO_NORMAL_MODE 0x00  //正常模式
			  [3][0]  BMI088_GYRO_CTRL 0x15   //数据中断触发寄存器
			  [3][1]  BMI088_DRDY_ON 0x80     //允许新的数据触发新的数据中断
			  [4][0]  BMI088_GYRO_INT3_INT4_IO_CONF 0x16  //中断引脚配置寄存器
			  [4][1]  BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)  //INT3推挽输出 | BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)  //INT3下拉 | 
			  [5][0]  BMI088_GYRO_INT3_INT4_IO_MAP 0x18  //中断映射寄存器
			  [5][1]  #define BMI088_GYRO_DRDY_IO_INT4 0x80  //映射到INT4
				*/
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);   //读取所有配置寄存器的ID号
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])  //判断ID号是否正确
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];  //错误就对应返回错误
        }
    }

    return BMI088_NO_ERROR;   //无错误，初始化正确
}


uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;

    // self test pass and init
    if (bmi088_accel_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_ACCEL_ERROR;
    }
    else
    {
        error |= bmi088_accel_init();
    }

    if (bmi088_gyro_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_GYRO_ERROR;
    }
    else
    {
        error |= bmi088_gyro_init();
    }

    delay_ms(5);
		if(!error)BMI088_Offset_Cali();
    return error;   //返回对应错误
}

void BMI088_read(float gyro[3], float accel[3], float *temperate)   //加速计与陀螺仪是三轴，所有用了数组，动量为3
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0]; //解算公式对照使用手册，这里将x轴的高八位左移8位，就相当于乘以256，要是右移，就是除以256，这里注意一定要强制改为int类型数据，因为最终结果较大，超过char的数据范围，因此需要转换
    mpu_data.ax = bmi088_raw_temp ;
	//	accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;  //这里的系数，根据0x41加速计范围设定寄存器地址的寄存器来进行计算得出
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    mpu_data.ay = bmi088_raw_temp ; 
	//	accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
 
		bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    mpu_data.az = bmi088_raw_temp ;
	//	accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);  //这一个函数，主要是将之前加速计模式下，拉回陀螺仪模式，并预留出数据缓冲区
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)   //如果保证是陀螺仪模式了，才开解算
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2] ;  //解算方法参考手册，进行移位计算
      mpu_data.gx = bmi088_raw_temp - mpu_data.gx_offset;
		//	gyro[0] = (bmi088_raw_temp - bmi088_real_data.gyro_offest[0]) * BMI088_GYRO_SEN;     //同样的，这里的系数根据角速率范围以及分辨率来进行设置
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
       mpu_data.gy = bmi088_raw_temp - mpu_data.gy_offset;
		  //	gyro[1] = (bmi088_raw_temp - bmi088_real_data.gyro_offest[1]) * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
       mpu_data.gz = bmi088_raw_temp - mpu_data.gz_offset;
			//gyro[2] = (bmi088_raw_temp - bmi088_real_data.gyro_offest[2]) * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);   //写入温度，读取温度

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));   //温度计算方法与上述一致

    if (bmi088_raw_temp > 1023)   //手册里给出，当采集回的温度值大于1023，则将其进行如下处理
    {
        bmi088_raw_temp -= 2048;
    }

	  if(ABS(mpu_data.gz)< 20)
	  {
		    mpu_data.gz = 0;
	  }
//		memcpy(&accel[0],&mpu_data.ax,6*sizeof(int16_t));
//		memcpy(&accel[1],&mpu_data.ay,6*sizeof(int16_t));
//		memcpy(&accel[2],&mpu_data.az,6*sizeof(int16_t));
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;   //计算最终温度值
		accel[0] = mpu_data.ax * BMI088_ACCEL_SEN;
		accel[1] = mpu_data.ay * BMI088_ACCEL_SEN;
		accel[2] = mpu_data.az * BMI088_ACCEL_SEN;
		gyro[0] = mpu_data.gx * BMI088_GYRO_SEN;
	  gyro[1] = mpu_data.gy * BMI088_GYRO_SEN;
	  gyro[2] = mpu_data.gz * BMI088_GYRO_SEN;
}

static void BMI088_Offset_Cali(void)
{
	
//	uint8_t buf[8] = {0, 0, 0, 0, 0, 0};

//	for(int i=0;i<1000;i++)
//	{
////		BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
//		BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
//		
////		mpu_data.ax_offset += (int16_t)((buf[1]) << 8) | buf[0];
////		mpu_data.ay_offset += (int16_t)((buf[3]) << 8) | buf[2];
////		mpu_data.az_offset += (int16_t)((buf[5]) << 8) | buf[4];
//	
//		mpu_data.gx_offset += (int16_t)((buf[3]) << 8) | buf[2];
//		mpu_data.gy_offset += (int16_t)((buf[5]) << 8) | buf[4];
//		mpu_data.gz_offset += (int16_t)((buf[7]) << 8) | buf[6];
//	
//		delay_ms(1);
//	}
//	
//	mpu_data.gx_offset=mpu_data.gx_offset / 1000;
//	mpu_data.gy_offset=mpu_data.gy_offset / 1000;
//	mpu_data.gz_offset=mpu_data.gz_offset / 1000;
	
	mpu_data.gx_offset=2;
	mpu_data.gy_offset=1;
	mpu_data.gz_offset=1;
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_Read_Write_Byte(reg);//先将地址写入传感器
    BMI088_Read_Write_Byte(data);//在将寄存器值写入传感器
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_Read_Write_Byte(reg | 0x80);  //这里为什么要单独与上0x80呢，因为手册里面提到，载陀螺仪模式下，要触发新的数据中断，通过将0x80写入寄存器去触发，其余模式也是一样的
    *return_data = BMI088_Read_Write_Byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_Read_Write_Byte( reg );
//    while( len != 0 )
//    {

//        BMI088_Read_Write_Byte( *buf );
//        buf ++;
//        len --;
//    }

//}

// 此函数需要读取的寄存器值，从长度上来判断，依次将其放入到指定的数据缓冲区中
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_Read_Write_Byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_Read_Write_Byte(0x55); 
        buf++;
        len--;
    }
}
#endif
