#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "BMI088reg.h"

#define BMI088_ACCEL_SEN BMI088_ACCEL_3G_SEN
#define BMI088_GYRO_SEN BMI088_GYRO_2000_SEN

#ifdef BMI088_USE_SPI

	#define BMI088_accel_write_single_reg(reg, data) \
			{                                            \
					BMI088_ACCEL_NS_L();                     \
					BMI088_write_single_reg((reg), (data));  \
					BMI088_ACCEL_NS_H();                     \
			}

	/**
		* @brief 对加速度计的单个寄存器进行读取
		*		
		* @note 1.NS口拉低，片选信号选中加速度计
		* @note 2.向加速度计写入data的低8位（寄存器地址及读写模式，|80给第0位赋1将其设置为读取模式）
		* @note 3.向加速度计写入data的高8位
		* @note 4.接收返回数据
		* @note 5.NS口拉高，取消片选，通信结束 		
		*/	
			
	#define BMI088_accel_read_single_reg(reg, data) \
			{                                           \
					BMI088_ACCEL_NS_L();                    \
					BMI088_read_write_byte((reg) | 0x80);   \
					BMI088_read_write_byte(0x55);           \
					(data) = BMI088_read_write_byte(0x55);  \
					BMI088_ACCEL_NS_H();                    \
			}
	//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }

	#define BMI088_accel_read_muli_reg(reg, data, len) \
			{                                              \
					BMI088_ACCEL_NS_L();                       \
					BMI088_read_write_byte((reg) | 0x80);      \
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
	//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
	#define BMI088_gyro_read_muli_reg(reg, data, len)   \
			{                                               \
					BMI088_GYRO_NS_L();                         \
					BMI088_read_muli_reg((reg), (data), (len)); \
					BMI088_GYRO_NS_H();                         \
			}

	static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
	static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
	//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
	static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
		
#endif

#ifndef BMI088_USE_SPI

#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
		
/**
	* @brief BMI088初始化
	* @param None
	* @retval None
	*/
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    //BMI088_GPIO_init(); 
    //BMI088_com_init();
    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();
    return error;
}

/**
	* @brief BMI088加速度计初始化
	* @param None
	* @retval None
	*/
unsigned char bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

/**
	* @brief BMI088陀螺仪初始化
	* @param None
	* @retval None
	*/
unsigned char bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

/**
	* @brief BMI088数据读取
	* @param None
	* @retval None
	*/
void BMI088_read(float gyro[3], float accel[3], float *temp)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    //加速度
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    //陀螺仪
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temp = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#ifdef BMI088_USE_SPI

	static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
	{
			BMI088_read_write_byte(reg);
			BMI088_read_write_byte(data);
	}

	static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
	{
			BMI088_read_write_byte(reg | 0x80); //需要在低八位或上0x80(10000000)使最低位为1以设置为读取模式
			*return_data = BMI088_read_write_byte(0x55);    //写入高八位（一个非地址的字节）的同时接收回传数据
	}

//	static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//	{
//	    BMI088_read_write_byte( reg );
//	    while( len != 0 )
//	    {
//	        BMI088_read_write_byte( *buf );
//	       buf ++;
//	        len --;
//	    }
//	}
	
	/**
		* @brief 按顺序读取多个寄存器的数据
		* 
		* @param reg 首个寄存器的地址
		* @param buf 接收读取数据缓冲区数组（首）地址
		* @param len 读取寄存器的个数
		*/
	static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
	{
			BMI088_read_write_byte(reg | 0x80); //选中寄存器 （burst模式？
			while (len != 0)
			{
					*buf = BMI088_read_write_byte(0x55);    //写入一个无关数据来提供时钟信号？，然后接收芯片回传的数据
					buf++;  
					len--;
			}
	}
#endif
