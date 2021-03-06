#include <stdio.h>
#include "stm32f10x.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "usart.h"
#include "stm32_iic.h"
#include "eeprom.h"
#include "adc.h"
#include "led.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"

#define  Pitch_error  1.0
#define  Roll_error   -2.0
#define  Yaw_error    0.0
#define DEFAULT_MPU_HZ  (100)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define PAGE_ADDR (0x08000000 + 63 * 1024)
#define uint8 unsigned char
#define uint16 unsigned int
#define uint32 unsigned long
#define q30  1073741824.0f

//相关变量
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];

// ADC
__IO u16 ADC_ConvertedValue;
float ADC_ConvertedValueLocal;

// 设定出厂偏差值
float rx = 0;
float ry = 0;

// 姿态角数据保存值
// r[0/2]表示姿态角正负
// r[1/3]表示姿态角*100（保存小数点后两位）
u16 r[4]={0};

// 姿态角
float Pitch,Roll,Yaw;

// 四元数
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// 方位矩阵
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/* 
 * 将方位矩阵转化为标量表示，DMP中会用到
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

struct rx_s 
{
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s 
{
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};

static struct hal_s hal = {0};

static void run_self_test()
{
    int result;

    long gyro[3], accel[3];

		// 自检
    result = mpu_run_self_test(gyro, accel);
	
    if (result == 0x7) 
    {
				// 自检成功后将数据送入DMP处理
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
				printf("setting bias succesfully ......\n");
    }
		else
		{
				printf("bias has not been modified ......\n");
		}
}

int main(void)
{  
    u16 count=0;  
	
	  // 串口初始化
    USART_Config(); 
	
		// LED初始化
    LED_Config();
	
    // IIC总线初始化
    I2CInit();

    // ADC初始化
		ADC1_Init();

    // 系统延时
    delay_ms(10);
	
		if(!mpu_init())   //返回0代表初始化成功
    {   
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        {
            printf("mpu_set_sensor complete ......\n");
        }
        else
        {
            printf("mpu_set_sensor come across error ......\n");
        }
        
        // MPU配置FIFO
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        {
            printf("mpu_configure_fifo complete ......\n");
        }
        else
        {
            printf("mpu_configure_fifo come across error ......\n");
        }
        
        // MPU设置采样率
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
        {
            printf("mpu_set_sample_rate complete ......\n");
        }
        else
        {
            printf("mpu_set_sample_rate error ......\n");
        }
        
        // 加载DMP驱动固件
        if(!dmp_load_motion_driver_firmware())
        {
            printf("dmp_load_motion_driver_firmware complete ......\n");
        }
        else
        {
            printf("dmp_load_motion_driver_firmware come across error ......\n");
        }
        
        // DMP设置姿态矩阵
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
        {
            printf("dmp_set_orientation complete ......\n");
        }
        else
        {
            printf("dmp_set_orientation come across error ......\n");
        }
        
        // DMP启动
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
        {
            printf("dmp_enable_feature complete ......\n");
        }
        else
        {
            printf("dmp_enable_feature come across error ......\n");
        }
        
        // DMP设置FIFO速率
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
        {
           printf("dmp_set_fifo_rate complete ......\n");
        }
        else
        {
            printf("dmp_set_fifo_rate come across error ......\n");
        }
				
				// 不开自检，以水平作为零度
        // 开启自检以当前位置作为零度
        //run_self_test();
				
				printf("Adjusting,please wait...");
				
				// 读取Flash中保存的配置
				STMFLASH_Read(0, (u16*)r, sizeof(r));
				if (r[0] == 0)
						rx = -(float)r[1] / 100;
				else
						rx = (float)r[1] / 100;
				if (r[2] == 0)
						ry = -(float)r[3] / 100;
				else 
						ry = (float)r[3] / 100;
				
				printf("rx = %.2f, ry = %.2f",rx, ry);
				
				printf("Adjustment complete!");
				
				if(!mpu_set_dmp_state(1))
        {
            printf("mpu_set_dmp_state complete ......\n");
        }
        else
        {
            printf("mpu_set_dmp_state come across error ......\n");
        }
		}
		
		// LED闪烁示意开启
		LED_Flash();
		
		while(1)
    {
				// 灯亮灭代表正在运行状态
        count ++;          
        if(count < 100)
        {
            led_on;  
        }
        else if(count < 200)
        {
            led_off;  
        }
        else if(count == 200)
        {
            count = 0; 
        }
				
        // 读取DMP
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
				
				// 四元数解姿态
        if (sensors & INV_WXYZ_QUAT )
        {
						float temp;
            q0 = quat[0] / q30;
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;
            
            Pitch  = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3 + Pitch_error; // pitch
            Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3 + Roll_error; // roll
            Yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3 + Yaw_error;
					
					 // 输出adc得到的电压，假设电源电压稳定3v3
						ADC_ConvertedValueLocal=(float) ADC_ConvertedValue/4096*3.3;
						printf("%.2f\n",ADC_ConvertedValueLocal);
					
            printf("Roll:");
            temp = (Roll);
            printf("%.2f ",temp - rx);
            printf("Pitch:");
            temp = (Pitch);
            printf("%.2f ",temp - ry);
            printf("Yaw:");
            temp = (Yaw);
            printf("%.2f",temp);
				}
				
				// 置零键被按下
				if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5))
				{
						delay_ms(500);
						if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5))
						{
								printf("Start adjusting, Please wait...\n");
								
								// 保存Flash配置
								if (Roll < 0)
										r[0] = 0;
								else
										r[0] = 1;
								r[1]=Roll*100;
								
								if (Pitch < 0)
										r[2] = 0;
								else
										r[2] = 1;
								r[3]=Pitch*100;

								STMFLASH_Write(0, (u16*)r, sizeof(r));
								STMFLASH_Read(0, (u16*)r, sizeof(r));
								
								if (r[0] == 0)
										rx = -(float)r[1] / 100;
								else
										rx = (float)r[1] / 100;
								if (r[2] == 0)
										ry = -(float)r[3] / 100;
								else 
										ry = (float)r[3] / 100;
								
								//run_self_test();
								LED_Flash();
								printf("Succeed!\n");
								printf("rx = %.2f, ry = %.2f",rx, ry);
						}
				}
				
        /*if(sensors & INV_XYZ_GYRO)
        {}
        if(sensors & INV_XYZ_ACCEL)
        {}*/
    }
}

void USART1_IRQHandler()
{
		uint8_t ch;

		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		{     
				ch = USART_ReceiveData(USART1);
				printf( "%c", ch ); 
		} 
}

void DMA1_Channel1_IRQHandler()
{
		if(DMA_GetITStatus(DMA1_IT_TC1))
		{
				TIM_Cmd(TIM3,DISABLE);  
				DMA_ClearITPendingBit(DMA1_IT_GL1);
		}
}