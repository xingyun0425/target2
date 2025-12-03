
#include "imu.h"
#include "os.h"

#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"

#include "MahonyAHRS.h"
#include "math.h"
#define MPU_TEMP_CONTROL
#define PI 3.14159265f

Attitude attitude;
float temp;
float gyro[3] = {0.0f, 0.0f, 0.0f};
float accel[3] = {0.0f, 0.0f, 0.0f};
float mag[3] = {0.0f, 0.0f, 0.0f};
float quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGTH];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGTH] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGTH];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGTH] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGTH];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGTH] = {0xA2,0xFF,0xFF,0xFF};
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

extern SPI_HandleTypeDef hspi1;

static osThreadId_t INS_task_local_handler;
float timing_time = 0.001f;   //loop run time , unit s.
// Acceleror meter filter
fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_filter_3[3] = {0.0f, 0.0f, 0.0f};
fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;


fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};   

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
#ifdef APPLY_BIAS
fp32 gyro_cali_offset[3];
#endif

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
#ifdef APPLY_BIAS
fp32 accel_cali_offset[3];
#endif

fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
#ifdef APPLY_BIAS
fp32 mag_cali_offset[3];
#endif



void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

#ifdef MPU_TEMP_CONTROL
  #include "pid.h"
  extern TIM_HandleTypeDef htim10;
  #define HEAT_HAL_TIM htim10
  #define HEAT_LL_TIM TIM10
  #define HEAT_CH TIM_CHANNEL_1
	#define TEMP_FACTOR 1
  #define TARGET_TEMP 47
	
  void mpu_temp_control(int16_t raw_temp) {

    static PID pid = {
      .kp = 6.0f,
      .ki = 0.2f,
      .kd = 0.4f,
      .max_out = 1400.0f,
      .integral_limit = 900.0f
    };

    static int32_t pulse = 0;

    /* apply Low-pass Filter for temperature sensor */
    static int16_t temp=0;
    #define TS 0.001f /* sample interval in ms */
    #define FC 100.0f /* cut-off frequency */
    /* filter update ratio in % */
    #define LPF_ALPHA ((int32_t) ((TS / (TS + 1 / (2 * PI * FC))) * 100.0f))
    temp = ((100-LPF_ALPHA) * temp + LPF_ALPHA * raw_temp) / 100;
    #undef TS
    #undef FC
    #undef LPF_ALPHA

    if (temp < 0.98 * TARGET_TEMP) {
      pulse = 60000;
    } else {
      pulse = PID_calc(&pid, temp, TARGET_TEMP);
      if (pulse < 0) pulse = 0;
    }

    /* not really switching, compiler will optimize this */
    switch (HEAT_CH) {
      case TIM_CHANNEL_1:
        HEAT_LL_TIM->CCR1 = pulse;
        break;
    }
  }

#endif

__weak void IMU_DataFrame_Handler(IMU const *imu) {
	
	float lastYaw = attitude.yaw;
	
	attitude.totalYaw = attitude.refYawCircle *360.0f +attitude.yaw;
  	attitude.yaw = (float)imu->attitude.x/PI*180;
	attitude.roll = (float)imu->attitude.y/PI*180;
	attitude.pitch = (float)imu->attitude.z/PI*180;
	
	if(attitude.yaw >90 && lastYaw<-90)
		attitude.refYawCircle--;
	if(attitude.yaw<-90&& lastYaw >90)
		attitude.refYawCircle++;
}
#define IMU_DATA_PENDING 0x0001
void IMUDataHandler(void *_) 
{
	IMU imu;
	//wait a time
	osDelay(INS_TASK_INIT_TIME);
	while(BMI088_init())
	{
			osDelay(100);
	}
	while(ist8310_init())
	{
			osDelay(100);
	}

	BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

	AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);

	//set spi frequency
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
			Error_Handler();
	}


	SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGTH);

	imu_start_dma_flag = 1;

	while (1)
	{
		//wait spi DMA tansmit done
		//等待SPI DMA传输
		osThreadFlagsWait(IMU_DATA_PENDING, osFlagsWaitAny, osWaitForever);


		if(gyro_update_flag & (1 << IMU_UPDATE_SHIFTS))
		{
			gyro_update_flag &= ~(1 << IMU_UPDATE_SHIFTS);
			BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
		}

		if(accel_update_flag & (1 << IMU_UPDATE_SHIFTS))
		{
			accel_update_flag &= ~(1 << IMU_UPDATE_SHIFTS);
			BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
		}

		if(accel_temp_update_flag & (1 << IMU_UPDATE_SHIFTS))
		{
			accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHIFTS);
			BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
			#ifdef MPU_TEMP_CONTROL
				mpu_temp_control((uint16_t)bmi088_real_data.temp);
			#endif
		}

		//加速度计低通滤波
		//accel low-pass filter
		accel_fliter_1[0] = accel_fliter_2[0];
		accel_fliter_2[0] = accel_filter_3[0];

		accel_filter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + accel[0] * fliter_num[2];

		accel_fliter_1[1] = accel_fliter_2[1];
		accel_fliter_2[1] = accel_filter_3[1];

		accel_filter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + accel[1] * fliter_num[2];

		accel_fliter_1[2] = accel_fliter_2[2];
		accel_fliter_2[2] = accel_filter_3[2];

		accel_filter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + accel[2] * fliter_num[2];
		AHRS_update(INS_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel, ist8310_real_data.mag);
		get_angle(INS_quat, &imu.attitude.x, &imu.attitude.y, &imu.attitude.z);
		
		IMU_DataFrame_Handler(&imu);
	}

}

void IMU_Init()
{
  /* start PWM if temperature control is enabled */
  #ifdef MPU_TEMP_CONTROL
    HAL_TIM_PWM_Start(&HEAT_HAL_TIM, HEAT_CH);
  #endif
	
	// Init data
	attitude.refYawCircle = 0;
	attitude.totalYaw = 0.0f;
	attitude.yaw = 0.0f;
	
  // Accelerate spi frequency
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	
  AHRS_init(quat, accel, mag);
	accel_fliter_1[0] = accel_fliter_2[0] = accel_filter_3[0] = accel[0];
  accel_fliter_1[1] = accel_fliter_2[1] = accel_filter_3[1] = accel[1];
  accel_fliter_1[2] = accel_fliter_2[2] = accel_filter_3[2] = accel[2];
    


	// imu_start_dma_flag = 1;
	#ifdef APPLY_BIAS
  /* try to restore gyro bias from flash */
  if (!restore_gyro_bias()) {
    /* perform a hasty gyro calibration (<0.7s) */
    calibrate_gyro(1);
  }
	#endif


  INS_task_local_handler = osThreadCreate("imu", IMUDataHandler, NULL, osPriorityAboveNormal, 128);
}
	




void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0, 0, 0);
}
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}


/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
void imu_cmd_spi_dma(void)
{

	//开启陀螺仪的DMA传输
	if( (gyro_update_flag & (1 << IMU_DR_SHIFTS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
	&& !(accel_update_flag & (1 << IMU_SPI_SHIFTS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHIFTS)))
	{
			gyro_update_flag &= ~(1 << IMU_DR_SHIFTS);
			gyro_update_flag |= (1 << IMU_SPI_SHIFTS);

			HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
			SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGTH);
			return;
	}
	//开启加速度计的DMA传输
	if((accel_update_flag & (1 << IMU_DR_SHIFTS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
	&& !(gyro_update_flag & (1 << IMU_SPI_SHIFTS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHIFTS)))
	{
			accel_update_flag &= ~(1 << IMU_DR_SHIFTS);
			accel_update_flag |= (1 << IMU_SPI_SHIFTS);

			HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
			SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGTH);
			return;
	}
	



	if((accel_temp_update_flag & (1 << IMU_DR_SHIFTS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
	&& !(gyro_update_flag & (1 << IMU_SPI_SHIFTS)) && !(accel_update_flag & (1 << IMU_SPI_SHIFTS)))
	{
			accel_temp_update_flag &= ~(1 << IMU_DR_SHIFTS);
			accel_temp_update_flag |= (1 << IMU_SPI_SHIFTS);

			HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
			SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGTH);
			return;
	}
}


void DMA2_Stream2_IRQHandler_IMU(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHIFTS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHIFTS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHIFTS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHIFTS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHIFTS);
            accel_update_flag |= (1 << IMU_UPDATE_SHIFTS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHIFTS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHIFTS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHIFTS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHIFTS))
        {
            //__HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
					osThreadFlagsSet(INS_task_local_handler, IMU_DATA_PENDING);
        }
    }
}

void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);

    __HAL_SPI_ENABLE(&hspi1);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    
    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_LISR_TCIF2);

    hdma_spi1_rx.Instance->PAR = (uint32_t) & (SPI1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_spi1_rx.Instance->M0AR = (uint32_t)(rx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, num);

    __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    
    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }


    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_LISR_TCIF3);

    hdma_spi1_tx.Instance->PAR = (uint32_t) & (SPI1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_spi1_tx.Instance->M0AR = (uint32_t)(tx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, num);


}

void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }
    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }
    //clear flag
    //清除标志位
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx));

    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmatx));
    //set memory address
    //设置数据地址
    hdma_spi1_rx.Instance->M0AR = rx_buf;
    hdma_spi1_tx.Instance->M0AR = tx_buf;
    //set data length
    //设置数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, ndtr);
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, ndtr);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_spi1_rx);
    __HAL_DMA_ENABLE(&hdma_spi1_tx);
}


