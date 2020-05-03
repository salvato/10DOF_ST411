#include "main.h"
#include "string.h" // for memset()


#include "stm32f4xx_hal.h"


#include <ADXL345.h>
#include <ITG3200.h>
#include <HMC5883L.h>
#include "MadgwickAHRS.h"


#define SAMPLING_FREQ  3200 // Hz
#define I2C_SPEEDCLOCK 400000 // Hz
#define I2C_DUTYCYCLE  I2C_DUTYCYCLE_2


static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void initSensors();
static void initTIM2(void);


ADXL345  Acc;      // Maximum Output Data Rate when using 400kHz I2C is 800 Hz
ITG3200  Gyro;     // 400KHz I2C capable
HMC5883L Magn;     // 400KHz I2C capable, left at the default 15Hz data Rate
Madgwick Madgwick;


// Each used module must be enabled in stm32f4xx_hal_conf.h
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef  Tim2Handle; // The Time Base

static float values[9];


int
main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    initSensors();

    // Wait for USER Button press before starting the Communication
    while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 1) ;
    // Wait for USER Button release before starting the Communication
    while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0) ;

    Madgwick.begin(float(SAMPLING_FREQ));

    while(!Acc.getInterruptSource(7)) {}
    Acc.get_Gxyz(&values[0]);
    while(!Gyro.isRawDataReadyOn()) {}
    Gyro.readGyro(&values[3]);
    while(!Magn.isDataReady()) {}
    Magn.ReadScaledAxis(&values[6]);

    // Let's start with a first estimate of the position (assumed the sensor static!)
    for(int i=0; i<10000; i++) { // ~13us per Madgwick.update()
        Madgwick.update(values[3], values[4], values[5],
                        values[0], values[1], values[2],
                        values[6], values[7], values[8]);
    }

    // Start periodic update interrupt !
    initTIM2();
    HAL_TIM_Base_Start_IT(&Tim2Handle);

    while(1) {
    }
}


void
initSensors() {
    bool bResult;
    bResult = Acc.init(ADXL345_ADDR_ALT_LOW, &hi2c1);
    if(!bResult) {
        while(1) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(50);
        }
    }
    Acc.setRate(800.0f);
    bResult = Gyro.init(ITG3200_ADDR_AD0_LOW, &hi2c1);
    if(!bResult) {
        while(1) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(50);
        }
    }
    HAL_Delay(1000);
    Gyro.zeroCalibrate(128, 5); // calibrate the ITG3200
    bResult = Magn.init(HMC5883L_Address, &hi2c1);
    if(!bResult) {
        while(1) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(50);
        }
    }
    int16_t error = Magn.SetScale(1300); // Set the scale (in milli Gauss) of the compass.
    if(error != 0) {
        Error_Handler();
    }
    error = Magn.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
    if(error != 0) {
        Error_Handler();
    }
}


// Timer2 provides periodic triggers to start ADC conversion
void
initTIM2(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    // Clock enable
    __HAL_RCC_TIM2_CLK_ENABLE();
    // Time base configuration (100 MHz max CPU frequency)
    // Prescaler value to have a 40 KHz TIM2 counter clock
    const uint32_t counterClock = SystemCoreClock/10;
    uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock/2)/counterClock)-1;

    memset(&Tim2Handle, 0, sizeof(Tim2Handle));
    Tim2Handle.Instance = TIM2;
    Tim2Handle.Init.Period            = (counterClock/SAMPLING_FREQ)-1;
    Tim2Handle.Init.Prescaler         = uwPrescalerValue;
    Tim2Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // tDTS=tCK_INT
    Tim2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    Tim2Handle.Init.RepetitionCounter = 0;
    Tim2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Base_Init(&Tim2Handle) != HAL_OK) {
        Error_Handler();
    }

    memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&Tim2Handle, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
} // void initTIM2()


void
TIM2_IRQHandler(void) {
    if(Acc.getInterruptSource(7)) {
        Acc.get_Gxyz(&values[0]);
    }
    if(Gyro.isRawDataReadyOn()) {
        Gyro.readGyro(&values[3]);
    }
    if(Magn.isDataReady()) {
        Magn.ReadScaledAxis(&values[6]);
    }

    Madgwick.update(values[3], values[4], values[5],
                    values[0], values[1], values[2],
                    values[6], values[7], values[8]);
    __HAL_TIM_CLEAR_IT(&Tim2Handle, TIM_IT_UPDATE);
}


void
SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
    memset(&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK   |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1  |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}


static void
MX_I2C1_Init(void) {
    memset(&hi2c1, 0, sizeof(hi2c1));
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}


static void
MX_USART2_UART_Init(void) {
    memset(&huart2, 0, sizeof(huart2));
    huart2.Instance = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}


static void
MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}


void
Error_Handler(void) {
    HAL_TIM_Base_Stop_IT(&Tim2Handle);
    while(true) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(50);
    }
}


#ifdef  USE_FULL_ASSERT
void
assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
