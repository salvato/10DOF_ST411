// Per la configurazione di QtCreatore vai a:
// https://www.youtube.com/watch?v=YgHe3D1t3Fs
//
#include "main.h"
#include "string.h" // for memset()
#include "stdio.h"
#include "stm32f4xx_hal.h"

#include <ADXL345.h>
#include <ITG3200.h>
#include <HMC5883L.h>
#include "MadgwickAHRS.h"
#include "MotorController.h"
#include "PID_v1.h"


//============================
// USART2 GPIO Configuration
//============================
// GND (CN10 20)    ------> GND
// PA2 (CN10 35)    ------> USART2_TX
// PA3 (CN10 37)    ------> USART2_RX


//============================
// I2C1 GPIO Configuration
//============================
// VIN (CN7  18)    ------> +5V
// GND (CN7  20)    ------> GND
// PB6 (CN10 17)    ------> I2C1_SCL
// PB7 (CN7  21)    ------> I2C1_SDA


#define I2C_SPEEDCLOCK 400000 // Hz
#define MIN_ABS_SPEED      20


static void SystemClock_Config(void);
static void GPIO_Init(void);
static void I2C1_Init(void);
static void USART2_UART_Init(void);
static void TIM3_Init(void);
static void Sensors_Init();
static void TIM2_Init(void);
static void executeCommand();
static bool isStationary();


bool
isStationary() {
    return true;
}


//=====================================================================
// Remember: Each used module MUST be enabled in stm32f4xx_hal_conf.h
//=====================================================================
I2C_HandleTypeDef  hi2c1;  // The Sensors Interface Handle
UART_HandleTypeDef huart2; // The Serial Interface Handle
TIM_HandleTypeDef  htim2;  // The Time Base Handle
TIM_HandleTypeDef  htim3;  // The PWM generator Handle for the Motors


ADXL345  Acc;      // 400KHz I2C Capable. Maximum Output Data Rate is 800 Hz
ITG3200  Gyro;     // 400KHz I2C Capable
HMC5883L Magn;     // 400KHz I2C Capable, left at the default 15Hz data Rate
Madgwick Madgwick; // ~13us per Madgwick.update() with NUCLEO-F411RE


uint32_t samplingFrequency = 300; // Hz


static float values[9];
static int UpdateRemote = 0;
static float q0, q1, q2, q3;
static HAL_StatusTypeDef result;
uint8_t sMessage[255];
uint8_t inBuf[256];
uint8_t sCommand[256];
uint8_t inChar;
int32_t nCharRead;
volatile bool bConnected;
volatile uint32_t nBusy = 0;


//================
// PID Regulator
//================
double input, output;
double Kp = 50.0;
double Kd = 1.4;
double Ki = 60.0;
double originalSetpoint = 175.8;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
int moveState = 0; // 0 = balance; 1 = back; 2 = forth
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


//===================
// Motor Controller
//===================
double motorSpeedFactorLeft  = 0.6;
double motorSpeedFactorRight = 0.5;
MotorController MotorController(Pin{GPIOA, GPIO_PIN_6},// ena
                                Pin{GPIOA, GPIO_PIN_7},// enb
                                Pin{GPIOC, GPIO_PIN_0},// in1
                                Pin{GPIOC, GPIO_PIN_1},// in2
                                Pin{GPIOC, GPIO_PIN_2},// in3
                                Pin{GPIOC, GPIO_PIN_3},// in4
                                motorSpeedFactorLeft,
                                motorSpeedFactorRight);


//==========================================================================


int
main(void) {
    bConnected = false;
    HAL_Init();
    SystemClock_Config();
    USART2_UART_Init(); // The Virtual Comunication Port (VCP)
    GPIO_Init();        // Various Input/Output Pin Initialization
    I2C1_Init();        // I2C Interface to AHRS Sensors
    TIM2_Init();        // AHRS update interrupt generator
    TIM3_Init();        // Motor PWM Generator
    Sensors_Init();     // 10DOF Sensor Initialization

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(100); // in ms
    pid.SetOutputLimits(-255, 255);

    // Wait for USER Button press before starting the Communication
    // while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1) ;
    // Wait for USER Button release before starting the Communication
    // while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 0) ;

    Madgwick.begin(float(samplingFrequency));

    // Get the first Sensor data
    while(!Acc.getInterruptSource(7)) {}
    Acc.get_Gxyz(&values[0]);
    while(!Gyro.isRawDataReadyOn()) {}
    Gyro.readGyro(&values[3]);
    while(!Magn.isDataReady()) {}
    Magn.ReadScaledAxis(&values[6]);

    // Initial estimate of the attitude (assumed a static sensor !)
    for(int i=0; i<10000; i++) { // ~13us per Madgwick.update() with NUCLEO-F411RE
        Madgwick.update(values[3], values[4], values[5], // Gyro in degrees/sec
                        values[0], values[1], values[2], // Acc
                        values[6], values[7], values[8]);// Mag
    }

//    // Start PWM signal on channel 1
//    if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
//        Error_Handler();
//    }
//    // Start PWM signal on channel 2
//    if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) {
//        Error_Handler();
//    }

    // Start the Comunication with the Remote
    nCharRead = 0;
    while(!bConnected) {
        while(HAL_UART_Receive(&huart2, inBuf, 2, 1000) != HAL_OK)
            ;
        if(inBuf[0] == 'C') {
            sMessage[0] = 'C';// Echoes back to Accept the Connection
            sMessage[1] = '#';
            result = HAL_UART_Transmit(&huart2, sMessage, 2, 1000);
            if(result != HAL_OK) {
                Error_Handler();
            }
            bConnected = true;
            HAL_Delay(500);
        }
    }

    // Start periodic update interrupt !
    if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }

    nCharRead = 0;
    HAL_UART_Receive_IT(&huart2, &inChar, 1);

    while(true) {
    }

}


void
executeCommand() {
    if(sCommand[0] == 'D') { // Is a Disconnection Request
        bConnected = false;
        sMessage[0] = 'D';// Echoes back to Aknowledge the Disconnection
        sMessage[1] = '#';
        sMessage[2] = 0;
        result = HAL_UART_Transmit_IT(&huart2, sMessage, strlen((char*)sMessage));
        if(result != HAL_OK) {
            Error_Handler();
        }
    }
}


void
Sensors_Init() {
    bool bResult;

    // Accelerator Init
    bResult = Acc.init(ADXL345_ADDR_ALT_LOW, &hi2c1);
    if(!bResult) {
        while(1) {
            Error_Handler();
        }
    }
    Acc.setRangeSetting(2); // +/- 2g. Possible values are: 2g, 4g, 8g, 16g

    // Gyroscope Init
    bResult = Gyro.init(ITG3200_ADDR_AD0_LOW, &hi2c1);
    if(!bResult) {
        Error_Handler();
    }
    if(!isStationary()) {
        HAL_Delay(100);
        Gyro.zeroCalibrate(600); // calibrate the ITG3200
    }
    else { // Use Precalculated Values
        Gyro.offsets[0] = -20.0;
        Gyro.offsets[1] = -110.0;
        Gyro.offsets[2] = -35.0;
    }
    // Magnetometer Init
    bResult = Magn.init(HMC5883L_Address, &hi2c1);
    if(!bResult) {
        Error_Handler();
    }
    HAL_Delay(100);
    int16_t error = Magn.SetScale(1300); // Set the scale (in milli Gauss) of the compass.
    if(error != 0) {
        Error_Handler();
    }
    HAL_Delay(100);
    error = Magn.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
    if(error != 0) {
        Error_Handler();
    }
}


//==================================================
// Periodic AHRS Update Interrupt Initialization !
//==================================================
void
TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    // Clock enable
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Time base configuration (based on 100 MHz CPU frequency)
    const uint32_t counterClock = SystemCoreClock/100;// 1MHz;

    // Prescaler value to have a 1MHz TIM2 Input Counter Clock
    uint32_t uwPrescalerValue = (uint32_t) (SystemCoreClock/counterClock)-1;

    memset(&htim2, 0, sizeof(htim2));
    htim2.Instance = TIM2;
    htim2.Init.Period            = (counterClock/samplingFrequency)-1; // (Sampling period) / (Clock Period)
    htim2.Init.Prescaler         = uwPrescalerValue;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // tDTS=tCK_INT
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.RepetitionCounter = 0;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    samplingFrequency = counterClock/(htim2.Init.Period+1); // The Real Obtained Value

    memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
} // void TIM2_Init()


void
TIM2_IRQHandler(void) {
    Acc.get_Gxyz(&values[0]);
    Gyro.readGyro(&values[3]);
    Magn.ReadScaledAxis(&values[6]);
	Madgwick.update(values[3], values[4], values[5],
					values[0], values[1], values[2],
					values[6], values[7], values[8]);

// yaw:   (about Z axis)
// pitch: (nose up/down, about Y axis)
// roll:  (tilt left/right, about X axis)

//    input = Madgwick.getPitch() + 180;
//    pid.Compute();
//    MotorController.move(output, MIN_ABS_SPEED);

    if(bConnected) {
        UpdateRemote++;
        UpdateRemote %= (samplingFrequency/20);
        if(!UpdateRemote) {
            Madgwick.getRotation(&q0, &q1, &q2, &q3);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            sprintf((char *)sMessage, "q %d %d %d %d#", int(q0*1000), int(q1*1000) ,int(q2*1000), int(q3*1000));
            result = HAL_UART_Transmit_DMA(&huart2, sMessage, strlen((char *)sMessage));
//            if(result != HAL_OK) {
//                Error_Handler();
//            }

        }
    }

    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
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
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}


static void
I2C1_Init(void) {
    memset(&hi2c1, 0, sizeof(hi2c1));
    hi2c1.Instance = I2C1;
    HAL_I2C_MspDeInit(&hi2c1);

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
USART2_UART_Init(void) {
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
GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

    __HAL_RCC_GPIOC_CLK_ENABLE(); // Motors Pins
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE(); // On Board Led
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Initial Satus of On Board Led
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    // Initial Satus of Motors Pins
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

    // Setup On Board Push Button
    GPIO_InitStruct.Pin  = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Setup On Board Led
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Setup Motor Controller
    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 ;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}


static void
TIM3_Init(void) { // TIM3 is a General Purpose Counter
    TIM_ClockConfigTypeDef  sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef      sConfigOC;
    memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
    memset(&sMasterConfig,      0, sizeof(sMasterConfig));
    memset(&sConfigOC,          0, sizeof(sConfigOC));

/* --------------------------------------------------------------------------------------------
    TIM3 Configuration to generate 2 PWM signals.

      In this example TIM3 input clock (TIM3CLK) is set to APB1 clock x 2,
      since APB1 prescaler is equal to 2 (see SystemClock_Config())
        TIM3CLK = APB1CLK*2
        APB1CLK = HCLK/2
         => TIM3CLK = HCLK = SystemCoreClock

      To get TIM3 counter clock at 4096000Hz, the prescaler is computed as follows:
         Prescaler = (TIM3CLK / TIM3 counter clock) - 1
         Prescaler = ((SystemCoreClock) /4096000 Hz) - 1

      To get TIM3 output clock at 4 KHz, the period (ARR)) is computed as follows:
         ARR = (TIM3 counter clock / TIM3 output clock) - 1
             = 255

      TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR + 1) * 100 = 50%
      TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR + 1) * 100 = 37.5%

      Note:
       SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
       Each time the core clock (HCLK) changes, user had to update SystemCoreClock
       variable value. Otherwise, any configuration based on this variable will be incorrect.
       This variable is updated in three ways:
        1) by calling CMSIS function SystemCoreClockUpdate()
        2) by calling HAL API function HAL_RCC_GetSysClockFreq()
        3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
-------------------------------------------------------------------------------------------- */


    // Compute the prescaler value to have TIM3 counter clock equal to 4096000Hz
    uint32_t uhPrescalerValue = (uint32_t)(SystemCoreClock / 4096000) - 1;
    htim3.Instance = TIM3;
    htim3.Init.Prescaler         = uhPrescalerValue;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 255; // ARR (Auto Reload Register)
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim3);
}


// This function handles DMA RX interrupt request.
void
DMA1_Stream5_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmarx);
}


// This function handles DMA TX interrupt request.
void
DMA1_Stream6_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmatx);
}


// This function handles USARTx interrupt request.
void
USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}



// Tx Transfer completed callback
void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
    UNUSED(UartHandle);
}


// Rx Transfer completed callback
void
HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
    UNUSED(UartHandle);
    inBuf[nCharRead] = inChar;
    nCharRead++;
    if(inChar == '#') {
        for(int i=0; i<nCharRead; i++)
            sCommand[i] = inBuf[i];
        nCharRead = 0;
        executeCommand();
    }
    HAL_UART_Receive_IT(&huart2, &inChar, 1);
}


//  UART error callbacks
void
HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
    UNUSED(UartHandle);
}



void
Error_Handler(void) {
    HAL_TIM_Base_Stop_IT(&htim2);
    while(true) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(50);
    }
}


void
Print_Error(char* sErrorMessage, int msgLen) {
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)sErrorMessage, msgLen);
    while(true) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
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
