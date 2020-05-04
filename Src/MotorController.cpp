#include "MotorController.h"

#include <algorithm> // min() & max()
#include "string.h"  // for memset()


using namespace std; // min() & max()


MotorController::MotorController(Pin _ena, Pin _enb,
                                 Pin _in1, Pin _in2, Pin _in3, Pin _in4,
                                 double _motorAConst, double _motorBConst)
{
    ena = _ena;
    enb = _enb;
    in1 = _in1;
    in2 = _in2;
    in3 = _in3;
    in4 = _in4;
    motorAConst = _motorAConst;
    motorBConst = _motorBConst;
}


int32_t
MotorController::map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void
MotorController::move(int leftSpeed, int rightSpeed, int minAbsSpeed) {
    if (rightSpeed < 0) {
        rightSpeed = min(rightSpeed, -1*minAbsSpeed);
        rightSpeed = max(rightSpeed, -255);
    }
    else if (rightSpeed > 0) {
        rightSpeed = max(rightSpeed, minAbsSpeed);
        rightSpeed = min(rightSpeed, 255);
    }
    
    int realRightSpeed = map(abs(rightSpeed), 0, 255, minAbsSpeed, 255);

    if (leftSpeed < 0) {
        leftSpeed = min(leftSpeed, -1*minAbsSpeed);
        leftSpeed = max(leftSpeed, -255);
    }
    else if (leftSpeed > 0) {
        leftSpeed = max(leftSpeed, minAbsSpeed);
        leftSpeed = min(leftSpeed, 255);
    }
    
    int realLeftSpeed = map(abs(leftSpeed), 0, 255, minAbsSpeed, 255);
    
    HAL_GPIO_WritePin(in3.GPIO_Port, in3.GPIO_Pin, rightSpeed > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in4.GPIO_Port, in4.GPIO_Pin, rightSpeed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(in1.GPIO_Port, in1.GPIO_Pin, leftSpeed  > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2.GPIO_Port, in2.GPIO_Pin, leftSpeed  > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Set the pulse value for channel 1
     TIM3->CCR1 = realRightSpeed * motorAConst; // ???????
//    sConfig.Pulse = realRightSpeed * motorAConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
    // Set the pulse value for channel 2
    TIM3->CCR2 = realLeftSpeed  * motorBConst; // ???????
//    sConfig.Pulse = realLeftSpeed  * motorBConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 1
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 2
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }
}


void
MotorController::move(int speed, int minAbsSpeed) {
    int direction = 1;
    
    if (speed < 0) {
        direction = -1;
        speed = min(speed, -1*minAbsSpeed);
        speed = max(speed, -255);
    }
    else {
        speed = max(speed, minAbsSpeed);
        speed = min(speed, 255);
    }
    
    if (speed == currentSpeed) return;
    
    int realSpeed = max(minAbsSpeed, abs(speed));
    
    HAL_GPIO_WritePin(in1.GPIO_Port, in1.GPIO_Pin, speed > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2.GPIO_Port, in2.GPIO_Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(in3.GPIO_Port, in3.GPIO_Pin, speed > 0 ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in4.GPIO_Port, in4.GPIO_Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Set the pulse value for channel 1
    TIM3->CCR1 = realSpeed * motorAConst; // ???????
//    sConfig.Pulse = realSpeed * motorAConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
    // Set the pulse value for channel 2
    TIM3->CCR2 = realSpeed * motorBConst; // ???????
//    sConfig.Pulse = realSpeed  * motorBConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 1
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 2
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }

    currentSpeed = direction * realSpeed;
}


void
MotorController::move(int speed) {
    if (speed == currentSpeed) return;
    
    if (speed > 255) speed = 255;
    else if (speed < -255) speed = -255;
    
    HAL_GPIO_WritePin(in1.GPIO_Port, in1.GPIO_Pin, speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2.GPIO_Port, in2.GPIO_Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(in3.GPIO_Port, in3.GPIO_Pin, speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in4.GPIO_Port, in4.GPIO_Pin, speed > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Set the pulse value for channel 1
    TIM3->CCR1 = abs(speed) * motorAConst; // ???????
//    sConfig.Pulse = abs(speed) * motorAConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
    // Set the pulse value for channel 2
    TIM3->CCR2 = abs(speed)  * motorBConst; // ???????
//    sConfig.Pulse = abs(speed)  * motorBConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 1
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 2
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }

    currentSpeed = speed;
}


void
MotorController::turnLeft(int speed, bool kick) {
    HAL_GPIO_WritePin(in1.GPIO_Port, in1.GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2.GPIO_Port, in2.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in3.GPIO_Port, in3.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in4.GPIO_Port, in4.GPIO_Pin, GPIO_PIN_SET);
    
    if (kick) {
        TIM3->CCR1 = 255; // ???????
        TIM3->CCR2 = 255; // ???????
//        sConfig.Pulse = 255;
//        if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
//            // Configuration Error
//            Error_Handler();
//        }
//        // Set the pulse value for channel 2
//        sConfig.Pulse = 255;
//        if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
//            // Configuration Error
//            Error_Handler();
//        }
//        // Start PWM signal on channel 1
//        if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
//            // PWM Generation Error
//            Error_Handler();
//        }
//        // Start PWM signal on channel 2
//        if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
//            // PWM Generation Error
//            Error_Handler();
//        }
        HAL_Delay(100);
    }
    
    // Set the pulse value for channel 1
    TIM3->CCR1 = speed * motorAConst; // ???????
    TIM3->CCR2 = speed * motorBConst; // ???????
//    sConfig.Pulse = speed * motorAConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
//    // Set the pulse value for channel 2
//    sConfig.Pulse = speed * motorBConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 1
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 2
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }
}


void
MotorController::turnRight(int speed, bool kick) {
    HAL_GPIO_WritePin(in1.GPIO_Port, in1.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2.GPIO_Port, in2.GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in3.GPIO_Port, in3.GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in4.GPIO_Port, in4.GPIO_Pin, GPIO_PIN_RESET);
 
    if (kick) {
        TIM3->CCR1 = 255; // ???????
        TIM3->CCR2 = 255; // ???????
        // Set the pulse value for channel 1
//        sConfig.Pulse = 255;
//        if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
//            // Configuration Error
//            Error_Handler();
//        }
//        // Set the pulse value for channel 2
//        if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
//            // Configuration Error
//            Error_Handler();
//        }
//        // Start PWM signal on channel 1
//        if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
//            // PWM Generation Error
//            Error_Handler();
//        }
//        // Start PWM signal on channel 2
//        if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
//            // PWM Generation Error
//            Error_Handler();
//        }
        HAL_Delay(100);
    }
    // Set the pulse value for channel 1
    TIM3->CCR1 = speed * motorAConst; // ???????
    TIM3->CCR2 = speed * motorBConst; // ???????
//    sConfig.Pulse = speed * motorAConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
//    // Set the pulse value for channel 2
//    sConfig.Pulse = speed * motorBConst;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 1
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 2
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }
}


void
MotorController::stopMoving() {
    HAL_GPIO_WritePin(in1.GPIO_Port, in1.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2.GPIO_Port, in2.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in3.GPIO_Port, in3.GPIO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in4.GPIO_Port, in4.GPIO_Pin, GPIO_PIN_RESET);

    TIM3->CCR1 = 0; // ???????
    TIM3->CCR2 = 0; // ???????
//    // Set the pulse value for channel 1
//    sConfig.Pulse = 0;
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
//    // Set the pulse value for channel 2
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
//        // Configuration Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 1
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }
//    // Start PWM signal on channel 2
//    if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
//        // PWM Generation Error
//        Error_Handler();
//    }

    currentSpeed = 0;
}
