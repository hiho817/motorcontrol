/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "structs.h"
#include "usart.h"
#include "fsm.h"
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "foc.h"
#include "can.h"
#include "position_sensor.h"
#include "hw_config.h"
#include "user_config.h"
#include "math_ops.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN
  uint32_t TxMailbox;
  pack_reply(&can_tx, comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR, VERSION_NUM, hall_cal.hall_cal_state, state.state, controller.i_q_des);	// Pack response
  HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

  /* Check for special Commands */
  if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) & (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFC))){
	  update_fsm(&state, MOTOR_CMD);
      }
  else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFD))){
      update_fsm(&state, MENU_CMD);
      }
  else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFE))){
	  update_fsm(&state, ZERO_CMD);
      }
  else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFA))){
      hall_cal.hall_cal_count = 0;
      hall_cal.hall_cal_state = 1; // calibrating
      /*----- convert theta_mech to 0~359.9999deg -----*/
      hall_cal.hall_present_pos = controller.theta_mech;
      hall_cal.hall_cal_pcmd = controller.theta_mech;
      static float _f_cal_round;
      modff(hall_cal.hall_cal_pcmd/(2*PI_F),&_f_cal_round);
      hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd - _f_cal_round*2*PI_F;
      if(hall_cal.hall_cal_pcmd < 0) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + 2*PI_F;
      update_fsm(&state, HALL_CAL_CMD);
  	  }
  else{
	  unpack_cmd(can_rx, controller.commands);	// Unpack commands
	  controller.timeout = 0;					// Reset timeout counter
  }

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	//LED_HIGH	// Useful for timing

	/* Sample ADCs */
	analog_sample(&controller);

	/* Sample position sensor */
	ps_sample(&comm_encoder, DT);

	/* Calibrate Hall Sensor */
	hall_calibrate();

	/* Run Finite State Machine */
	run_fsm(&state);

	/* Check for CAN messages */
	can_tx_rx();

	/* increment loop count */
	controller.loop_count++;
	//LED_LOW

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	HAL_UART_IRQHandler(&huart2);

	char c = Serial2RxBuffer[0];
	update_fsm(&state, c);

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void can_tx_rx(void){

	int no_mesage = HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN
	if(!no_mesage){
		uint32_t TxMailbox;
		pack_reply(&can_tx, comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR, VERSION_NUM, hall_cal.hall_cal_state, state.state, controller.i_q_des);	// Pack response
		HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

		/* Check for special Commands */
		if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) & (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFC))){
			  update_fsm(&state, MOTOR_CMD);
			}
		else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFD))){
			update_fsm(&state, MENU_CMD);
			}
		else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFE))){
			  update_fsm(&state, ZERO_CMD);
			}
		else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFA))){
		      hall_cal.hall_cal_count = 0;
		      hall_cal.hall_cal_state = 1; // calibrating
		      /*----- convert theta_mech to 0~359.9999deg -----*/
		      hall_cal.hall_present_pos = controller.theta_mech;
		      hall_cal.hall_cal_pcmd = controller.theta_mech;
		      static float _f_cal_round;
		      modff(hall_cal.hall_cal_pcmd/(2*PI_F),&_f_cal_round);
		      hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd - _f_cal_round*2*PI_F;
		      if(hall_cal.hall_cal_pcmd < 0) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + 2*PI_F;
		      update_fsm(&state, HALL_CAL_CMD);
		  	  }
		else{
			  unpack_cmd(can_rx, controller.commands);	// Unpack commands
			  controller.timeout = 0;					// Reset timeout counter
		controller.i_mag_max = controller.i_q;
		}
	}

}

void hall_calibrate(void){
    if(hall_cal.hall_cal_state == 0 || hall_cal.hall_cal_state >= 2 );
    else{
        // read hall sensor
    	hall_cal.hall_input = HAL_GPIO_ReadPin(HALL_IO);
        // calculate new position
        if((HALL_CAL_DIR == 1 && controller.theta_mech >= hall_cal.hall_present_pos + 2*PI_F) || (HALL_CAL_DIR == -1 && controller.theta_mech <= hall_cal.hall_present_pos - 2*PI_F)){
        	hall_cal.hall_cal_state = 3 ;
            state.state = MENU_MODE ;
            state.state_change = 1 ;
        }
        else{
        	// rotate the motor forward and backward to read the hall sensor (1: no magnet detected, 0: magnet detected)
        	// record the position at the moment from 1 to 0 (in_pos), and keep rotating
            // record the position at the moment from 0 to 1 (out_pos), and stop rotating.
        	// calculate the average value of in_pos and out_pos, and rotate the motor to that position slowly
            if(hall_cal.hall_input != hall_cal.hall_preinput ) {
            	hall_cal.hall_cal_count += 1 ;
                if(hall_cal.hall_input == 0) hall_cal.hall_in_pos = controller.theta_mech ;
                else{
                	hall_cal.hall_out_pos = controller.theta_mech ;
                	hall_cal.hall_mid_pos = (hall_cal.hall_in_pos + hall_cal.hall_out_pos)/2.0f ;
                }
            }
            if(hall_cal.hall_cal_count <= 1) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + HALL_CAL_DIR*(1.0f/(40000.0f)*HALL_CAL_SPEED ) ;
            else{
                if(HALL_CAL_DIR == 1 ){
                    if(HALL_CAL_OFFSET == 0){
                        // keep turning
                        if(controller.theta_mech >= hall_cal.hall_mid_pos) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd - HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED ;
                        else{
                        	// stop
                        	hall_cal.hall_cal_pcmd = 0.0f;
                        	hall_cal.hall_cal_state = 2; // success
                            // zero
                            hall_cal.hall_cal_count = 0 ;
                            state.state = MOTOR_MODE;
                        }
                    }
                    else{
                        if(controller.theta_mech <= hall_cal.hall_mid_pos + HALL_CAL_OFFSET*PI_F/180)  hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED ;
                        else{
                        	// stop
                        	hall_cal.hall_cal_pcmd = 0.0f;
                        	hall_cal.hall_cal_state = 2; // success
                            // zero
                            hall_cal.hall_cal_count = 0 ;
                            state.state = MOTOR_MODE;
                        }
                    }
                }
                else if(HALL_CAL_DIR == -1){
                    if(HALL_CAL_OFFSET == 0){
                        // keep turning
                        if(controller.theta_mech <= hall_cal.hall_mid_pos) hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd - HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED ;
                        else{
                        	// stop
                        	hall_cal.hall_cal_pcmd = 0.0f;
                        	hall_cal.hall_cal_state = 2; // success
                            // zero
                            hall_cal.hall_cal_count = 0 ;
                            state.state = MOTOR_MODE;
                        }
                    }
                    else{
                    	// calibrate_offset != 0
                        if(controller.theta_mech >= hall_cal.hall_mid_pos - HALL_CAL_OFFSET*PI_F/180)  hall_cal.hall_cal_pcmd = hall_cal.hall_cal_pcmd + HALL_CAL_DIR*1.0f/40000.0f*HALL_CAL_SPEED ;
                        else{
                        	// stop
                        	hall_cal.hall_cal_pcmd = 0.0f;
                        	hall_cal.hall_cal_state = 2; // success
                            // zero
                            hall_cal.hall_cal_count = 0 ;
                            state.state = MOTOR_MODE;
                        }
                    }
                }
            }
            hall_cal.hall_cal_pcmd = (hall_cal.hall_cal_pcmd>2*PI_F) ? hall_cal.hall_cal_pcmd-=2*PI_F : hall_cal.hall_cal_pcmd ;
            hall_cal.hall_cal_pcmd = (hall_cal.hall_cal_pcmd < 0)  ? hall_cal.hall_cal_pcmd+=2*PI_F : hall_cal.hall_cal_pcmd ;
            controller.p_des = hall_cal.hall_cal_pcmd ;
        }
        hall_cal.hall_preinput = hall_cal.hall_input ;
    }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
