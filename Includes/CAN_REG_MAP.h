/*
   CAN_REG_MAP.h
   Created on: 18 Mar 2020
       Author: Aly
 */

#ifndef INC_CAN_REG_MAP_H_
#define INC_CAN_REG_MAP_H_

#include "stm32f4xx_hal_can.h"

#define RESET_VALUE 0x10002

#define FilterBanks_Base_R CAN1+ 0x240U

#define ON 1U
#define OFF 0U

#define MailBox0  0U
#define MailBox1  1U
#define MailBox2  2U

#define Hwreg_Bitseg1_Shift  16U
#define Hwreg_Bitseg2_Shift  20U
#define Hwreg_SJW_Shift  24U

#define MailBox0_INT_CLEAR_FLAG   0x1U
#define MailBox1_INT_CLEAR_FLAG   0x100U
#define MailBox2_INT_CLEAR_FLAG   0x10000U

#define ERRI_Clear 0x4U
#define WKUI_Clear 0x8U


#define ERROR_MASK 7U
#define BOFF 0x7U
#define EPASS 0x3U
#define EWARN 0x1U

/*Controller0*/
#define CAN0_MCR_R     (*((volatile unsigned long *)0x40006400U))
#define CAN0_MSR_R     (*((volatile unsigned long *)0x40006404U))
#define CAN0_TSR_R     (*((volatile unsigned long *)0x40006408U))
#define CAN0_RF0R_R    (*((volatile unsigned long *)0x4000640CU))
#define CAN0_RF1R_R    (*((volatile unsigned long *)0x40006410U))
#define CAN0_IER_R     (*((volatile unsigned long *)0x40006414U))
#define CAN0_ESR_R     (*((volatile unsigned long *)0x40006418U))
#define CAN0_BTR_R     (*((volatile unsigned long *)0x4000641CU))
#define CAN0_FIFO_0_RIR_R     (*((volatile unsigned long *)0x400065B0U))
#define CAN0_FIFO_0_RDTR_R     (*((volatile unsigned long *)0x400065B4U))
#define CAN0_FIFO_0_RDLR_R     (*((volatile unsigned long *)0x400065B8U))
#define CAN0_FIFO_0_RDHR_R     (*((volatile unsigned long *)0x400065BCU))
#define CAN0_FIFO_1_RIR_R      (*((volatile unsigned long *)0x400065C0U))
#define CAN0_FIFO_1_RDTR_R     (*((volatile unsigned long *)0x400065C4U))
#define CAN0_FIFO_1_RDLR_R     (*((volatile unsigned long *)0x400065C8U))
#define CAN0_FIFO_1_RDHR_R     (*((volatile unsigned long *)0x400065CCU))
#define CAN0_FMR_R     (*((volatile unsigned long *)0x40006600U))
#define CAN0_FA1R_R    (*((volatile unsigned long *)0x4000661CU))
#define CAN0_FM1R_R    (*((volatile unsigned long *)0x40006604U))
#define CAN0_FS1R_R    (*((volatile unsigned long *)0x4000660CU))
#define CAN0_FFA1R_R   (*((volatile unsigned long *)0x40006614U))


/*Controller1*/
#define CAN1_MCR_R     (*((volatile unsigned long *)0x40006800U))
#define CAN1_MSR_R    (*((volatile unsigned long *)0x40006804U))
#define CAN1_TSR_R    (*((volatile unsigned long *)0x40006808U))
#define CAN1_RF0R_R    (*((volatile unsigned long *)0x4000680C))
#define CAN1_RF1R_R    (*((volatile unsigned long *)0x40006810U))
#define CAN1_IER_R     (*((volatile unsigned long *)0x40006814U))
#define CAN1_ESR_R     (*((volatile unsigned long *)0x40006818U))
#define CAN1_BTR_R     (*((volatile unsigned long *)0x4000681CU))
#define CAN1_FIFO_0_RIR_R     (*((volatile unsigned long *)0x400069B0U))
#define CAN1_FIFO_0_RDTR_R     (*((volatile unsigned long *)0x400069B4U))
#define CAN1_FIFO_0_RDLR_R     (*((volatile unsigned long *)0x400069B8U))
#define CAN1_FIFO_0_RDHR_R     (*((volatile unsigned long *)0x400069BCU))
#define CAN1_FIFO_1_RIR_R      (*((volatile unsigned long *)0x400069C0U))
#define CAN1_FIFO_1_RDTR_R     (*((volatile unsigned long *)0x400069C4U))
#define CAN1_FIFO_1_RDLR_R     (*((volatile unsigned long *)0x400069C8U))
#define CAN1_FIFO_1_RDHR_R     (*((volatile unsigned long *)0x400069CCU))








#endif /* INC_CAN_REG_MAP_H_ */
