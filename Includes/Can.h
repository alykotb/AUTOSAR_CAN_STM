
#ifndef INCLUDES_CAN_H_
#define INCLUDES_CAN_H_
#include "Can_Cfg.h"
#include "ComStack_Types.h"
#pragma CHECK_MISRA("none")
#include "Mcu_Cfg.h"
#include "Can_GeneralTypes.h"
#pragma RESET_MISRA("all")
/* shall not be included here */
/*[SWS_Can_00435] The Can.h file shall include Can_GeneralTypes.h.*/



void Can_Init(const Can_ConfigType* Config);
void Can_DeInit(void);
Std_ReturnType Can_SetControllerMode(uint8_t Controller,Can_ControllerStateType Transition);
Std_ReturnType Can_SetBaudrate(uint8 Controller, uint16 BaudRateConfigID);
Std_ReturnType Can_GetControllerMode(uint8_t Controller, Can_ControllerStateType* ControllerModePtr);
Std_ReturnType Can_GetControllerErrorState(uint8_t ControllerId, Can_ErrorStateType* ErrorStatePtr);
Std_ReturnType Can_Write(Can_HwHandleType Hth, const Can_PduType* PduInfo);
void Can_MainFunction_Write(void);
void Can_MainFunction_Read(void);
void Can_MainFunction_BusOff(void);
void Can_MainFunction_Mode(void);
void Can_MainFunction_Wakeup(void);
Std_ReturnType Can_CheckWakeup(uint8_t Controller);
void Can_DisableControllerInterrupts(uint8_t Controller);
void Can_EnableControllerInterrupts(uint8_t Controller);


#endif /* INCLUDES_CAN_H_ */
