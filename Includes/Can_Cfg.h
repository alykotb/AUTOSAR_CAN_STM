#ifndef INCLUDES_CAN_CFG_H_  
#define INCLUDES_CAN_CFG_H_  

#define CPU_CLOCK (16000000U)


#define False 0U  
#define True  1U

#define  CanDevErrorDetect 0U
#define  CanIndex 0U
#define  CanMainFunctionBusoffPeriod (float32)(0.05)
#define  CanMainFunctionModePeriod (float32)(0.05)
#define  CanMainFunctionWakeupPeriod (float32)(0.05)
#define  CanMultiplexedTransmission 1U
#define  CanPublicIcomSupport       0U
#define  CanSetBaudrateApi 1U
#define  CanTimeoutDuration (float32)(0.05)
#define  CanVersionInfoApi 0U


#define Can_MainFunction_Write0_Period_index 0U
#define Can_MainFunction_Read0_Period_index  1U

#define More_Than_Write_period  0U
#define More_Than_Read_period  0U

#define  NUM_OF_ACTIVE_CAN_CONTROLLERS  1U

#define CanController0Activation 1U
#define CanController1Activation 0U

#define Controller0_ID 0U
#define Controller1_ID 1U



#define UserCANCfg \
{.CanConfigSet.CanController =\
    {\
{\
.CanControllerId = 0U,\
.CanControllerBaseAddress = 0x40006400U,\
.CanCpuClockRef=&McuPerClockConfigData,\
.CanControllerDefaultBaudrate =&CanController0_BaudrateConfig[0U],\
},\
{\
.CanControllerId = 1U,\
.CanControllerBaseAddress = 0x40006800U,\
.CanCpuClockRef=&McuPerClockConfigData,\
.CanControllerDefaultBaudrate =&CanController1_BaudrateConfig[0U]\
 }\
   },\
.CanConfigSet.CanHardwareObject =\
 {\
  {\
.CanHandleType = FULL,\
.CanObjectType = transmit,\
.CanIdType = STANDARD,\
.CanObjectId = 0U,\
.CanTriggerTransmitEnable = False,\
.CanHwObjectCount = 3U,\
.CanTriggerTransmitEnable=False,\
.CanHardwareObjectUsesPolling=False,\
.CanControllerRef = &CanContainer.CanConfigSet.CanController[0],\
 },\
  {\
.CanHandleType = FULL,\
.CanObjectType = receive,\
.CanIdType =EXTENDED,\
.CanObjectId = 1U,\
.CanTriggerTransmitEnable = False,\
.CanHwObjectCount = 3U,\
.CanHardwareObjectUsesPolling=False,\
.CanTriggerTransmitEnable=False,\
.CanControllerRef = &CanContainer.CanConfigSet.CanController[0],\
 .CanHwFilter=\
 {\
.CanHwFilterMask =0x1FFFFFFFU,\
.CanHwFilterCode =0x0U,\
 }\
 }\
 }\
};

#define Controller0_BaudrateConfigs \
{\
 {\
.CanControllerBaudRate = 500U,\
.CanControllerPropSeg = 2U,\
.CanControllerSeg1 = 6U,\
.CanControllerSeg2 = 7U,\
.CanControllerSyncJumpWidth = 4U,\
.CanControllerBaudRateConfigID = 0U,\
 },\
 {\
 .CanControllerBaudRate = 300U,\
.CanControllerPropSeg = 2U,\
.CanControllerSeg1 = 6U,\
.CanControllerSeg2 = 7U,\
.CanControllerSyncJumpWidth = 4U,\
.CanControllerBaudRateConfigID = 0U,\
 }\
};

#define Controller1_BaudrateConfigs \
{\
 {\
.CanControllerBaudRate = 300U,\
.CanControllerPropSeg = 2U,\
.CanControllerSeg1 =6U,\
.CanControllerSeg2 =7U,\
.CanControllerSyncJumpWidth =4U,\
.CanControllerBaudRateConfigID = 0U,\
 },\
 {\
 .CanControllerBaudRate = 300U,\
.CanControllerPropSeg = 2U,\
.CanControllerSeg1 = 6U,\
.CanControllerSeg2 = 7U,\
.CanControllerSyncJumpWidth = 4U,\
.CanControllerBaudRateConfigID = 0U,\
 }\
};

#define Controller0_BaudrateConfigs_Prescalers \
{\
2U,\
2U\
};

#define Controller1_BaudrateConfigs_Prescalers \
{\
2U,\
2U\
};





#define  NUM_OF_HTH  1U
#define  NUM_OF_HRH  1U
#define  NUM_OF_HOH  2U

#define  Num_of_TX0_MailBoxes    3U
#define  Num_of_TX1_MailBoxes    0U


#define Controller0_MailBox0 1U/*ON or OFF*/
#define Controller0_MailBox1 1U
#define Controller0_MailBox2 1U

#define Controller0_MailBox0_HTH 0U
#define Controller0_MailBox1_HTH 0U
#define Controller0_MailBox2_HTH 0U


#define  ACTIVE_Filter_Banks    3U
#define  FIFOs_to_FilterBanks  0x0U
#define  Filter_Banks_Scale     0U/*Value determines the scale for each of the used  filter banks (standard or extended ID),here one filter bank is used*/
#define  Filter_Banks_Used      1U

#define Controller0_FIFO_0_HRH  1U
#define Controller0_FIFO_1_HRH  1U

#define Controller1_FIFO_0_HRH  2U
#define Controller1_FIFO_1_HRH  2U

#define Controller0_FIFO_0_Processing  0U
#define Controller0_FIFO_1_Processing  0U

#define Controller1_FIFO_0_Processing  0U
#define Controller1_FIFO_1_Processing  0U








#define  Controller0_num_of_baudrates  2U
#define  Controller1_num_of_baudrates  2U



#define Controller0_START_MODE  0xCU
#define Controller0_STOP_MODE   0xDU
#define Controller0_SLEEP_MODE  0xEU

#define Controller1_START_MODE  0xCU
#define Controller1_STOP_MODE   0xDU
#define Controller1_SLEEP_MODE  0xEU


#define  INTERRUPT  0U
#define  POLLING    1U
#define  MIXED      2U

#define  CanTxProcessing0 0U
#define  CanRxProcessing0 0U
#define  CanBusoffProcessing0 0U
#define  CanWakeupProcessing0  0U
#define  CanWakeupFunctionalityAPI0 1U
#define  CanWakeupSupport0 1U
#define  CONTROLLER0_STARTMODE_INT_ENABLE 0x0U
#define  CONTROLLER0_SLEEPMODE_INT_ENABLE 0x10002U

#define  CanTxProcessing1 0U
#define  CanRxProcessing1 0U
#define  CanBusoffProcessing1 0U
#define  CanWakeupProcessing1  0U
#define  CanWakeupFunctionalityAPI1 1U
#define  CanWakeupSupport1 1U
#define  CONTROLLER1_STARTMODE_INT_ENABLE 0x1BU
#define  CONTROLLER1_SLEEPMODE_INT_ENABLE 0x12U

#endif
