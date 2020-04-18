

#include "main.h"
#include <float.h>
#include "Can.h"
#include "CAN_REG_MAP.H"

static void Recieve_objects_config (const Can_ConfigType* Config);
static inline void Controller0_StartTransition_Poll(void);
static inline void Controller1_StartTransition_Poll(void);
static inline void Controller0_StopTransition_Poll(void);
static inline void Controller1_StopTransition_Poll(void);
static inline void Controller0_SleepTransition_Poll(void);
static inline void Controller1_SleepTransition_Poll(void);

void Can0_TX_InterruptHandler(void);
void Can0_RX0_InterruptHandler(void);
void Can0_RX1_InterruptHandler(void);
void Can0_BUSOFF_and_WAKEUP_InterruptHandler(void);


void Can1_TX_InterruptHandler(void);
void Can1_RX0_InterruptHandler(void);
void Can1_RX1_InterruptHandler(void);
void Can1_BUSOFF_and_WAKEUP_InterruptHandler(void);

const Mcu_PerClockConfigType McuPerClockConfigData;

static const CanControllerBaudrateConfig CanController0_BaudrateConfig[Controller0_num_of_baudrates]=Controller0_BaudrateConfigs;
static const CanControllerBaudrateConfig CanController1_BaudrateConfig[Controller1_num_of_baudrates]= Controller1_BaudrateConfigs;

static const uint8_t Controller0_Prescalers[Controller0_num_of_baudrates]=Controller0_BaudrateConfigs_Prescalers;
static const uint8_t Controller1_Prescalers[Controller0_num_of_baudrates]=Controller1_BaudrateConfigs_Prescalers;

//uint64_t INT_Disable_counter[NUM_OF_ACTIVE_CAN_CONTROLLERS]={0,0};

const Can_ConfigType CanContainer=UserCANCfg;

static Can_ControllerStateType Can_ControllerMode [NUM_OF_ACTIVE_CAN_CONTROLLERS];


static Can_ControllerStateType Requested_Mode_Transition [NUM_OF_ACTIVE_CAN_CONTROLLERS];

static uint8_t IntDisableCount[NUM_OF_ACTIVE_CAN_CONTROLLERS];
//static uint8_t IntEnableCount[NUM_OF_ACTIVE_CAN_CONTROLLERS];

//static uint8_t INT_Enable_flag[NUM_OF_ACTIVE_CAN_CONTROLLERS];

static Can_StateType CanUnitState=CAN_UNINIT;


const static uint8_t HTH_To_MailBoxes_Map[NUM_OF_HTH];

static PduIdType Controller0_MailBox_Pending_PduId[Num_of_TX0_MailBoxes];
static PduIdType Controller1_MailBox_Pending_PduId[Num_of_TX1_MailBoxes];

static uint8_t Controller0_MailBoxes_semaphore[Num_of_TX0_MailBoxes];
static uint8_t Controller1_MailBoxes_semaphore[Num_of_TX1_MailBoxes];


static bool  state_transition_flag[NUM_OF_ACTIVE_CAN_CONTROLLERS];

static bool WakeUp_Flag[NUM_OF_ACTIVE_CAN_CONTROLLERS];



/*Helper Functions*/
static inline void Controller0_StartTransition_Poll(void)
{
	uint32_t tickstart;
    tickstart = HAL_GetTick();


    /*[SWS_Can_00398] ⌈The function Can_SetControllerMode shall use the system service GetCounterValue for
	timeout monitoring to avoid blocking functions.*/
    /*we cannot use this service because it is not available in our stack*/

    /*[SWS_Can_00372]In case the flag signals that the change takes no effect and the maximum time CanTimeoutDuration is elapsed,
	  the function Can_SetControllerMode shall be left and the function Can_Mainfunction_Mode shall continue to poll the flag.*/
    while ((CAN0_MSR_R & CAN_MSR_INAK) != zero)
	   {
		 /* Check for the Timeout */
		 if ((HAL_GetTick() - tickstart) >= (CanTimeoutDuration*CPU_CLOCK))
		 {
		   return;
		 }
		 else
		 {
			 /*MISRA*/
		 }
	   }
    Can_ControllerMode [Controller0_ID]=CAN_CS_STARTED;
}

static inline void Controller1_StartTransition_Poll(void)
{
	uint32_t tickstart;
    tickstart = HAL_GetTick();
    while ((CAN1_MSR_R & CAN_MSR_INAK) != zero)
	   {
		 /* Check for the Timeout */
		 if ((HAL_GetTick() - tickstart) >= (CanTimeoutDuration*CPU_CLOCK))
		 {
		   return;
		 }
		 else
		 {
			 /*MISRA*/
		 }
	   }
  Can_ControllerMode [Controller1_ID]=CAN_CS_STARTED;
}

static inline void Controller0_StopTransition_Poll(void)
{
	uint32_t tickstart;
    tickstart = HAL_GetTick();
    while ((CAN0_MSR_R & CAN_MSR_INAK) == zero)
	   {
		 /* Check for the Timeout */
		 if ((HAL_GetTick() - tickstart) >= (CanTimeoutDuration*CPU_CLOCK))
		 {
			   return;
		 }
		 else
		 {
			 /*MISRA*/
		 }
	   }
  Can_ControllerMode [Controller0_ID]=CAN_CS_STOPPED;
}
static inline void Controller1_StopTransition_Poll(void)
{
	uint32_t tickstart;
    tickstart = HAL_GetTick();
    while ((CAN1_MSR_R & CAN_MSR_INAK) == zero)
	   {
		 /* Check for the Timeout */
		 if ((HAL_GetTick() - tickstart) >= (CanTimeoutDuration*CPU_CLOCK))
		 {
			   return;
		 }
		 else
		 {
			 /*MISRA*/
		 }
	   }
  Can_ControllerMode [Controller1_ID]=CAN_CS_STOPPED;
}
static inline void Controller0_SleepTransition_Poll(void)
{
	uint32_t tickstart;
    tickstart = HAL_GetTick();
    while ((CAN0_MSR_R & CAN_MSR_SLAK_Msk ) == zero)
	   {
		 /* Check for the Timeout */
		 if ((HAL_GetTick() - tickstart) >=(CanTimeoutDuration*CPU_CLOCK))
		 {
		   return;
		 }
		 else
		 {
			 /*MISRA*/
		 }
	   }
     Can_ControllerMode [Controller0_ID]=CAN_CS_SLEEP;
}
static inline void Controller1_SleepTransition_Poll(void)
{
	uint32_t tickstart;
    tickstart = HAL_GetTick();
    while ((CAN1_MSR_R & CAN_MSR_SLAK_Msk) == zero)
	   {
		 /* Check for the Timeout */
		 if ((HAL_GetTick() - tickstart) >= (CanTimeoutDuration*CPU_CLOCK))
		 {
			   return;
		 }
		 else
		 {
			 /*MISRA*/
		 }
	   }
	 Can_ControllerMode [Controller1_ID]=CAN_CS_SLEEP;
}

/*Helper function which Configure Receive Mailboxes*/
static void Recieve_objects_config (const Can_ConfigType* Config)
{

CAN_TypeDef *can_ip =CAN1;
uint8_t index0;
uint8_t Controller0_filter_bank_num=Controller0_Filterbanks_start;
uint8_t Controller1_filter_bank_num=Controller1_Filterbanks_start;

CAN0_FMR_R=CAN_FMR_FINIT|(Controller1_Filterbanks_start<< CAN_FMR_CAN2SB_Pos);
CAN0_FA1R_R=zero;/*Deactivating  all filter banks to be configured*/
CAN0_FS1R_R=Filter_Banks_Scale;/*Determines the scale of each filter banks standard or extended*/
CAN0_FM1R_R=zero;/*All banks are used in IDMASK mode*/


 for(index0=zero;index0<NUM_OF_HOH;index0++)

 {
   if(Config->CanConfigSet.CanHardwareObject[index0].CanObjectType== receive)
{

#if ((CanController0Activation)&&(CanController1Activation))
   if(Config->CanConfigSet.CanHardwareObject[index0].CanControllerRef->CanControllerBaseAddress==Config->CanConfigSet.CanController[zero].CanControllerBaseAddress)
   {
#endif

#if (CanController0Activation)
	if(Config->CanConfigSet.CanHardwareObject[index0].CanIdType==STANDARD)
	{


						can_ip->sFilterRegister[Controller0_filter_bank_num].FR1 =
					    (((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<STD_ID_POS)<< STD_ID_SHIFT)|
						((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<STD_ID_POS);
						                /*STD_ID_POS shifts the value of mask  to the MSB 11 bits in the register*/
					                    /*STD_ID_SHIFT shifts the value of mask to the MSB 16 bits  in the register  in case of standard identifier
					                     the bank uses its two 32 bit registers as one of the halves of the register as ID and the other as a mask
					                     and you are obliged to activate both registers for same FIFO so both are used with same parameters to prevent unwanted messages
					                     from coming into the Hardware*/
						can_ip->sFilterRegister[Controller0_filter_bank_num].FR2 =
						(((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<STD_ID_POS)<< STD_ID_SHIFT)|
						((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<STD_ID_POS);
	 }                                  /*STD_ID_MASK is used to*/
	else
	{
		                can_ip->sFilterRegister[Controller0_filter_bank_num].FR1 =Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<EXT_ID_POS;
		                /*EXTID_POS shifts the value of mask and filter to the MSB in the register*/
						can_ip->sFilterRegister[Controller0_filter_bank_num].FR2 =Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<EXT_ID_POS;
						   if (Config->CanConfigSet.CanHardwareObject[index0].CanIdType ==MIXED)
							   {
							       Controller0_filter_bank_num++;
							   can_ip->sFilterRegister[Controller0_filter_bank_num].FR1 =
									(((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<STD_ID_POS)<< STD_ID_SHIFT)|
									((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<STD_ID_POS);
							   can_ip->sFilterRegister[Controller0_filter_bank_num].FR2 =
										(((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<STD_ID_POS)<< STD_ID_SHIFT)|
										((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<STD_ID_POS);
							   }
							   else
							   {
									 /*MISRA*/
							   }
				 Controller0_filter_bank_num++;
			}
#endif

#if ((CanController0Activation)&&(CanController1Activation))
   }
   else
   {
#endif

#if (CanController1Activation)
			  /*Mailboxes of controller 1*/
	   if(Config->CanConfigSet.CanHardwareObject[index0].CanIdType==STANDARD)
	   	{
	   						can_ip->sFilterRegister[Controller1_filter_bank_num].FR1 =
	   										(((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<STD_ID_POS)<< STD_ID_SHIFT)|
	   										((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<STD_ID_POS);
	   						can_ip->sFilterRegister[Controller0_filter_bank_num].FR2 =
	   										(((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<STD_ID_POS)<< STD_ID_SHIFT)|
	   										((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<STD_ID_POS);
	   	 }
	   	else
	   	{
	   		                can_ip->sFilterRegister[Controller1_filter_bank_num].FR1 =Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<EXT_ID_POS;
	   						can_ip->sFilterRegister[Controller1_filter_bank_num].FR2 =Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<EXT_ID_POS;
	   					   if (Config->CanConfigSet.CanHardwareObject[index0].CanIdType ==MIXED)
							   {
								   Controller1_filter_bank_num++;
									can_ip->sFilterRegister[Controller1_filter_bank_num].FR1 =
									(((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<STD_ID_POS)<< STD_ID_SHIFT)|
									((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<STD_ID_POS);
								   can_ip->sFilterRegister[Controller1_filter_bank_num].FR2 =
											(((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterMask<<STD_ID_POS)<< STD_ID_SHIFT)|
											((uint32_t)Config->CanConfigSet.CanHardwareObject[index0].CanHwFilter.CanHwFilterCode<<STD_ID_POS);

							   }
							   else
							   {
									 /*MISRA*/
							   }
						  Controller1_filter_bank_num++;
				}
#endif
#if ((CanController0Activation)&&(CanController1Activation))
		   }
#endif
   CAN0_FFA1R_R=FIFOs_to_FilterBanks;/*Matching each filter to a FIFO*/
   CAN0_FMR_R=(Controller1_Filterbanks_start|(!CAN_FMR_FINIT));/*get out of filter initialisation mode*/
   CAN0_FA1R_R= ACTIVE_Filter_Banks;/*Activating used filter banks*/
  }
  else
  {
	      /*MISRA*/
  }

 }

}
/* Service Name     :Can_Init
Syntax           : Std_ReturnType Can_SetControllerMode (uint8 Controller,Can_ControllerStateType Transition)
Service ID[hex]:0x00
Sync/Async       : Synchronous
Re-entrancy       : Non Reentrant
Parameters (in)  :Config,Pointer to driver configuration.
Parameters(in out): None
Parameters (out) : None
Return Value     : None
DESCRIPTION      : This function initialises the module.*/
void Can_Init(const Can_ConfigType* Config)
{
#if ((CanController0Activation)||(CanController1Activation))
	Register_Baudrate_paramters Baudrate_parameters;

			/*[SWS_Can_00174]If development error detection for the Can module is enabled: The function Can_Init shall raise the error
			  CAN_E_TRANSITION if the driver is not in state CAN_UNINIT.*/

if(CanUnitState==CAN_UNINIT)

	        /*[SWS_Can_00246]  The function Can_Init shall change the module state to
	            CAN_READY, after initialising all controllers inside the HW Unit.*/
{
#if ((CanController0Activation)&&(CanController1Activation))
			/*[SWS_Can_00408] ⌈If development error detection for the Can module is enabled: The function Can_Init shall raise the error CAN_E_TRANSITION
			if the CAN controllers are not in state UNINIT.*/
	 if  ((Can_ControllerMode [zero]== CAN_CS_UNINIT) && ( Can_ControllerMode[one]== CAN_CS_UNINIT))
	       {
		       /*[SWS_Can_00245]  The function Can_Init shall initialise all CAN controllers
			    according to their configuration.*/
		      CAN0_MSR_R=WKUI_Clear;
		      CAN0_MCR_R=Controller0_STOP_MODE;/*This value passed to the register puts the can controller into initialisation modes and sets
		                                        the required bits for AUTOSAR compliant functionality as:
		                                                                                  1-no hardware recovery in case of bus off.
		                                                                                  2-[SWS_Can_00012]If the CAN hardware can be configured
		                                                                                  to lock the hardware object after reception to protect
		                                                                                  data received from being overwritten before CPU reads it.
		                                        and fulfilling requirements of user's configuration as:
		                                                                                  1-Multiplexed Transmission
                                                                                          2-TTCAN
                                               */

		      while ((CAN0_MSR_R & CAN_MSR_INAK) == zero){}/*waiting for hardware to enter INIT mode*/
			 Baudrate_parameters.SyncJumpWidth=(Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerSyncJumpWidth-1)<<Hwreg_SJW_Shift;

			 Baudrate_parameters.TimeSeg1=((Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerPropSeg+
				                                Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerSeg1)-1)<<Hwreg_Bitseg1_Shift;

			 Baudrate_parameters.TimeSeg2=(Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerSeg2-1)<<Hwreg_Bitseg2_Shift;

			  CAN0_BTR_R= Baudrate_parameters.SyncJumpWidth|
					    Baudrate_parameters.TimeSeg1|
						 Baudrate_parameters.TimeSeg2|
						 (Controller0_Prescalers[Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerBaudRateConfigID]-1);
			  /*End of controller 0 initialisation*/
			CAN1_MSR_R=WKUI_Clear;
		    CAN1_MCR_R=Controller1_STOP_MODE;
		    while ((CAN1_MSR_R & CAN_MSR_INAK) == zero){}
		   Baudrate_parameters.SyncJumpWidth=(Config->CanConfigSet.CanController[one].CanControllerDefaultBaudrate->CanControllerSyncJumpWidth-1)<<Hwreg_SJW_Shift;

		   Baudrate_parameters.TimeSeg1=((Config->CanConfigSet.CanController[one].CanControllerDefaultBaudrate->CanControllerPropSeg+
										Config->CanConfigSet.CanController[one].CanControllerDefaultBaudrate->CanControllerSeg1)-1)<<Hwreg_Bitseg1_Shift;

		   Baudrate_parameters.TimeSeg2=(Config->CanConfigSet.CanController[one].CanControllerDefaultBaudrate->CanControllerSeg2-1)<<Hwreg_Bitseg2_Shift;

		    CAN1_BTR_R= Baudrate_parameters.SyncJumpWidth|
					  Baudrate_parameters.TimeSeg1|
					  Baudrate_parameters.TimeSeg2|
					  (Controller1_Prescalers[Config->CanConfigSet.CanController[one].CanControllerDefaultBaudrate->CanControllerBaudRateConfigID]-1);
      /*End of controller 1 initialisation*/

		   /*[SWS_Can_00246]  The function Can_Init shall change the module state to
		      CAN_READY, after initialising all controllers inside the HW Unit.*/
		 Recieve_objects_config(Config);
		 CanUnitState = CAN_READY;
		 /*[SWS_Can_00250] The function Can_Init shall initialise:  static variables, including flags,
		    Common setting for the complete CAN HW unitCAN controller specific settings for each CAN controller*/

		  /* only function can_Init can change controller state from UNINIT to stopped*/
	 	 Can_ControllerMode [zero]=CAN_CS_STOPPED;
		 Can_ControllerMode [one]=CAN_CS_STOPPED;

	    }
#endif

#if ((CanController0Activation)&&(!CanController1Activation))
	 if  (Can_ControllerMode [zero]== CAN_CS_UNINIT)
		{
		 CAN0_MSR_R=WKUI_Clear;
		 CAN0_MCR_R=Controller0_STOP_MODE;/*This value passed to the register puts the can controller into initialisation modes and sets
				                                        the required bits for AUTOSAR compliant functionality as:
				                                                                                  1-no hardware recovery in case of bus off.
				                                                                                  2-[SWS_Can_00012]If the CAN hardware can be configured
				                                                                                  to lock the hardware object after reception to protect
				                                                                                  data received from being overwritten before CPU reads it.
				                                        and fulfilling requirements of user's configuration as:
				                                                                                  1-Multiplexed Transmission
		                                                                                          2-TTCAN
		                                               */
		              while ((CAN0_MSR_R & CAN_MSR_INAK) == zero){}
					 Baudrate_parameters.SyncJumpWidth=(Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerSyncJumpWidth-1)<<Hwreg_SJW_Shift;

					 Baudrate_parameters.TimeSeg1=((Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerPropSeg+
						                                Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerSeg1)-1)<<Hwreg_Bitseg1_Shift;

					 Baudrate_parameters.TimeSeg2=(Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerSeg2-1)<<Hwreg_Bitseg2_Shift;

					  CAN0_BTR_R= Baudrate_parameters.SyncJumpWidth|
							    Baudrate_parameters.TimeSeg1|
								 Baudrate_parameters.TimeSeg2|
								 (Controller0_Prescalers[Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerBaudRateConfigID]-1);
			  Recieve_objects_config(Config);
			  CanUnitState=CAN_READY;
			  Can_ControllerMode[zero]=CAN_CS_STOPPED;
		  }
#endif

#if ((!CanController0Activation)&&(CanController1Activation))
	 if  (Can_ControllerMode [zero]== CAN_CS_UNINIT)
		    {
		      CAN1_MSR_R=WKUI_Clear;
		      CAN1_MCR_R=Controller1_STOP_MODE;/*This value passed to the register puts the can controller into initialisation modes and sets
													the required bits for AUTOSAR compliant functionality as:
																							  1-no hardware recovery in case of bus off.
																							  2-[SWS_Can_00012]If the CAN hardware can be configured
																							  to lock the hardware object after reception to protect
																							  data received from being overwritten before CPU reads it.
													and fulfilling requirements of user's configuration as:
																							  1-Multiplexed Transmission
																							  2-TTCAN
												   */
		          while ((CAN1_MSR_R & CAN_MSR_INAK) == zero){}
				 Baudrate_parameters.SyncJumpWidth=(Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerSyncJumpWidth-1)<<Hwreg_SJW_Shift;

				 Baudrate_parameters.TimeSeg1=((Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerPropSeg+
													Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerSeg1)-1)<<Hwreg_Bitseg1_Shift;

				 Baudrate_parameters.TimeSeg2=(Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerSeg2-1)<<Hwreg_Bitseg2_Shift;

				  CAN1_BTR_R= Baudrate_parameters.SyncJumpWidth|
							 Baudrate_parameters.TimeSeg1|
							 Baudrate_parameters.TimeSeg2|
							 (Controller0_Prescalers[Config->CanConfigSet.CanController[zero].CanControllerDefaultBaudrate->CanControllerBaudRateConfigID]-1);
				  Recieve_objects_config(Config);
		          CanUnitState=CAN_READY;
		          Can_ControllerMode[zero]=CAN_CS_STOPPED;
		    }
#endif
	 else{
#if CanDevErrorDetect
		                /*Det_ReportError( CAN_MODULE_ID, CAN_INSTANCE_ID, CAN_INIT_API_ID, CAN_E_TRANSITION);*/

#endif
	      }
}
 else
     {
#if CanDevErrorDetect
	                     /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, CAN_INIT_API_ID, CAN_E_TRANSITION)*/
#endif

      }
#endif
}
/*Service name:Can_DeInit
Syntax:void Can_DeInit(void)
Service ID[hex]:0x10
Sync/Async:Synchronous
Reentrancy:Non Reentrant
Parameters (in):None
None Parameters (in out):None
None Parameters (out):None
None Return value:
None Description:This function de-initialises the module.*/
void Can_DeInit(void)
{
uint8_t index4;
/*   The function Can_DeInit shall raise the error CAN_E_TRANSITION if the driver is not
	in state CAN_READY [SWS_Can_91011]*/
	 if(CanUnitState ==CAN_UNINIT)
	 {
#if CanDevErrorDetect
		 Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_DeInit_Id, CAN_E_TRANSITION);
#endif
	 }
	 else
	 {
#if ((CanController0Activation)&&(CanController1Activation))
		  /*[SWS_Can_91012] If development error detection for the Can module is enabled: The function Can_DeInit shall
		    raise the error CAN_E_TRANSITION if any of the CAN controllers is in state STARTED.*/

	 if  ((Can_ControllerMode [zero]== CAN_CS_STARTED) | ( Can_ControllerMode[one]== CAN_CS_STARTED))
	       {
#else
		if(Can_ControllerMode [zero]== CAN_CS_STARTED)
			{
#endif
#if CanDevErrorDetect
			/*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_DeInit_Id,CAN_E_TRANSITION)*/
#endif

	       }
		else{

#if (CanController0Activation)
			      CAN0_MCR_R=CAN_MCR_RESET;/*Resetting hardware*/
  	              CAN0_MCR_R&=~one;/*to clear error status*/
  	              CAN0_MCR_R=CAN_MCR_RESET;/*Resetting hardware*/
  	              CAN0_IER_R=zero;
	    	      CAN0_MSR_R=WKUI_Clear;/*Resetting wake-up  status*/
	    	      CAN0_TSR_R=MailBox0_INT_CLEAR_FLAG|MailBox1_INT_CLEAR_FLAG|MailBox2_INT_CLEAR_FLAG;/*Resetting TX status*/
	    	      /*Resetting RX status*/
	    	      CAN0_RF0R_R=CAN_RF1R_FOVR1;
	    	      CAN0_RF1R_R=CAN_RF1R_FOVR1;
	    	      /*Resetting all global variables*/
	    	      IntDisableCount[Controller0_ID]=zero;
				  state_transition_flag[Controller0_ID]=zero;
				  WakeUp_Flag[Controller0_ID]=zero;
				  Can_ControllerMode [Controller0_ID]= CAN_CS_UNINIT;
                for (index4=0;index4<Num_of_TX0_MailBoxes;index4++)
					{
					  Controller0_MailBoxes_semaphore[index4]=zero;
					}

#endif
#if (CanController1Activation)
                  CAN1_MCR_R=CAN_MCR_RESET;/*Resetting hardware*/
                  CAN1_MCR_R&=~one;/*to clear error status*/
                  CAN1_MCR_R= CAN_MCR_RESET;/*Resetting hardware*/
                  CAN1_IER_R=zero;
                  CAN1_MSR_R=WKUI_Clear;
                  CAN1_TSR_R=MailBox0_INT_CLEAR_FLAG|MailBox1_INT_CLEAR_FLAG|MailBox2_INT_CLEAR_FLAG;
                  /*Resetting RX status*/
				  CAN1_RF0R_R=CAN_RF1R_FOVR1;
				  CAN1_RF1R_R=CAN_RF1R_FOVR1;
	    	      IntDisableCount[Controller1_ID]=zero;
				  state_transition_flag[Controller1_ID]=zero;
				  WakeUp_Flag[Controller1_ID]=zero;
				  Can_ControllerMode [Controller1_ID]= CAN_CS_UNINIT;
				     for (index4=0;index4>Num_of_TX0_MailBoxes;index4++)
						{
						  Controller1_MailBoxes_semaphore[index4]=zero;
						}
#endif
	    	      CanUnitState =CAN_UNINIT;
		}
	 }

}
/*Service name:Can_SetBaudrate
Syntax:Std_ReturnType Can_SetBaudrate( uint8 Controller, uint16 BaudRateConfigID )
Service ID[hex]:0x0f
Sync/Async:Synchronous
Reentrancy:Reentrant
Parameters (in):	 Controller
				     BaudRateConfigID
Parameters (in out):None
Parameters (in out):None
Parameters (out):None
Return value:Std_ReturnType
	 E_OK: Service request accepted, setting of (new) baud rate started
	 E_NOT_OK: Service request not accepted
Description: This service shall set the baud rate configuration of the CAN controller.
             Depending on necessary baud rate modifications the controller might have to reset.*/


Std_ReturnType Can_SetBaudrate(uint8 Controller, uint16 BaudRateConfigID)
{
	uint8_t return_value3;
	Register_Baudrate_paramters Baudrate_parameters0;
	/* [SWS_CAN_00492] If development error detection for the Can module is enabled
	   The function Can_SetBaudrate shall raise the error CAN_E_UNINIT and return
	   E_NOT_OK if the driver is not yet initialised.*/
	if(CanUnitState == CAN_READY)
	 {
#if CanDevErrorDetect

			/*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, CAN_SETBAUDRATE_API_ID,CAN_E_UNINIT);*/

#endif
		return_value3=E_NOT_OK;
	 }else
	 {

		if(Controller >=NUM_OF_ACTIVE_CAN_CONTROLLERS )
	    {
#if CanDevErrorDetect
			/*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, CAN_SETBAUDRATE_API_ID,CAN_E_PARAM_CONTROLLER);*/
#endif
			return_value3=E_NOT_OK ;
	    }
		else{
#if ((CanController0Activation)&&(CanController1Activation))
              if(Controller==zero)
              {
#endif
#if (CanController0Activation)
                 if(BaudRateConfigID>= Controller0_num_of_baudrates)
                 {
                	 /* [SWS_CAN_00493] If development error detection for the Can module is enabled:
						The function Can_SetBaudrate shall raise the error CAN_E_PARAM_BAUDRATE
						and return E_NOT_OK if the parameter BaudRateConfigID has an invalid value*/
#if CanDevErrorDetect
                	 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, CAN_SETBAUDRATE_API_ID,CAN_E_PARAM_BAUDRATE);*/
#endif
         			return_value3=E_NOT_OK ;
                 }

                 else
                 {
                	 Baudrate_parameters0.SyncJumpWidth=(CanController0_BaudrateConfig[BaudRateConfigID].CanControllerSyncJumpWidth-1)<<Hwreg_SJW_Shift;
					 Baudrate_parameters0.TimeSeg1=((CanController0_BaudrateConfig[BaudRateConfigID].CanControllerPropSeg+
							                         CanController0_BaudrateConfig[BaudRateConfigID].CanControllerSeg1)-1)<<Hwreg_Bitseg1_Shift;
					 Baudrate_parameters0.TimeSeg2=(CanController0_BaudrateConfig[BaudRateConfigID].CanControllerSeg2-1)<<Hwreg_Bitseg2_Shift;

					  CAN0_BTR_R= Baudrate_parameters0.SyncJumpWidth|
								  Baudrate_parameters0.TimeSeg1|
								  Baudrate_parameters0.TimeSeg2|
								 (Controller0_Prescalers[BaudRateConfigID]-1);
					             return_value3=E_OK ;
                 }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
              }
             else{
#endif
#if (CanController1Activation)
            	   if(BaudRateConfigID>= Controller1_num_of_baudrates)
					  {
            		   /* [SWS_CAN_00493] If development error detection for the Can module is enabled:
            		   	 The function Can_SetBaudrate shall raise the error CAN_E_PARAM_BAUDRATE
            		   	 and return E_NOT_OK if the parameter BaudRateConfigID has an invalid value*/
#if CanDevErrorDetect
                	 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, CAN_SETBAUDRATE_API_ID,CAN_E_PARAM_BAUDRATE);*/
#endif
            		   return_value3=E_OK ;
					  }
					  else
					  {
						 Baudrate_parameters0.SyncJumpWidth=(CanController1_BaudrateConfig[BaudRateConfigID].CanControllerSyncJumpWidth-1)<<Hwreg_SJW_Shift;
						 Baudrate_parameters0.TimeSeg1=((CanController1_BaudrateConfig[BaudRateConfigID].CanControllerPropSeg+
														 CanController1_BaudrateConfig[BaudRateConfigID].CanControllerSeg1)-1)<<Hwreg_Bitseg1_Shift;
						 Baudrate_parameters0.TimeSeg2=(CanController1_BaudrateConfig[BaudRateConfigID].CanControllerSeg2-1)<<Hwreg_Bitseg2_Shift;

						  CAN1_BTR_R= Baudrate_parameters0.SyncJumpWidth|
									  Baudrate_parameters0.TimeSeg1|
									  Baudrate_parameters0.TimeSeg2|
									  (Controller1_Prescalers[BaudRateConfigID]-1);
						    return_value3=E_OK ;
					  }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
                  }
#endif
	        }
	 }
	return  return_value3;
}
/* Service name:Can_DisableControllerInterrupts
 Syntax:void Can_DisableControllerInterrupts( uint8 Controller )
 Service ID[hex]: 0x04
 Sync/Async: Synchronous
 Reentrancy:Reentrant
 Parameters (in):Controller
 Parameters (in out):None
 Parameters (out):None
 Return value:None
 Description:This function disables all interrupts for this CAN controller.*/
void Can_DisableControllerInterrupts(uint8_t Controller)
{
	/*[SWS_Can_00049] The function Can_DisableControllerInterrupts shall access the CAN controller registers to disable all interrupts
	  for that CAN controller only, if interrupts for that CAN Controller are enabled.*/
if(CanUnitState==CAN_UNINIT)
{
	/*[SWS_Can_00210] The function Can_EnableControllerInterrupts shall raise the error
	   CAN_E_PARAM_CONTROLLER if the parameter Controller is out of range*/
#if CanDevErrorDetect

	 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_DisableControllerInterrupts_Id,CAN_E_UNINIT);*/
#endif
}
else{
	 /*[SWS_Can_00206] ⌈If development error detection for the Can module is enabled: The function
	   Can_DisableControllerInterrupts shall raise the error CAN_E_PARAM_CONTROLLER if the parameter
	   Controller is out of range.*/
	 if(Controller >= NUM_OF_ACTIVE_CAN_CONTROLLERS )
	 {
#if CanDevErrorDetect
		 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_DisableControllerInterrupts_Id,CAN_E_PARAM_CONTROLLER);*/
#endif
	 }
	 else{
		 IntDisableCount[Controller]++;
//			 if(INT_Enable_flag[Controller])
//			 {
#if ((CanController0Activation)&&(CanController1Activation))
              if(Controller==Controller0_ID)
              {
#endif
#if (CanController0Activation)
            	  CAN0_IER_R=zero;
#endif
#if ((CanController0Activation)&&(CanController1Activation))
              }
			 else
				 {
#endif
#if (CanController1Activation)
				   CAN1_IER_R=zero;
#endif
#if ((CanController0Activation)&&(CanController1Activation))
				 }
#endif
//		  }else{
//			     /*MISRA*/
//		       }
}
}
}
/*Service name:Can_EnableControllerInterrupts
Syntax:void Can_EnableControllerInterrupts( uint8 Controller )
Service ID[hex]:0x05
Sync/Async:Synchronous
Reentrancy:Reentrant
Parameters (in):Controller
Parameters (in out):None
Parameters (out):None
Return value:None
Description:This function enables all allowed interrupts.
*/
void Can_EnableControllerInterrupts(uint8_t Controller)
{
if(CanUnitState==CAN_UNINIT)
{
		/*[SWS_Can_00210] The function Can_EnableControllerInterrupts shall raise the error
		   CAN_E_PARAM_CONTROLLER if the parameter Controller is out of range*/
#if CanDevErrorDetect

		 /* Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_EnableControllerInterrupts_Id,CAN_E_UNINIT);*/
#endif
	}
else{
		 /*[SWS_Can_00206] ⌈If development error detection for the Can module is enabled: The function
		   Can_DisableControllerInterrupts shall raise the error CAN_E_PARAM_CONTROLLER if the parameter
		   Controller is out of range.*/
	 if(Controller >= NUM_OF_ACTIVE_CAN_CONTROLLERS )
	 {
#if CanDevErrorDetect
			   /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_EnableControllerInterrupts_Id,CAN_E_PARAM_CONTROLLER);*/
#endif
	 }
	    else
		 {
				 if(Controller >= NUM_OF_ACTIVE_CAN_CONTROLLERS)
					 {
				#if CanDevErrorDetect
						 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_DisableControllerInterrupts_Id,CAN_E_PARAM_CONTROLLER);*/
				#endif
					 }
				 else{
						IntDisableCount[Controller]--;
						 if(IntDisableCount[Controller]==zero)
						 {
#if ((CanController0Activation)&&(CanController1Activation))
						  if(Controller==Controller0_ID)
						  {
#endif
#if (CanController0Activation)
								switch(Can_ControllerMode[Controller])
								{
								  case CAN_CS_STARTED  :
									  CAN0_IER_R=CONTROLLER0_STARTMODE_INT_ENABLE;
									  break;


								   case CAN_CS_SLEEP:
									   CAN0_IER_R=CONTROLLER0_SLEEPMODE_INT_ENABLE;
									   break;
								}

#endif
#if ((CanController0Activation)&&(CanController1Activation))
						  }
						 else
							 {
#endif
#if (CanController1Activation)
								switch(Can_ControllerMode[Controller])
									{
									  case CAN_CS_STARTED  :
										  CAN1_IER_R=CONTROLLER1_STARTMODE_INT_ENABLE;
										  break;
                                     /*No interrupts to be enabled in stopped mode*/
									   case CAN_CS_SLEEP :
										  CAN1_IER_R=CONTROLLER0_SLEEPMODE_INT_ENABLE;
										   break;
									}
#endif
#if ((CanController0Activation)&&(CanController1Activation))
				      }
#endif
						  }else{
							     /*MISRA*/
						       }
			 }
			 /*[SWS_Can_00208] ⌈The function Can_EnableControllerInterrupts shall
			 perform no action when Can_DisableControllerInterrupts has not been
			 called before.*/
		    }
}
}
/* Service Name  :Can_SetControllerMode
Syntax           : Std_ReturnType Can_SetControllerMode( uint8 Controller, Can_ControllerStateType Transition )
Service ID[hex]  :0x03
Sync/Async       : Asynchronous
Reentrancy       : Non Reentrant
Parameters (in)  :Controller:CAN controller for which the status shall be changed,
                  Transition:Transition value to request new CAN controller state
Parameters(in out): None
Parameters (out) : None
Return Value     : E_OK: request accepted ,E_NOT_OK:request not accepted, a development error occurred
DESCRIPTION      : This function performs software triggered state transitions of the CAN controller State machine.*/
Std_ReturnType Can_SetControllerMode(uint8_t Controller,Can_ControllerStateType Transition)
{

	/*[SWS_Can_00017] ⌈The function Can_SetControllerMode shall perform software triggered state
	transitions of the CAN controller State machine.*/
	uint8_t return_value;
	/*[SWS_Can_00198]If development error detection for the Can module is enabled: if the module is not yet initialised,
	the function Can_SetControllerMode shall raise development error CAN_E_UNINIT and return E_NOT_OK.*/
if(CanUnitState == CAN_UNINIT)
{
#if CanDevErrorDetect
	 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, CAN_SET_CONTROLLER_MODE, CAN_E_UNINIT)*/

#endif
		return_value=E_NOT_OK;
}
else{
	/*[SWS_Can_00199]If development error detection for the Can module is enabled: if the parameter Controller is out of range,
	the function Can_SetControllerMode shall raise development error CAN_E_PARAM_CONTROLLER and return E_NOT_OK.*/
	 if(Controller>= NUM_OF_ACTIVE_CAN_CONTROLLERS)
	{
#if CanDevErrorDetect
	/*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, CAN_SET_CONTROLLER_MODE, CAN_E_PARAM_CONTROLLER);*/

#endif
		return_value=E_NOT_OK;
	}
 else
    {
	 /*[SWS_Can_00200]If development error detection for the Can module is enabled: if an invalid transition has been requested,
	 the function Can_SetControllerMode shall raise the error CAN_E_TRANSITION and return E_NOT_OK.*/
     if(((Transition == CAN_CS_STARTED) && ( Can_ControllerMode [Controller]!=CAN_CS_STOPPED))||
	  ((Transition ==  CAN_CS_SLEEP ) && (Can_ControllerMode [Controller]!= CAN_CS_STOPPED)))
	 {
#if CanDevErrorDetect
    	 /*Det_ReportError( CAN_MODULE_ID ,CAN_INSTANCE_ID ,CAN_SET_CONTROLLER_MODE ,CAN_E_TRANSITION );*/

#endif
    	 return_value=E_NOT_OK;
	 }
    else
     {
       return_value=E_OK;

       Requested_Mode_Transition [Controller]=Transition;
       state_transition_flag[Controller]=one;


	  switch(Transition)
	  {
	  /*[SWS_Can_00384] ⌈Each time the CAN controller state machine is triggered with the state transition value CAN_CS_STARTED,
	  the function Can_SetControllerMode shall re-initialise theCAN controller with the same controller configuration set
	  previously used by functions Can_SetBaudrate or Can_Init.⌋()*/
         case  CAN_CS_STARTED :
//        	 INT_Enable_flag[Controller]=one;
        	   /* [SWS_Can_00261] The function Can_SetControllerMode(CAN_CS_STARTED) shall set
				  the hardware registers in a way that makes the CAN controller participating
				  on the network.*/
#if ((CanController0Activation)&&(CanController1Activation))
        	  switch(Controller)
        	  {
        	   case zero :
#endif
        		   /*[SWS_Can_00425] ⌈Enabling of CAN interrupts shall not be executed, when CAN interrupts
				     have been disabled by function Can_DisableControllerInterrupts*/
					       switch(IntDisableCount[zero])/*check weather interrupts were disabled by Can_DisableControllerInterrupts API*/
					       {
#if (CanController0Activation)
					          case zero:
					       /*[SWS_Can_00196] The function Can_SetControllerMode shall enable interrupts that are needed in the new state.*/
					       /*[SWS_Can_00197] The function Can_SetControllerMode shall disable interrupts that are not allowed in the new state.*/
					        	         CAN0_IER_R=CONTROLLER0_STARTMODE_INT_ENABLE;/*enabling interrupts required for this state*/
					        	         CAN0_MCR_R= Controller0_START_MODE;/*Value responsible for putting controller in started mode*/
					        	         Controller0_StartTransition_Poll();
					        	         break;
					          default :/*switch to start mode without enabling interrupts*/
					        	          CAN0_MCR_R=Controller0_START_MODE;/*Value responsible for putting controller in started mode*/
					        	          Controller0_StartTransition_Poll();
					        	          break;
					       }
					       /* Wait the acknowledge */
#endif
#if ((CanController0Activation)&&(CanController1Activation))
					            break;
			  case one :
				           switch(IntDisableCount[one])/*check weather interrupts were disabled by Can_DisableControllerInterrupts API*/
				           {
#endif
#if (CanController1Activation)
							  case zero:
										 CAN1_IER_R=CONTROLLER1_STARTMODE_INT_ENABLE;/*enabling interrupts required for this state*/
										 CAN1_MCR_R=Controller1_START_MODE;/*Value responsible for putting controller in started mode*/
										 Controller1_StartTransition_Poll();
										 break;
							  default :/*switch to start mode without enabling interrupts*/
								         CAN1_MCR_R=Controller1_START_MODE;/*Value responsible for putting controller in started mode*/
								         Controller1_StartTransition_Poll();
								         break;
						   }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
						         break;
        	  }
#endif
        	  break;/*getting out of start case*/

	     case   CAN_CS_STOPPED :
//	    	 INT_Enable_flag[Controller]=zero;
	    	   /*[SWS_Can_00263] The function Can_SetControllerMode(CAN_CS_STOPPED) shall set the
	    	     bits inside the CAN hardware such that the CAN controller stops participating on the
	    	     network.*/
#if ((CanController0Activation)&&(CanController1Activation))
			  switch(Controller)
			  {

			   case zero :
#endif
						   /*[SWS_Can_00426] ⌈Disabling of CAN interrupts shall not be executed, when CAN interrupts
						   have been disabled by function Can_DisableControllerInterrupts.⌋*/
						   switch(IntDisableCount[zero])/*check weather interrupts were disabled by Can_DisableControllerInterrupts API*/
						   {/*no interrupts are required in stopped mode*/
#if (CanController0Activation)
							  case zero:
					  /*[SWS_Can_00197] The function Can_SetControllerMode shall disable interrupts that are not allowed in the new state.*/
										 CAN0_IER_R=zero;/*disabling interrupts not required for this state*/
										 CAN0_MCR_R=Controller0_STOP_MODE;/*Value responsible for putting controller in stopped mode*/
										 CAN0_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2); /*[SWS_Can_00282]ˆThe function Can_SetControllerMode(CAN_CS_STOPPED)
                                                                                                      shall cancel pending messages.*/
										 Controller0_StopTransition_Poll();


										 break;
							  default :/*disabling interrupts is not required*/
										 CAN0_MCR_R=Controller0_STOP_MODE ;/*Value responsible for putting controller in stopped mode*/
										 CAN0_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2);/*Cancelling all pending messages*/
										 Controller0_StopTransition_Poll();
										 break;
						   }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
								break;
			   case one :
						   switch(IntDisableCount[one])/*check weather interrupts were disabled by Can_DisableControllerInterrupts API*/
						   {
#endif
#if (CanController1Activation)
							  case zero:
										 CAN1_IER_R=zero;/*disabling interrupts not allowed for this state*/
										 CAN1_MCR_R=Controller1_STOP_MODE ;/*Value responsible for putting controller in stopped mode*/
										 /*[SWS_Can_00282] The function Can_SetControllerMode(CAN_CS_STOPPED)
										   shall cancel pending messages.*/
										 CAN1_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2);
										 Controller1_StopTransition_Poll();
										 break;
							  default :/*switch to start mode without enabling interrupts*/
										 CAN1_MCR_R=Controller1_STOP_MODE ;/*Value responsible for putting controller in stopped mode*/


										 Controller1_StopTransition_Poll();
										 break;
						   }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
								 break;
			     }
#endif

							 break;/*getting out of stop case*/



		  case  CAN_CS_SLEEP  :
//			  INT_Enable_flag[Controller]=one;
			  /*[SWS_Can_00265]The function Can_SetControllerMode(CAN_CS_SLEEP) shall set the controller into sleep mode.*/
#if ((CanController0Activation)&&(CanController1Activation))
				  switch(Controller)
				  {

				   case zero :
#endif
							   /*[SWS_Can_00426] Disabling of CAN interrupts shall not be executed, when CAN interrupts
							   have been disabled by function Can_DisableControllerInterrupts.*/
					   switch(IntDisableCount[zero])/*check weather interrupts were disabled by Can_DisableControllerInterrupts API*/
					   {
#if (CanController0Activation)
						  case zero:
				     /*[SWS_Can_00196] ⌈The function Can_SetControllerMode shall enable interrupts that are needed in the new state.*/
					/*[SWS_Can_00197] ⌈The function Can_SetControllerMode shall disable interrupts that are not allowed in the new state.*/
									 CAN0_IER_R=CONTROLLER0_SLEEPMODE_INT_ENABLE;/*Enabling interrupts needed for this state*/
									 CAN0_MCR_R=Controller0_SLEEP_MODE;/*Value responsible for putting controller in sleep mode*/
									 Controller0_SleepTransition_Poll();
									 break;
						  default :/*switch to sleep mode without enabling interrupts*/
									 CAN0_MCR_R=Controller0_SLEEP_MODE;/*Value responsible for putting controller in sleep mode*/
									 Controller0_SleepTransition_Poll();
									 break;
					   }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
					   break;
		   case one :
					   switch(IntDisableCount[one])/*check weather interrupts were disabled by Can_DisableControllerInterrupts API*/
					   {
#endif
#if (CanController1Activation)
						  case zero:
									 CAN1_IER_R=CONTROLLER1_SLEEPMODE_INT_ENABLE;/*Enabling interrupts needed for this state*/
									 CAN1_MCR_R=Controller1_SLEEP_MODE;/*Value responsible for putting controller in sleep mode*/
									 Controller1_SleepTransition_Poll();
									 break;
						  default :/*switch to start mode without enabling interrupts*/
									 CAN1_MCR_R=Controller1_SLEEP_MODE;/*Value responsible for putting controller in sleep mode*/
									 Controller1_SleepTransition_Poll();
									 break;
					   }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
							 break;
			 }
#endif
						 break;/*getting out of sleep case*/
	  }
     }
}
}
return return_value;
}
/*Service name:Can_GetControllerMode
Syntax:Std_ReturnType Can_GetControllerMode( uint8 Controller, Can_ControllerStateType* ControllerModePtr )
Service ID[hex]:0x12
Sync/Async:Synchronous
Reentrancy:Non Reentrant
Parameters (in):Controller
Parameters (in out):None
Parameters (out):ControllerModePtr
Return value:E_OK: Controller mode request has been accepted.
             E_NOT_OK: Controller mode request has not been*/
Std_ReturnType Can_GetControllerMode(uint8_t Controller, Can_ControllerStateType* ControllerModePtr)
{
	uint8_t return_value1;
    if(Controller >= NUM_OF_ACTIVE_CAN_CONTROLLERS)
    {
    	/*[SWS_Can_91017] ⌈If parameter Controller of Can_GetControllerMode() has an invalid value,
    	the CanDrv shall report development error code CAN_E_PARAM_CONTROLLER to the Det_ReportError service of the DET.*/
#if CanDevErrorDetect
    	 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_GetControllerErrorState_Id, CAN_E_UNINIT);*/
#endif
    	   return_value1=E_NOT_OK;
    }
    else{
         if(ControllerModePtr==NULL)
         {
        	/* [SWS_Can_91018]If parameter ControllerModePtr of Can_GetControllerMode() has an null pointer,
        	 the CanDrv shall report development error code CAN_E_PARAM_POINTER to the Det_ReportError service of the DET.*/
#if CanDevErrorDetect
        	 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID,Can_GetControllerMode_Id, CAN_E_PARAM_CONTROLLER);*/
#endif
        	 return_value1=E_NOT_OK;
         }
         else{
                if(CanUnitState==CAN_UNINIT)
                {
#if CanDevErrorDetect
	        Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID,Can_GetControllerMode_Id, CAN_E_PARAM_POINTER);
#endif
                	   return_value1=E_NOT_OK;
                        /*[SWS_Can_91016] If development error detection for the Can module is enabled:
                        The function Can_GetControllerMode shall raise the error CAN_E_UNINIT and return
                        E_NOT_OK if the driver is not yet initialised.*/
                    }
                   else{
                	   /*[SWS_Can_91015] The service Can_GetControllerMode shall return
                	     the mode of the requested CAN controller.*/
                	         *ControllerModePtr=Can_ControllerMode [Controller];
                	         return_value1=E_OK;
                       }
       }
    }
    return return_value1;
}
/*Service name:Can_GetControllerErrorState
Syntax:Std_ReturnType Can_GetControllerErrorState( uint8 ControllerId, Can_ErrorStateType* ErrorStatePtr )
Service ID[hex]:0x11
Sync/Async:Synchronous
Reentrancy:Non Reentrant for the same ControllerId
Parameters (in):ControllerId
Parameters (in out):None
Parameters (out):ErrorStatePtr
Return value:Std_ReturnType
E_OK: Error state request has been accepted. E_NOT_OK: Error state request has not been accepted.
Description:This service obtains the error state of the CAN controller.*/
Std_ReturnType Can_GetControllerErrorState(uint8_t ControllerId, Can_ErrorStateType* ErrorStatePtr)
{
	uint8_t error_status;
	uint8_t return_value2;

	  /*[SWS_Can_91005]  If development error detection for the Can module is enabled: if the module is not yet initialised,
	    the function Can_GetControllerErrorState shall raise development error CAN_E_UNINIT and return E_NOT_OK*/
	if(CanUnitState==CAN_UNINIT)
	 {
#if CanDevErrorDetect
		 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_GetControllerErrorState_Id, CAN_E_UNINIT);*/
#endif
	     return_value2=E_NOT_OK;
	 }
	 else{
		   /*
		    [SWS_Can_91006] If development error detection for the Can module is enabled: if the parameter ControllerId is out of range,
		    the function Can_GetControllerErrorState shall raise development error CAN_E_PARAM_CONTROLLER and return E_NOT_OK.*/
	         if(ControllerId >=NUM_OF_ACTIVE_CAN_CONTROLLERS)
	         {
#if CanDevErrorDetect
		         /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_GetControllerErrorState_Id, CAN_E_PARAM_CONTROLLER);*/
#endif
	               return_value2=E_NOT_OK;
	         }
	             else{
	            	   /* [SWS_Can_91007] If development error detection for the Can module is enabled: if the parameter ErrorStatePtr is a null pointer,
	            	      the function Can_GetControllerErrorState shall raise development error  CAN_E_PARAM_POINTER and return E_NOT_OK.*/
	                   if(ErrorStatePtr==NULL)
	                   {
#if CanDevErrorDetect
		 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_GetControllerErrorState_Id,);*/
#endif
	                	   return_value2=E_NOT_OK;
	                   }
	                   else{
	                	   return_value2=E_OK;
#if ((CanController0Activation)&&(CanController1Activation))
	                	       if(ControllerId==Controller0_ID)
	                	       {
#endif
#if (CanController0Activation)
	                	    	   error_status=CAN0_ESR_R&ERROR_MASK;
                                    switch(error_status)
                                    {
                                     case EWARN:
                                    	 *ErrorStatePtr=CAN_ERRORSTATE_ACTIVE;
                                    	 break;
                                     case EPASS:
                                    	 *ErrorStatePtr=CAN_ERRORSTATE_PASSIVE;
                                    	 break;
                                     case BOFF:
                                    	 *ErrorStatePtr=CAN_ERRORSTATE_BUSOFF;
                                    	 break;
                                    }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
	                	       }else
	                	         {
#endif

#if (CanController1Activation)
	                	    	   error_status=CAN1_ESR_R&ERROR_MASK;
	                	    	   switch(error_status)
									   {
										case EWARN:
											*ErrorStatePtr=CAN_ERRORSTATE_ACTIVE;
										 break;
										case EPASS:
											*ErrorStatePtr=CAN_ERRORSTATE_PASSIVE;
										 break;
										case BOFF:
											*ErrorStatePtr=CAN_ERRORSTATE_BUSOFF;
										 break;
									   }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
	                	         }
#endif
	                       }
 }
}
    return return_value2;
}
/*Service name:Can_Write '
Syntax:Std_ReturnType Can_Write( Can_HwHandleType Hth, const Can_PduType* PduInfo )
Service ID[hex]:0x06
Sync/Async:Synchronous
Reentrancy:Reentrant (thread-safe)
Parameters (in):Hth :information which HW-transmit handle shall be used for transmit. Implicitly this is also the information about
                the controller to use because the Hth numbers are unique inside one hardware unit.
                ,PduInfo:Pointer to SDU user memory, Data Length and Identifier.
Parameters (in out):None
Parameters (out):None
Return value:Std_ReturnType: E_OK: Write command has been accepted
                             E_NOT_OK: development error occurred
             Can_ReturnType: CAN_BUSY: No TX hardware buffer available or pre-emptive call of Can_Write that can't be implemented re-entrant.
Description:This function is called by CanIf to pass a CAN message to CanDrv for transmission.*/

Std_ReturnType Can_Write(Can_HwHandleType Hth,const Can_PduType* PduInfo)
{
CAN_TypeDef *can_ip;
uint8_t Mailbox_index;
uint8_t j;
uint8_t return_value0;

	 /*[SWS_Can_00216] If development error detection for the Can module is enabled: The functionCan_Write shall raise
	    the error CAN_E_UNINIT and shall return E_NOT_OK if the driver is not yet initia_lized.*/
 if(CanUnitState==CAN_UNINIT)
 {
#if CanDevErrorDetect
	  /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_Write_Id, CAN_E_UNINIT);*/
#endif
	 return_value0= E_NOT_OK;
 }
 else
 {
	 /*[SWS_CAN_00219] If development error detection for CanDrv is enabled: Can_Write()
	   shall raise CAN_E_PARAM_POINTER and shall return E_NOT_OK if the parameter PduInfo
	   is a null pointer.*/
	 if(PduInfo==NULL)
	 {
#if CanDevErrorDetect
	  /* Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_Write_Id, CAN_E_PARAM_POINTER);*/
#endif
		 return_value0= E_NOT_OK;
	  }
	  else
		{
		      /*[SWS_CAN_00505] If development error detection for CanDrv is enabled: Can_Write()
			  shall raise CAN_E_PARAM_POINTER and shall return E_NOT_OK if the trigger transmit
			  API is disabled for this hardware object (CanTriggertransmitEnable = FALSE) and
			  the SDU pointer inside PduInfo is a null pointer.*/

		  if(PduInfo->sdu==NULL)
		   {
			 if(CanContainer.CanConfigSet.CanHardwareObject[Hth].CanTriggerTransmitEnable==FALSE)
			 {
#if CanDevErrorDetect
				 /*	Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_Write_Id, CAN_E_PARAM_DATA_LENGTH)*/
#endif
				 		 return_value0= E_NOT_OK;
			 }
			else
			{
			   /*SWS_CAN_00504] If the trigger transmit API is enabled for the hardware object,
			   Can_Write() shall interpret a null pointer as SDU (Can_PduType.Can_SduPtrType = NULL)
			   as request for using the trigger transmit interface. If so and the hardware object is
			   free, Can_Write() shall call CanIf_Triggertransmit() with the maximum size
			   of the message buffer to acquire the PDU's data.*/
#if ((CanController0Activation)&&(CanController1Activation))
			 if(CanContainer.CanConfigSet.CanHardwareObject[Hth].CanControllerRef==&CanContainer.CanConfigSet.CanController[zero])
			 {
#endif
#if (CanController0Activation)
				 for(Mailbox_index=HTH_To_MailBoxes_Map[Hth];Mailbox_index<(CanContainer.CanConfigSet.CanHardwareObject[Hth].CanHwObjectCount
				        		 +HTH_To_MailBoxes_Map[Hth]);Mailbox_index++)
				    {
				if(Controller0_MailBoxes_semaphore[Mailbox_index]== confirmed)
				{
				   /*[SWS_CAN_00506] Can_Write() shall return E_NOT_OK if the trigger transmit API
				 	 CanIf_Triggertransmit() returns E_NOT_OK.*/
		    	 /*PduInfoType TTCAN_PduInfo;
				   TTCAN_PduInfo.SduDataptr=NULL;
				   TTCAN_PduInfo.SduLength=Maximum_Data_Size;

						if(CanIf_Triggertransmit(PduInfo->swPduHandle,PduInfoType* PduInfoPtr)==E_NOT_OK.)
						{
						  return_value= E_NOT_OK;
						}
						else
						{
						  return_value= E_OK;
						}*/
				}else{
               	     if(Mailbox_index==CanContainer.CanConfigSet.CanHardwareObject[Hth].CanHwObjectCount+HTH_To_MailBoxes_Map[Hth]-1U)
						 {
							 return_value0=CAN_BUSY;
							 break;
						 }else{}
                     }
			       }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
						 }
			 else
			 {/*for controller 1*/
#endif
#if (CanController1Activation)
			 for(Mailbox_index=HTH_To_MailBoxes_Map[Hth];Mailbox_index<(CanContainer.CanConfigSet.CanHardwareObject[Hth].CanHwObjectCount
							 +HTH_To_MailBoxes_Map[Hth]);Mailbox_index++)
			  {
				 if(Controller1_MailBoxes_semaphore[Mailbox_index]== confirmed)
			     {
					  /*[SWS_CAN_00506] Can_Write() shall return E_NOT_OK if the trigger transmit API
						 CanIf_Triggertransmit() returns E_NOT_OK.*/
					 /*PduInfoType TTCAN_PduInfo;
					   TTCAN_PduInfo.SduDataptr=NULL;
					   TTCAN_PduInfo.SduLength=Maximum_Data_Size;

					if(CanIf_Triggertransmit(PduInfo->swPduHandle,PduInfoType* PduInfoPtr)==E_NOT_OK.)
					{
					  return_value= E_NOT_OK;
					}
					else
					{
					  return_value= E_OK;
					}*/

			      }
				 else{
					 if(Mailbox_index==CanContainer.CanConfigSet.CanHardwareObject[Hth].CanHwObjectCount+HTH_To_MailBoxes_Map[Hth]-1U)
					 {
						 return_value0=CAN_BUSY;
						 break;
					 }else{}
				  }
		        }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
				     }
#endif
			 }
		   }
		  else
		  {
			  /*[SWS_Can_00218] The function Can_Write shall return E_NOT_OK and if development error detection
			  	for the CAN module is enabled shall raise the error CAN_E_PARAM_DATA_LENGTH,If the length is more than 8 byte*/
		   if(PduInfo->length>Maximum_Data_Size)
			{
#if CanDevErrorDetect
				 /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID, Can_Write_Id, CAN_E_PARAM_DATA_LENGTH)*/
#endif
			return_value0= E_NOT_OK;
			}

		   else
		   {
			   /*The Can module shall support multiplexed transmission for devices, which send L-PDUs in order of L-PDU priority.
			    This requirement is full-filled within the Initialisation according to user's configuration*/

#if ((CanController0Activation)&&(CanController1Activation))
			 if(CanContainer.CanConfigSet.CanHardwareObject[Hth].CanControllerRef==&CanContainer.CanConfigSet.CanController[zero])
			 {
#endif
#if (CanController0Activation)
				  /*[SWS_Can_00276] The function Can_Write shall store the swPduHandle that is given inside the parameter
					PduInfo until the Can module calls the CanIf_TxConfirmation for this request where
					the swPduHandle is given as parameter*/
         for(Mailbox_index=HTH_To_MailBoxes_Map[Hth];Mailbox_index<(CanContainer.CanConfigSet.CanHardwareObject[Hth].CanHwObjectCount
        		 +HTH_To_MailBoxes_Map[Hth]);Mailbox_index++)
         {

            if(Controller0_MailBoxes_semaphore[Mailbox_index]== confirmed)
            {
            	Controller0_MailBoxes_semaphore[Mailbox_index]= un_confirmed;

				/*Mailbox_index=HTH_To_MailBoxes_Map[Hth].Busy_Mailboxes_Num+HTH_To_MailBoxes_Map[Hth].Starting_Mailbox;
				 HTH_To_MailBoxes_Map[Hth].Busy_Mailboxes_Num++;*/
            	can_ip=CAN1;
#if CanMultiplexedTransmission
      for(j=Mailbox_index+1;j<=2;j++)
      {
    	 if((can_ip->sTxMailBox[j].TIR&CAN_ID_EXT)==CAN_ID_EXT)
    	 {
    		   if(PduInfo->id==(can_ip->sTxMailBox[j].TIR>>CAN_TI0R_EXID_Pos))
    		    	  {

    			        switch(Mailbox_index)
    			        {
    			          case MailBox0:
    			        	  CAN0_TSR_R|=CAN_TSR_ABRQ1;/*abort previous mailbox*/
    			        	  if ((CAN_TSR_TXOK1 & CAN0_TSR_R) != 0U)
    			        	  {

    			        	  }else{

    			        		  if(Controller0_MailBoxes_semaphore[j]==un_confirmed)
    			        	       {
								    can_ip->sTxMailBox[Mailbox_index].TIR =can_ip->sTxMailBox[j].TIR;
									can_ip->sTxMailBox[Mailbox_index].TDLR=can_ip->sTxMailBox[j].TDLR;
									can_ip->sTxMailBox[Mailbox_index].TDTR=can_ip->sTxMailBox[j].TDTR;
									can_ip->sTxMailBox[Mailbox_index].TDHR=can_ip->sTxMailBox[j].TDHR;
									can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;
									Controller0_MailBox_Pending_PduId[Mailbox_index]=Controller0_MailBox_Pending_PduId[j];
									Mailbox_index+=1;/*MISRA*/
    			        	       }else{

    			        	            }
							      }
							  break;
    			          case MailBox1:
    			        	  CAN0_TSR_R|=CAN_TSR_ABRQ2;/*aborting previous mailbox*/
    			        	  if ((CAN_TSR_TXOK2 & CAN0_TSR_R) != 0U)
								  {

								  }else{

									  if(Controller0_MailBoxes_semaphore[j]==un_confirmed)
							          {
									    can_ip->sTxMailBox[Mailbox_index].TIR =can_ip->sTxMailBox[j].TIR;
										can_ip->sTxMailBox[Mailbox_index].TDLR=can_ip->sTxMailBox[j].TDLR;
										can_ip->sTxMailBox[Mailbox_index].TDTR=can_ip->sTxMailBox[j].TDTR;
										can_ip->sTxMailBox[Mailbox_index].TDHR=can_ip->sTxMailBox[j].TDHR;
										can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;
										Controller0_MailBox_Pending_PduId[Mailbox_index]=Controller0_MailBox_Pending_PduId[j];
										Mailbox_index+=1;
							          }else
										  {
                                              /*MISRA*/
										  }
								      }
    			               break;

    			        }
    		    	  }
    	 }
    	 else{
    		    if(PduInfo->id==(can_ip->sTxMailBox[j].TIR>>CAN_TI0R_STID_Pos))
    		      {    switch(Mailbox_index)
			        {
			          case MailBox0:
			        	  CAN0_TSR_R|=CAN_TSR_ABRQ1;
			        	  if ((CAN_TSR_TXOK1 & CAN0_TSR_R) != 0U)
						  {


						  }else{

							    if(Controller0_MailBoxes_semaphore[j]==un_confirmed)
							    {
								can_ip->sTxMailBox[Mailbox_index].TIR =can_ip->sTxMailBox[j].TIR;
								can_ip->sTxMailBox[Mailbox_index].TDLR=can_ip->sTxMailBox[j].TDLR;
								can_ip->sTxMailBox[Mailbox_index].TDTR=can_ip->sTxMailBox[j].TDTR;
								can_ip->sTxMailBox[Mailbox_index].TDHR=can_ip->sTxMailBox[j].TDHR;
								can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;
								Controller0_MailBox_Pending_PduId[Mailbox_index]=Controller0_MailBox_Pending_PduId[j];
								Mailbox_index+=1;
							    }
							    else{
							    	 /*MISRA*/
							        }
							  }
						  break;
			          case MailBox1:
			        	  CAN0_TSR_R|=CAN_TSR_ABRQ2;
			        	  if ((CAN_TSR_TXOK2 & CAN0_TSR_R) != 0U)
							  {

							  }else{

								  if(Controller0_MailBoxes_semaphore[j]==un_confirmed)
						          {
									can_ip->sTxMailBox[Mailbox_index].TIR =can_ip->sTxMailBox[j].TIR;
									can_ip->sTxMailBox[Mailbox_index].TDLR=can_ip->sTxMailBox[j].TDLR;
									can_ip->sTxMailBox[Mailbox_index].TDTR=can_ip->sTxMailBox[j].TDTR;
									can_ip->sTxMailBox[Mailbox_index].TDHR=can_ip->sTxMailBox[j].TDHR;
									can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;
									Controller0_MailBox_Pending_PduId[Mailbox_index]=Controller0_MailBox_Pending_PduId[j];
									Mailbox_index+=1;
								  }else{

								       }
			               break;
			        }

    		      }
    	     }
    	  }
      }
#endif

				Controller0_MailBox_Pending_PduId[Mailbox_index]=PduInfo->swPduHandle;

			  	can_ip->sTxMailBox[Mailbox_index].TDTR =PduInfo->length;

			      switch(CanContainer.CanConfigSet.CanHardwareObject[Hth].CanIdType)
				   {
			         case STANDARD:
										 can_ip->sTxMailBox[Mailbox_index].TIR = ((PduInfo->id<<CAN_TI0R_STID_Pos));
										 break;

			         case EXTENDED:
			       			        	 can_ip->sTxMailBox[Mailbox_index].TIR = ((PduInfo->id<<CAN_TI0R_EXID_Pos)|CAN_ID_EXT);
			       			        	 break;
			         default:/*MIXED case*/
                              if((PduInfo->id&EXT_MASK)==EXT_MASK)/*Determining if it is standard or extended*/
                              {
                                 /*ID is Extended*/
                            	  can_ip->sTxMailBox[Mailbox_index].TIR = ((PduInfo->id<<CAN_TI0R_EXID_Pos)|CAN_ID_EXT);
                              }
                              else
                              {
                            	  /*ID is Standard*/
                            	  can_ip->sTxMailBox[Mailbox_index].TIR = ((PduInfo->id<<CAN_TI0R_STID_Pos));
                              }
				     }
			                can_ip->sTxMailBox[Mailbox_index].TDLR=
																 ((uint32_t)PduInfo->sdu[3U]<< CAN_TDL0R_DATA3_Pos)|
																 ((uint32_t)PduInfo->sdu[2U]<< CAN_TDL0R_DATA2_Pos)|
																 ((uint32_t)PduInfo->sdu[1U]<< CAN_TDL0R_DATA1_Pos)|
																 ((uint32_t)PduInfo->sdu[0U]<< CAN_TDL0R_DATA0_Pos);
                             if(PduInfo->length>4U)
                           {
						    can_ip->sTxMailBox[Mailbox_index].TDHR=
																((uint32_t)PduInfo->sdu[7U]<< CAN_TDH0R_DATA7_Pos)|
																((uint32_t)PduInfo->sdu[6U]<< CAN_TDH0R_DATA6_Pos)|
																((uint32_t)PduInfo->sdu[5U]<< CAN_TDH0R_DATA5_Pos)|
																((uint32_t)PduInfo->sdu[4U]<< CAN_TDH0R_DATA4_Pos);
                           }
                           else
                           {
                        	    /*MISRA*/
                           }
                              can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;/*initiating request*/
                              return_value0=E_OK;
                              									 break;


                         }else{
                        	     if(Mailbox_index==CanContainer.CanConfigSet.CanHardwareObject[Hth].CanHwObjectCount+HTH_To_MailBoxes_Map[Hth]-1U)
								 {
                        	    	 /*[SWS_Can_00213] The function Can_Write shall perform no actions if the
                        	    						 hardware transmit object is busy with another transmit request.for an L-PDU:
                        	    						 1. The transmission of the other L-PDU shall not be cancelled
                        	    						 and the function Can_Write is left without any actions.
                        	    						 2. The function Can_Write shall return CAN_BUSY.*/
									 return_value0=CAN_BUSY;
									 break;
								 }else{}
                              }
                     }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
						 }



			 else
			 {/*for controller 1*/
#endif

#if (CanController1Activation)
				 for(Mailbox_index=HTH_To_MailBoxes_Map[Hth];Mailbox_index<(CanContainer.CanConfigSet.CanHardwareObject[Hth].CanHwObjectCount
				        		 +HTH_To_MailBoxes_Map[Hth]);Mailbox_index++)
				     {
					 can_ip=CAN2;
						if(Controller1_MailBoxes_semaphore[Mailbox_index]== confirmed)
						{
							Controller1_MailBoxes_semaphore[Mailbox_index]= un_confirmed;
#if CanMultiplexedTransmission
      for(j=Mailbox_index+1;j<2;j++)
      {
    	 if((can_ip->sTxMailBox[j].TIR&CAN_ID_EXT)==CAN_ID_EXT)
    	 {
    		   if(PduInfo->id==(can_ip->sTxMailBox[j].TIR>>CAN_TI0R_EXID_Pos))
    		    	  {

    			        switch(Mailbox_index)
    			        {
    			          case MailBox0:
    			        	  CAN1_TSR_R|=CAN_TSR_ABRQ1;/*abort previous mailbox*/
    			        	  if ((CAN_TSR_TXOK1 & CAN0_TSR_R) != 0U)
    			        	  {

    			        	  }else{

    			        		  if(Controller1_MailBoxes_semaphore[j]==un_confirmed)
    			        	       {
								    can_ip->sTxMailBox[Mailbox_index].TIR =can_ip->sTxMailBox[j].TIR;
									can_ip->sTxMailBox[Mailbox_index].TDLR=can_ip->sTxMailBox[j].TDLR;
									can_ip->sTxMailBox[Mailbox_index].TDTR=can_ip->sTxMailBox[j].TDTR;
									can_ip->sTxMailBox[Mailbox_index].TDHR=can_ip->sTxMailBox[j].TDHR;
									can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;
									Controller1_MailBox_Pending_PduId[Mailbox_index]=Controller0_MailBox_Pending_PduId[j];
									Mailbox_index+=1;/*MISRA*/
    			        	       }else{

    			        	            }
							      }
							  break;
    			          case MailBox1:
    			        	  CAN1_TSR_R|=CAN_TSR_ABRQ2;/*aborting previous mailbox*/
    			        	  if ((CAN_TSR_TXOK2 & CAN0_TSR_R) != 0U)
								  {

								  }else{

									  if(Controller1_MailBoxes_semaphore[j]==un_confirmed)
							          {
									    can_ip->sTxMailBox[Mailbox_index].TIR =can_ip->sTxMailBox[j].TIR;
										can_ip->sTxMailBox[Mailbox_index].TDLR=can_ip->sTxMailBox[j].TDLR;
										can_ip->sTxMailBox[Mailbox_index].TDTR=can_ip->sTxMailBox[j].TDTR;
										can_ip->sTxMailBox[Mailbox_index].TDHR=can_ip->sTxMailBox[j].TDHR;
										can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;
										Controller1_MailBox_Pending_PduId[Mailbox_index]=Controller1_MailBox_Pending_PduId[j];
										Mailbox_index+=1;
							          }else
										  {
                                              /*MISRA*/
										  }
								      }
    			               break;

    			        }
    		    	  }
    	 }
    	 else{
    		    if(PduInfo->id==(can_ip->sTxMailBox[j].TIR>>CAN_TI0R_STID_Pos))
    		      {    switch(Mailbox_index)
			        {
			          case MailBox0:
			        	  CAN1_TSR_R|=CAN_TSR_ABRQ1;
			        	  if ((CAN_TSR_TXOK1 & CAN1_TSR_R) != 0U)
						  {


						  }else{

							    if(Controller1_MailBoxes_semaphore[j]==un_confirmed)
							    {
								can_ip->sTxMailBox[Mailbox_index].TIR =can_ip->sTxMailBox[j].TIR;
								can_ip->sTxMailBox[Mailbox_index].TDLR=can_ip->sTxMailBox[j].TDLR;
								can_ip->sTxMailBox[Mailbox_index].TDTR=can_ip->sTxMailBox[j].TDTR;
								can_ip->sTxMailBox[Mailbox_index].TDHR=can_ip->sTxMailBox[j].TDHR;
								can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;
								Controller1_MailBox_Pending_PduId[Mailbox_index]=Controller1_MailBox_Pending_PduId[j];
								Mailbox_index+=1;
							    }
							    else{
							    	 /*MISRA*/
							        }
							  }
						  break;
			          case MailBox1:
			        	  CAN1_TSR_R|=CAN_TSR_ABRQ2;
			        	  if ((CAN_TSR_TXOK2 & CAN1_TSR_R) != 0U)
							  {

							  }else{

								  if(Controller1_MailBoxes_semaphore[j]==un_confirmed)
						          {
									can_ip->sTxMailBox[Mailbox_index].TIR =can_ip->sTxMailBox[j].TIR;
									can_ip->sTxMailBox[Mailbox_index].TDLR=can_ip->sTxMailBox[j].TDLR;
									can_ip->sTxMailBox[Mailbox_index].TDTR=can_ip->sTxMailBox[j].TDTR;
									can_ip->sTxMailBox[Mailbox_index].TDHR=can_ip->sTxMailBox[j].TDHR;
									can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;
									Controller1_MailBox_Pending_PduId[Mailbox_index]=Controller1_MailBox_Pending_PduId[j];
									Mailbox_index+=1;
								  }else{

								       }
			               break;
			        }

    		      }
    	     }
    	  }
      }
#endif
					        /*Mailbox_index=HTH_To_MailBoxes_Map[Hth].Busy_Mailboxes_Num+HTH_To_MailBoxes_Map[Hth].Starting_Mailbox;
					         HTH_To_MailBoxes_Map[Hth].Busy_Mailboxes_Num++;*/

				 	        Controller1_MailBoxes_semaphore[Mailbox_index]= un_confirmed;

							Controller1_MailBox_Pending_PduId[Mailbox_index]=PduInfo->swPduHandle;
							can_ip->sTxMailBox[Mailbox_index].TDTR =PduInfo->length;
				 switch(CanContainer.CanConfigSet.CanHardwareObject[Hth].CanIdType)
					  {
							 case STANDARD:
										 can_ip->sTxMailBox[Mailbox_index].TIR = ((PduInfo->id<<CAN_TI0R_STID_Pos));
										 break;

							 case EXTENDED:
										 can_ip->sTxMailBox[Mailbox_index].TIR = ((PduInfo->id<<CAN_TI0R_EXID_Pos)|CAN_ID_EXT);
										 break;
							 default:/*MIXED case*/
									  if((PduInfo->id&EXT_MASK)==EXT_MASK)
									  {
										 /*ID is Extended*/
										  can_ip->sTxMailBox[Mailbox_index].TIR = ((PduInfo->id<<CAN_TI0R_EXID_Pos)|CAN_ID_EXT);
									  }
									  else
									  {
										  /*ID is Standard*/
										  can_ip->sTxMailBox[Mailbox_index].TIR = ((PduInfo->id<<CAN_TI0R_STID_Pos));
									  }
								 }
				  /* [SWS_Can_00059] Data mapping by CAN to memory is defined in a way that the
					 CAN data byte which is sent out first is array element 0, the CAN data byte which
					 is sent out last is array element 7 or 63 in case of CAN FD.
					 N.B:Our STM32 doesn't support FD*/
										can_ip->sTxMailBox[Mailbox_index].TDLR=
																			 ((uint32_t)PduInfo->sdu[3U]<< CAN_TDL0R_DATA3_Pos)|
																			 ((uint32_t)PduInfo->sdu[2U]<< CAN_TDL0R_DATA2_Pos)|
																			 ((uint32_t)PduInfo->sdu[1U]<< CAN_TDL0R_DATA1_Pos)|
																			 ((uint32_t)PduInfo->sdu[0U]<< CAN_TDL0R_DATA0_Pos);
										 if(PduInfo->length>4U)
									   {
										can_ip->sTxMailBox[Mailbox_index].TDHR=
																			((uint32_t)PduInfo->sdu[7U]<< CAN_TDH0R_DATA7_Pos)|
																			((uint32_t)PduInfo->sdu[6U]<< CAN_TDH0R_DATA6_Pos)|
																			((uint32_t)PduInfo->sdu[5U]<< CAN_TDH0R_DATA5_Pos)|
																			((uint32_t)PduInfo->sdu[4U]<< CAN_TDH0R_DATA4_Pos);
									   }
									   else
									   {
											/*MISRA*/
									   }
										can_ip->sTxMailBox[Mailbox_index].TIR|= CAN_TI0R_TXRQ;/*initiating request*/
						 }else{

								 if(Mailbox_index==CanContainer.CanConfigSet.CanHardwareObject[Hth].CanHwObjectCount+HTH_To_MailBoxes_Map[Hth]-1U)
								 {
									 return_value0=CAN_BUSY;
									 break;
								 }else{}
							  }
				     }
#endif
#if ((CanController0Activation)&&(CanController1Activation))
				     }
#endif
		   }
          }
       }
 }
 return   return_value0; /*One return point in the function (MISRA)*/
}
/*Service name:Can_MainFunction_Write
Syntax: void Can_MainFunction_Write( void )
Service ID[hex]:0x01
Description:This function performs the polling of TX confirmation when CAN_TX_PROCESSING is set to POLLING.*/
void Can_MainFunction_Write(void)
{
#if ((CanTxProcessing0==POLLING)||(CanTxProcessing1==POLLING))
	   uint32_t TX_STATUS;
#if (CanTxProcessing0==POLLING)
	  TX_STATUS=CAN0_TSR_R;

#if (Controller0_MailBox0==ON)
#if  More_Than_One_Write_Period
	 if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_MailBox0_HTH].CanMainFunctionRWPeriodRef==
			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Write0_Period_index])
	 {
#endif
		  if ((TX_STATUS & CAN_TSR_TXOK0) != 0U)
				{
		            CAN0_TSR_R=MailBox0_INT_CLEAR_FLAG ;/*clearing the interrupt flag*/
		            Controller0_MailBoxes_semaphore[MailBox0]= confirmed;
			        /*[SWS_Can_00276] The function Can_Write shall store the swPduHandle that is given inside the parameter
					  PduInfo until the Can module calls the CanIf_TxConfirmation for this request where the swPduHandle is
					  given as parameter.*/
			       /*CanIf_TxConfirmation(Controller0_MailBox_Pending_PduId[0];*/
				}else{
					   /*MISRA*/
				     }
#if  More_Than_One_Write_Period
	 }
else{
	   /*MISRA*/
    }
#endif

#endif
#if (Controller0_MailBox1==ON)
#if  More_Than_One_Write_Period
	 if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_MailBox1_HTH].CanMainFunctionRWPeriodRef==
			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Write0_Period_index])
	 {
#endif
		  if ((TX_STATUS & CAN_TSR_TXOK1) != 0U)
		 	    {
			        CAN0_TSR_R=MailBox1_INT_CLEAR_FLAG;/*clearing the interrupt flag*/
			        Controller0_MailBoxes_semaphore[MailBox1]= confirmed;
			        /*CanIf_TxConfirmation(Controller0_MailBox_Pending_PduId[1];*/
		 	    }
#endif
#if (Controller0_MailBox2==ON)
#if  More_Than_One_Write_Period
	 if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_MailBox2_HTH].CanMainFunctionRWPeriodRef==
			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Write0_Period_index])
	 {
#endif
		  if ((TX_STATUS & CAN_TSR_TXOK2) != 0U)
		 	 	{
			        CAN0_TSR_R=MailBox2_INT_CLEAR_FLAG;/*clearing the interrupt flag*/
			        Controller0_MailBoxes_semaphore[MailBox2]= confirmed;
			    /*CanIf_TxConfirmation(Controller0_MailBox_Pending_PduId[2];*/
		 	 	}else{
					   /*MISRA*/
				     }
#if  More_Than_One_Write_Period
	 }
else{
	   /*MISRA*/
     }
#endif
#endif
#endif

#if (CanTxProcessing1==POLLING)
   TX_STATUS=CAN1_TSR_R;
#if (Controller1_MailBox0==ON)
#if  More_Than_One_Write_Period
	 if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_MailBox0_HTH].CanMainFunctionRWPeriodRef==
			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Write0_Period_index])
	 {
#endif
         if ((TX_STATUS & CAN_TSR_TXOK0) != 0U)
				{
					CAN1_TSR_R=MailBox0_INT_CLEAR_FLAG ;/*clearing the interrupt flag*/
					Controlle1_MailBoxes_semaphore[MailBox0]= confirmed;
					/*[SWS_Can_00276] The function Can_Write shall store the swPduHandle that is given inside the parameter
					  PduInfo until the Can module calls the CanIf_TxConfirmation for this request where the swPduHandle is
					  given as parameter.*/
				   /*CanIf_TxConfirmation(Controller1_MailBox_Pending_PduId[0];*/
				}else{
					   /*MISRA*/
				     }
#if  More_Than_One_Write_Period
else{
	   /*MISRA*/
  }
#endif
#endif
#if (Controller1_MailBox1==ON)
#if  More_Than_One_Write_Period
	 if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_MailBox1_HTH].CanMainFunctionRWPeriodRef==
			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Write0_Period_index])
	 {
#endif
		  if ((TX_STATUS & CAN_TSR_TXOK1) != 0U)
				{
					CAN1_TSR_R=MailBox1_INT_CLEAR_FLAG;/*clearing the interrupt flag*/
					Controlle1_MailBoxes_semaphore[MailBox1]= confirmed;
					/*CanIf_TxConfirmation(Controller1_MailBox_Pending_PduId[1];*/
				}else{
					   /*MISRA*/
				     }
#if  More_Than_One_Write_Period
else{
	   /*MISRA*/
  }
#endif
#endif
#if (Controller1_MailBox2==ON)
#if  More_Than_One_Write_Period
	 if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_MailBox2_HTH].CanMainFunctionRWPeriodRef==
			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Write0_Period_index])
	 {
#endif
		  if ((TX_STATUS & CAN_TSR_TXOK2) != 0U)
				{
					CAN1_TSR_R=MailBox2_INT_CLEAR_FLAG;/*clearing the interrupt flag*/
					Controlle1_MailBoxes_semaphore[MailBox2]= confirmed;
				/*CanIf_TxConfirmation(Controller1_MailBox_Pending_PduId[2];*/
				 	 	}else{
							   /*MISRA*/
						     }
#if  More_Than_One_Write_Period
	 }
else{
	   /*MISRA*/
	 }
#endif
#endif
#endif
#endif
}


/*Service name:Can_MainFunction_Read Syntax:
void Can_MainFunction_Read(void)
Service ID[hex]:0x08
Description:This function performs the polling of RX indications when CAN_RX_PROCESSING is set to POLLING.*/

void Can_MainFunction_Read(void)
{
#if (((CanRxProcessing0==POLLING)||(CanRxProcessing1==POLLING))||((CanRxProcessing0==MIXED)||(CanRxProcessing1==MIXED)))
	   uint32_t RX_STATUS;
	   PduInfoType received_PduInfo;
	   Can_HwType Rx_Mailbox;
	   uint8_t rx_MsgData[8U];
#if (Controller0_FIFO_0_Processing==POLLING)
#if  More_Than_Read_period
	   if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_0_HRH].CanMainFunctionRWPeriodRef==
	   			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Read0_Period_index])
	 {
#endif
	   RX_STATUS=CAN0_RF0R_R;
if ((RX_STATUS & CAN_RF0R_FMP0) != 0U)
{
	    Rx_Mailbox.hoh=Controller0_FIFO_0_HRH;
	    /*[SWS_Can_00423] In case of an Extended CAN frame, the Can module shall convert the ID to a
		  standardised format since the Upper layer (CANIF) does not know whether the received CAN frame
		  is a Standard CAN frame or Extended CAN frame. In case of an Extended CAN frame, MSB of a received
		  CAN frame ID needs to be made as ‘1’ to mark the received CAN frame as Extended.*/
	    Rx_Mailbox.id=CAN_RI0R_IDE &CAN0_FIFO_0_RIR_R;
	    if (Rx_Mailbox.id == STANDARD)
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_0_HRH].CanIdType==EXTENDED)
					{
	    		       CAN0_RF0R_R|=CAN_RF0R_RFOM0;
					}
	    	    else{
					 Rx_Mailbox.id = (CAN_RI0R_STID &CAN0_FIFO_0_RIR_R) >> CAN_TI0R_STID_Pos;
					 Rx_Mailbox.controllerlId=Controller0_ID;;

						/* Get the data */
					received_PduInfo.SduLength=(CAN_RDT0R_DLC &CAN0_FIFO_0_RDTR_R)>>CAN_RDT0R_DLC_Pos;
					received_PduInfo.SduDataptr=rx_MsgData;

					/*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
					  which is received first is array element 0 the CAN data byte which is received last is array element 7*/
					received_PduInfo.SduDataptr[0] = (uint8_t)((CAN_RDL0R_DATA0 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
					received_PduInfo.SduDataptr[1] = (uint8_t)((CAN_RDL0R_DATA1 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
					received_PduInfo.SduDataptr[2] = (uint8_t)((CAN_RDL0R_DATA2 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
					received_PduInfo.SduDataptr[3] = (uint8_t)((CAN_RDL0R_DATA3 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
					received_PduInfo.SduDataptr[4] = (uint8_t)((CAN_RDH0R_DATA4 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
					received_PduInfo.SduDataptr[5]=  (uint8_t)((CAN_RDH0R_DATA5 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
					received_PduInfo.SduDataptr[6] = (uint8_t)((CAN_RDH0R_DATA6 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
					received_PduInfo.SduDataptr[7] = (uint8_t)((CAN_RDH0R_DATA7 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
					CAN0_RF0R_R|=CAN_RF0R_RFOM0;/*releasing the FIFO as if clearing the interrupt*/
						/*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
						  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
						  parameter PduInfoPtr.*/
						/*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
						  the function Can_MainFunction_Read in case of polling mode shall call the callback function
						  CanIf_RxIndication.*/
						/* CanIf_RxIndication(&Rx_Mailbox0,&received_PduInfo0);*/
				}
	    }
	    else
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_0_HRH].CanIdType==STANDARD)
					{
	    		       CAN0_RF0R_R|=CAN_RF0R_RFOM0;
					}
	    	    else
	    	    {
	    	        Rx_Mailbox.id = (((CAN_RI0R_EXID | CAN_RI0R_STID) &CAN0_FIFO_0_RIR_R) >> CAN_RI0R_EXID_Pos)|EXT_MASK;
	    	        Rx_Mailbox.controllerlId=Controller0_ID;


					received_PduInfo.SduLength=(CAN_RDT0R_DLC &CAN0_FIFO_0_RDTR_R)>>CAN_RDT0R_DLC_Pos;
					received_PduInfo.SduDataptr=rx_MsgData;

					/*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
					  which is received first is array element 0 the CAN data byte which is received last is array element 7*/
					received_PduInfo.SduDataptr[0] = (uint8_t)((CAN_RDL0R_DATA0 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
					received_PduInfo.SduDataptr[1] = (uint8_t)((CAN_RDL0R_DATA1 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
					received_PduInfo.SduDataptr[2] = (uint8_t)((CAN_RDL0R_DATA2 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
					received_PduInfo.SduDataptr[3] = (uint8_t)((CAN_RDL0R_DATA3 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
					received_PduInfo.SduDataptr[4] = (uint8_t)((CAN_RDH0R_DATA4 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
					received_PduInfo.SduDataptr[5]=  (uint8_t)((CAN_RDH0R_DATA5 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
					received_PduInfo.SduDataptr[6] = (uint8_t)((CAN_RDH0R_DATA6 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
					received_PduInfo.SduDataptr[7] = (uint8_t)((CAN_RDH0R_DATA7 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
					CAN0_RF0R_R|=CAN_RF0R_RFOM0;/*releasing the FIFO as if clearing the interrupt*/

					/*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
					  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
					  parameter PduInfoPtr.*/
					/*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
					  the function Can_MainFunction_Read in case of polling mode shall call the callback function
					  CanIf_RxIndication.*/
					/* CanIf_RxIndication(&Rx_Mailbox0,&received_PduInfo0);*/
	    	    }
	    }
}else{
	   /*MISRA*/
   }
#if  More_Than_One_Read_Period
	 }
else{
	   /*MISRA*/
	 }
#endif
#endif
#if (Controller0_FIFO_1_Processing==POLLING)
#if  More_Than_Read_period
	   if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_0_HRH].CanMainFunctionRWPeriodRef==
	   			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Read0_Period_index])
	 {
#endif
RX_STATUS=CAN0_RF1R_R;
if ((RX_STATUS & CAN_RF1R_FMP1) != 0U)
{
	    Rx_Mailbox.hoh=Controller0_FIFO_0_HRH;
	    /*[SWS_Can_00423] In case of an Extended CAN frame, the Can module shall convert the ID to a
		  standardised format since the Upper layer (CANIF) does not know whether the received CAN frame
		  is a Standard CAN frame or Extended CAN frame. In case of an Extended CAN frame, MSB of a received
		  CAN frame ID needs to be made as ‘1’ to mark the received CAN frame as Extended.*/
	    Rx_Mailbox.id=CAN_RI0R_IDE &CAN0_FIFO_1_RIR_R;
	    if (Rx_Mailbox.id == STANDARD)
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_1_HRH].CanIdType==EXTENDED)
					{
	    		       CAN0_RF1R_R|=CAN_RF1R_RFOM1;
					}
	    	    else{
					 Rx_Mailbox.id = (CAN_RI1R_STID &CAN0_FIFO_1_RIR_R) >> CAN_TI1R_STID_Pos;
					 Rx_Mailbox.controllerlId=Controller0_ID;

						/* Get the data */
					received_PduInfo.SduLength=(CAN_RDT1R_DLC &CAN0_FIFO_1_RDTR_R)>>CAN_RDT1R_DLC_Pos;
					received_PduInfo.SduDataptr=rx_MsgData;

					/*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
					  which is received first is array element 0 the CAN data byte which is received last is array element 7*/
					received_PduInfo.SduDataptr[0] = (uint8_t)((CAN_RDL1R_DATA0 &CAN0_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
					received_PduInfo.SduDataptr[1] = (uint8_t)((CAN_RDL1R_DATA1 &CAN0_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
					received_PduInfo.SduDataptr[2] = (uint8_t)((CAN_RDL1R_DATA2 &CAN0_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
					received_PduInfo.SduDataptr[3] = (uint8_t)((CAN_RDL1R_DATA3 &CAN0_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
					received_PduInfo.SduDataptr[4] = (uint8_t)((CAN_RDH1R_DATA4 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
					received_PduInfo.SduDataptr[5]=  (uint8_t)((CAN_RDH1R_DATA5 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
					received_PduInfo.SduDataptr[6] = (uint8_t)((CAN_RDH1R_DATA6 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
					received_PduInfo.SduDataptr[7] = (uint8_t)((CAN_RDH1R_DATA7 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
					CAN0_RF1R_R|=CAN_RF1R_RFOM1;/*releasing the FIFO as if clearing the interrupt*/
						/*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
						  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
						  parameter PduInfoPtr.*/
						/*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
						  the function Can_MainFunction_Read in case of polling mode shall call the callback function
						  CanIf_RxIndication.*/
						/* CanIf_RxIndication(&Rx_Mailbox0,&received_PduInfo0);*/
				}
	    }
	    else
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_0_HRH].CanIdType==STANDARD)
					{
	    		       CAN0_RF1R_R|=CAN_RF1R_RFOM1;
					}
	    	    else
	    	    {
	    	        Rx_Mailbox.id = (((CAN_RI1R_EXID | CAN_RI1R_STID) &CAN0_FIFO_1_RIR_R) >> CAN_RI1R_EXID_Pos)|EXT_MASK;
	    	        Rx_Mailbox.controllerlId=Controller0_ID;

					received_PduInfo.SduLength=(CAN_RDT1R_DLC &CAN0_FIFO_1_RDTR_R)>>CAN_RDT0R_DLC_Pos;
					received_PduInfo.SduDataptr=rx_MsgData;

					/*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
					  which is received first is array element 0 the CAN data byte which is received last is array element 7*/
					received_PduInfo.SduDataptr[0] = (uint8_t)((CAN_RDL1R_DATA0 &CAN0_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
					received_PduInfo.SduDataptr[1] = (uint8_t)((CAN_RDL1R_DATA1 &CAN0_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
					received_PduInfo.SduDataptr[2] = (uint8_t)((CAN_RDL1R_DATA2 &CAN0_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
					received_PduInfo.SduDataptr[3] = (uint8_t)((CAN_RDL1R_DATA3 &CAN0_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
					received_PduInfo.SduDataptr[4] = (uint8_t)((CAN_RDH1R_DATA4 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
					received_PduInfo.SduDataptr[5]=  (uint8_t)((CAN_RDH1R_DATA5 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
					received_PduInfo.SduDataptr[6] = (uint8_t)((CAN_RDH1R_DATA6 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
					received_PduInfo.SduDataptr[7] = (uint8_t)((CAN_RDH1R_DATA7 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
					CAN0_RF1R_R|=CAN_RF1R_RFOM1;/*releasing the FIFO as if clearing the interrupt*/

					/*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
					  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
					  parameter PduInfoPtr.*/
					/*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
					  the function Can_MainFunction_Read in case of polling mode shall call the callback function
					  CanIf_RxIndication.*/
					/* CanIf_RxIndication(&Rx_Mailbox0,&received_PduInfo0);*/
	    	    }
	    }
}else{
	   /*MISRA*/
     }
#if  More_Than_One_Read_Period
	 }
else{
	   /*MISRA*/
	 }
#endif
#endif
#if (Controller1_FIFO_0_Processing==POLLING)
#if  More_Than_Read_period
	   if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_FIFO_0_HRH].CanMainFunctionRWPeriodRef==
	   			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Read0_Period_index])
	 {
#endif

     RX_STATUS=CAN1_RF0R_R;
		if ((RX_STATUS & CAN_RF0R_FMP0) != 0U)
		{
	    Rx_Mailbox.hoh=Controller1_FIFO_0_HRH;
	    /*[SWS_Can_00423] In case of an Extended CAN frame, the Can module shall convert the ID to a
		  standardised format since the Upper layer (CANIF) does not know whether the received CAN frame
		  is a Standard CAN frame or Extended CAN frame. In case of an Extended CAN frame, MSB of a received
		  CAN frame ID needs to be made as ‘1’ to mark the received CAN frame as Extended.*/
	    Rx_Mailbox.id=CAN_RI0R_IDE &CAN0_FIFO_0_RIR_R;
	    if (Rx_Mailbox.id == STANDARD)
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_FIFO_0_HRH].CanIdType==EXTENDED)
					{
	    		       CAN1_RF0R_R|=CAN_RF0R_RFOM0;
					}
	    	    else{
					 Rx_Mailbox.id = (CAN_RI0R_STID &CAN0_FIFO_0_RIR_R) >> CAN_TI0R_STID_Pos;
					 Rx_Mailbox.controllerId=Controller1_ID;

						/* Get the data */
					received_PduInfo.SduLength=(CAN_RDT0R_DLC &CAN1_FIFO_0_RDTR_R)>>CAN_RDT0R_DLC_Pos;
					received_PduInfo.SduDataptr=rx_MsgData;

					/*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
					  which is received first is array element 0 the CAN data byte which is received last is array element 7*/
					received_PduInfo.SduDataptr[0] = (uint8_t)((CAN_RDL0R_DATA0 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
					received_PduInfo.SduDataptr[1] = (uint8_t)((CAN_RDL0R_DATA1 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
					received_PduInfo.SduDataptr[2] = (uint8_t)((CAN_RDL0R_DATA2 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
					received_PduInfo.SduDataptr[3] = (uint8_t)((CAN_RDL0R_DATA3 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
					received_PduInfo.SduDataptr[4] = (uint8_t)((CAN_RDH0R_DATA4 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
					received_PduInfo.SduDataptr[5]=  (uint8_t)((CAN_RDH0R_DATA5 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
					received_PduInfo.SduDataptr[6] = (uint8_t)((CAN_RDH0R_DATA6 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
					received_PduInfo.SduDataptr[7] = (uint8_t)((CAN_RDH0R_DATA7 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
					CAN1_RF0R_R|=CAN_RF0R_RFOM0; /*releasing the FIFO as if clearing the interrupt*/
						/*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
						  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
						  parameter PduInfoPtr.*/
						/*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
						  the function Can_MainFunction_Read in case of polling mode shall call the callback function
						  CanIf_RxIndication.*/
						/* CanIf_RxIndication(&Rx_Mailbox0,&received_PduInfo0);*/
				}
	    }
	    else
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_0_HRH].CanIdType==STANDARD)
					{
	    		       CAN1_RF0R_R|=CAN_RF0R_RFOM0;
					}
	    	    else
	    	    {
	    	        Rx_Mailbox.id = (((CAN_RI0R_EXID | CAN_RI0R_STID) &CAN0_FIFO_0_RIR_R) >> CAN_RI0R_EXID_Pos)|EXT_MASK;
	    	        Rx_Mailbox.controllerlId=Controller1_ID
	    	       						/* Get the data */
					received_PduInfo.SduLength=(CAN_RDT0R_DLC &CAN1_FIFO_0_RDTR_R)>>CAN_RDT0R_DLC_Pos;
					received_PduInfo.SduDataptr=rx_MsgData;

					/*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
					  which is received first is array element 0 the CAN data byte which is received last is array element 7*/
					received_PduInfo.SduDataptr[0] = (uint8_t)((CAN_RDL0R_DATA0 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
					received_PduInfo.SduDataptr[1] = (uint8_t)((CAN_RDL0R_DATA1 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
					received_PduInfo.SduDataptr[2] = (uint8_t)((CAN_RDL0R_DATA2 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
					received_PduInfo.SduDataptr[3] = (uint8_t)((CAN_RDL0R_DATA3 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
					received_PduInfo.SduDataptr[4] = (uint8_t)((CAN_RDH0R_DATA4 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
					received_PduInfo.SduDataptr[5]=  (uint8_t)((CAN_RDH0R_DATA5 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
					received_PduInfo.SduDataptr[6] = (uint8_t)((CAN_RDH0R_DATA6 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
					received_PduInfo.SduDataptr[7] = (uint8_t)((CAN_RDH0R_DATA7 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
					CAN1_RF0R_R|=CAN_RF0R_RFOM0;/*releasing the FIFO as if clearing the interrupt*/

					/*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
					  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
					  parameter PduInfoPtr.*/
					/*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
					  the function Can_MainFunction_Read in case of polling mode shall call the callback function
					  CanIf_RxIndication.*/
					/* CanIf_RxIndication(&Rx_Mailbox0,&received_PduInfo0);*/
	    	    }
	    }
}else{
	   /*MISRA*/
     }
#if  More_Than_One_Read_Period
	 }
else{
	   /*MISRA*/
	 }
#endif
#endif

#if (Controller1_FIFO_1_Processing==POLLING)
#if  More_Than_Read_period
	   if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_FIFO_1_HRH].CanMainFunctionRWPeriodRef==
	   			 &CanContainer.CanGeneral.CanMainFunctionRWPeriods[Can_MainFunction_Read0_Period_index])
	 {
#endif
   RX_STATUS=CAN1_RF1R_R;
if ((RX_STATUS & CAN_RF1R_FMP1) != 0U)
{
	    Rx_Mailbox.hoh=Controller1_FIFO_1_HRH;
	    /*[SWS_Can_00423] In case of an Extended CAN frame, the Can module shall convert the ID to a
		  standardised format since the Upper layer (CANIF) does not know whether the received CAN frame
		  is a Standard CAN frame or Extended CAN frame. In case of an Extended CAN frame, MSB of a received
		  CAN frame ID needs to be made as ‘1’ to mark the received CAN frame as Extended.*/
	    Rx_Mailbox.id=CAN_RI0R_IDE &CAN1_FIFO_1_RIR_R;
	    if (Rx_Mailbox.id == STANDARD)
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_FIFO_1_HRH].CanIdType==EXTENDED)
					{
	    		       CAN1_RF1R_R|=CAN_RF1R_RFOM1;
					}
	    	    else{
					 Rx_Mailbox.id = (CAN_RI1R_STID &CAN1_FIFO_1_RIR_R) >> CAN_TI1R_STID_Pos;
					 Rx_Mailbox.controllerlId=Controller1_ID;

						/* Get the data */
					received_PduInfo.SduLength=(CAN_RDT1R_DLC &CAN1_FIFO_1_RDTR_R)>>CAN_RDT1R_DLC_Pos;
					received_PduInfo.SduDataptr=rx_MsgData;

					/*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
					  which is received first is array element 0 the CAN data byte which is received last is array element 7*/
					received_PduInfo.SduDataptr[0] = (uint8_t)((CAN_RDL1R_DATA0 &CAN1_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
					received_PduInfo.SduDataptr[1] = (uint8_t)((CAN_RDL1R_DATA1 &CAN1_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
					received_PduInfo.SduDataptr[2] = (uint8_t)((CAN_RDL1R_DATA2 &CAN1_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
					received_PduInfo.SduDataptr[3] = (uint8_t)((CAN_RDL1R_DATA3 &CAN1_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
					received_PduInfo.SduDataptr[4] = (uint8_t)((CAN_RDH1R_DATA4 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
					received_PduInfo.SduDataptr[5]=  (uint8_t)((CAN_RDH1R_DATA5 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
					received_PduInfo.SduDataptr[6] = (uint8_t)((CAN_RDH1R_DATA6 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
					received_PduInfo.SduDataptr[7] = (uint8_t)((CAN_RDH1R_DATA7 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
					CAN1_RF1R_R|=CAN_RF1R_RFOM1;/*releasing the FIFO as if clearing the interrupt*/
						/*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
						  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
						  parameter PduInfoPtr.*/
						/*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
						  the function Can_MainFunction_Read in case of polling mode shall call the callback function
						  CanIf_RxIndication.*/
						/* CanIf_RxIndication(&Rx_Mailbox0,&received_PduInfo0);*/
				}
	    }
	    else
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_0_HRH].CanIdType==STANDARD)
					{
	    		       CAN1_RF1R_R|=CAN_RF1R_RFOM1;
					}
	    	    else
	    	    {
	    	        Rx_Mailbox.id = (((CAN_RI1R_EXID | CAN_RI1R_STID) &CAN0_FIFO_1_RIR_R) >> CAN_RI1R_EXID_Pos)|EXT_MASK;
	    	        Rx_Mailbox.controllerId=Controller1_ID;

					received_PduInfo.SduLength=(CAN_RDT1R_DLC &CAN0_FIFO_1_RDTR_R)>>CAN_RDT0R_DLC_Pos;
					received_PduInfo.SduDataptr=rx_MsgData;

					/*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
					  which is received first is array element 0 the CAN data byte which is received last is array element 7*/
					received_PduInfo.SduDataptr[0] = (uint8_t)((CAN_RDL1R_DATA0 &CAN1_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
					received_PduInfo.SduDataptr[1] = (uint8_t)((CAN_RDL1R_DATA1 &CAN1_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
					received_PduInfo.SduDataptr[2] = (uint8_t)((CAN_RDL1R_DATA2 &CAN1_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
					received_PduInfo.SduDataptr[3] = (uint8_t)((CAN_RDL1R_DATA3 &CAN1_FIFO_1_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
					received_PduInfo.SduDataptr[4] = (uint8_t)((CAN_RDH1R_DATA4 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
					received_PduInfo.SduDataptr[5]=  (uint8_t)((CAN_RDH1R_DATA5 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
					received_PduInfo.SduDataptr[6] = (uint8_t)((CAN_RDH1R_DATA6 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
					received_PduInfo.SduDataptr[7] = (uint8_t)((CAN_RDH1R_DATA7 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
					CAN1_RF1R_R|=CAN_RF1R_RFOM1;/*releasing the FIFO as if clearing the interrupt*/

					/*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
					  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
					  parameter PduInfoPtr.*/
					/*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
					  the function Can_MainFunction_Read in case of polling mode shall call the callback function
					  CanIf_RxIndication.*/
					/* CanIf_RxIndication(&Rx_Mailbox0,&received_PduInfo0);*/
	    	    }
	    }
}else{
	   /*MISRA*/
     }
#if  More_Than_One_Read_Period
	 }
else{
	   /*MISRA*/
	 }
#endif
#endif
#endif
}
/*Service name:Can_MainFunction_BusOff Syntax:
void Can_MainFunction_BusOff(void)
Service ID[hex]:0x09
Description:This function performs the polling of bus-off events that are configured statically as 'to be polled.*/
void Can_MainFunction_BusOff(void)
{
#if (CanBusoffProcessing0==POLLING||CanBusoffProcessing0==POLLING)
	uint8_t BUS_OFF_STATUS;
#if (CanBusoffProcessing0==POLLING)
  BUS_OFF_STATUS = CAN0_ESR_R;
  if ((BUS_OFF_STATUS & CAN_ESR_BOFF) != 0U)
	  {
		/*Bus-Off */
	   CAN0_MCR_R=Controller0_STOP_MODE;/*Value responsible for putting controller in stopped mode*/
	   CAN0_IER_R=zero;/*disabling interrupts not required for this state*/
	   CAN0_MSR_R =ERRI_Clear ;/*clearing error INT flag*/
	   CAN0_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2);

	   /*[SWS_Can_00272]After bus-off detection, the CAN controller shall transition to the state
	   STOPPED and the Can module shall ensure that the CAN controller doesn’t participate on the
	   network anymore.*/
	   Controller0_StopTransition_Poll();
	   /*[SWS_Can_00020]  triggered by hardware if the CAN controller reaches bus-off state
		 The CanIf module is notified with the function CanIf_ControllerBusOff after bus off
		 detection*/
	   /*CanIf_ControllerBusOff(Controller0_ID)*/
	  }
#endif
#if (CanBusoffProcessing1==POLLING)
  BUS_OFF_STATUS = CAN1_ESR_R;
  if ((BUS_OFF_STATUS & CAN_ESR_BOFF) != 0U)
	  {
		/*Bus-Off */
	   CAN1_MCR_R=Controller0_STOP_MODE;/*Value responsible for putting controller in stopped mode*/
	   CAN1_IER_R=zero;/*disabling interrupts not required for this state*/
	   CAN1_MSR_R =ERRI_Clear ;/*clearing error INT flag*/
	   CAN1_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2);
	   /*[SWS_Can_00272]After bus-off detection, the CAN controller shall transition to the state
	   STOPPED and the Can module shall ensure that the CAN controller doesn’t participate on the
	   network anymore.*/
	   Controller0_StopTransition_Poll();
	   /*[SWS_Can_00020]  triggered by hardware if the CAN controller reaches bus-off state
		 The CanIf module is notified with the function CanIf_ControllerBusOff after bus off
		 detection*/
	   /*CanIf_ControllerBusOff(Controller1_ID)*/
	  }
#endif
#endif
}


/*Service name:Can_MainFunction_Wakeup
Syntax:void Can_MainFunction_Wakeup( void )
Service ID[hex]:0x0a
Description:This function performs the polling of wake-up events that are configured statically
as 'to be polled'.*/
void Can_MainFunction_Wakeup(void)
{
#if ((CanWakeupProcessing0==POLLING)||(CanWakeupProcessing1==POLLING))
	uint8_t  WAKEUP_STATUS;
#if (CanWakeupProcessing0==POLLING)
	WAKEUP_STATUS=CAN0_MSR_R;
	 if ((WAKEUP_STATUS& CAN_MSR_WKUI) != 0U)
		 {

		 /*SWS_Can_00270] ⌈On hardware wake-up (triggered by a wake-up event from CAN bus),
		 the CAN controller shall transition into the state STOPPED.*/
		           WakeUp_Flag[Controller0_ID]=one;
				   CAN0_MCR_R=Controller0_STOP_MODE;/*Value responsible for putting controller in stopped mode*/
				   CAN0_IER_R=zero;/*disabling interrupts not required for this state*/
		           CAN0_MSR_R =WKUI_Clear;/*clearing error INT flag*/
				   CAN0_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2); /*[SWS_Can_00273] After bus-off detection, the Can module shall cancel still pending messages.*/
				   /*Cancel pending RX messages*/
				   Controller0_StopTransition_Poll();
				    /*[SWS_Can_00271] On hardware wake up (triggered by a wake-up event from CAN bus),
				      the Can module shall call the function EcuM_CheckWakeup either in interrupt context
					  or in the context of Can_MainFunction_Wakeup.*/

				    /*EcuM_CheckWakeup(*CanContainer.CanConfigSet.CanController[Controller0_ID].CanWakeupSourceRef)*/
		 }
#endif
#if (CanWakeupProcessing1==POLLING)
	 WAKEUP_STATUS=CAN1_MSR_R;
	 if ((WAKEUP_STATUS & CAN_MSR_WKUI) != 0U)
		 {
		           WakeUp_Flag[Controller1_ID]=zero;
				   CAN1_MCR_R=Controller1_STOP_MODE;/*Value responsible for putting controller in stopped mode*/
				   CAN1_IER_R=zero;/*disabling interrupts not required for this state*/
		           CAN1_MSR_R =WKUI_Clear;/*clearing error INT flag*/
				   CAN1_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2); /*[SWS_Can_00273] After bus-off detection, the Can module shall cancel still
																			  pending messages.*/
				   Controller1_StopTransition_Poll();
				    /*[SWS_Can_00271] On hardware wake up (triggered by a wake-up event from CAN bus),
				      the Can module shall call the function EcuM_CheckWakeup either in interrupt context
					  or in the context of Can_MainFunction_Wakeup.*/

				    /*EcuM_CheckWakeup(*CanContainer.CanConfigSet.CanController[Controller0_ID].CanWakeupSourceRef)*/
		 }
#endif
#endif
}

/*Service name:Can_CheckWakeup
 Syntax:Std_ReturnType Can_CheckWakeup( uint8 Controller )
Service ID[hex]:0x0b
Sync/Async:Synchronous R
eentrancy:Non Reentrant
Parameters (in):Controller
 Parameters (in out):None
Parameters (out):None
Return value:Std_ReturnType
                           E_OK: API call has been accepted ,E_NOT_OK: API call has not been accepted
Description:This function checks if a wake-up has occurred for the given controller.*/
#if (CanWakeupFunctionalityAPI0||CanWakeupFunctionalityAPI1)
Std_ReturnType Can_CheckWakeup(uint8 Controller)
{


if(WakeUp_Flag[Controller])
{
	/*EcuM_SetWakeupEvent*/
	WakeUp_Flag[Controller]=zero;
}else{
	   /*MISRA*/
     }
}
#endif
/*Service name:Can_MainFunction_Mode Syntax:
void Can_MainFunction_Mode(void)
Service ID[hex]:0x0c
Description:This function performs the polling of CAN controller mode transitions.*/
void Can_MainFunction_Mode(void)
{
#if (CanController0Activation)
 if(state_transition_flag[Controller0_ID])
 {
         state_transition_flag[0]=zero;

         switch(Requested_Mode_Transition[Controller0_ID])
         {
           case CAN_CS_STARTED :
				  if((CAN0_MSR_R & CAN_MSR_INAK) != CAN_MSR_INAK)
					{
					  Can_ControllerMode[Controller0_ID]=CAN_CS_STARTED;
					  /*void CanIf_ControllerModeIndication(Controller0_ID,CAN_CS_STARTED)*/
					}else{
							/*MISRA*/
						 }
				  break;
           case CAN_CS_STOPPED :
        	   if((CAN0_MSR_R & CAN_MSR_INAK) == CAN_MSR_INAK)
					{
        		       Can_ControllerMode[Controller0_ID]=CAN_CS_STOPPED;
					  /*void CanIf_ControllerModeIndication(Controller0_ID,CAN_CS_STOPPED )*/
					}else{
							/*MISRA*/
						 }
                   break;
           case CAN_CS_SLEEP :
        	   if((CAN0_MSR_R & CAN_MSR_SLAK_Msk) ==CAN_MSR_SLAK_Msk)
        	   {
        		    Can_ControllerMode[Controller0_ID]=CAN_CS_SLEEP;
        		   /*void CanIf_ControllerModeIndication(Controller0_ID,CAN_CS_SLEEP)*/
        	   }else{
					/*MISRA*/
				    }
        	   break;
         }
     }
 else{
	   /*MISRA*/
     }
#endif
#if (CanController1Activation)
 if(state_transition_flag[Controller1_ID])
 {
         state_transition_flag[Controller1_ID]=zero;

         switch(Requested_Mode_Transition[Controller1_ID])
         {
           case CAN_CS_STARTED :
				  if((CAN1_MSR_R & CAN_MSR_INAK) != CAN_MSR_INAK)
					{
					  Can_ControllerMode[Controller1_ID]=CAN_CS_STARTED;
					  /*void CanIf_ControllerModeIndication(Controller1_ID,CAN_CS_STARTED)*/
					}else{
							/*MISRA*/
						 }
           case CAN_CS_STOPPED :
        	   if((CAN1_MSR_R & CAN_MSR_INAK) == CAN_MSR_INAK)
					{
        		      Can_ControllerMode[Controller1_ID]=CAN_CS_STOPPED;
					  /*void CanIf_ControllerModeIndication(Controller1_ID,CAN_CS_STOPPED )*/
					}else{
							/*MISRA*/
						 }
           case CAN_CS_SLEEP :
        	   if((CAN1_MSR_R & CAN_MSR_SLAK_Msk)==CAN_MSR_SLAK_Msk)
        	   {
        		    Can_ControllerMode[Controller1_ID]=CAN_CS_SLEEP;
        		   /*void CanIf_ControllerModeIndication(Controller1_ID,CAN_CS_SLEEP)*/
        	   }else{
					/*MISRA*/
				    }
         }
     }
 else{
	   /*MISRA*/
     }
#endif
}
void Can0_BUSOFF_and_WAKEUP_InterruptHandler(void)
{
	uint8_t  STATUS0;
#if (CanWakeupProcessing0==INTERRUPT)
	STATUS0=CAN0_MSR_R;
	 if ((STATUS0 & CAN_MSR_WKUI) != 0U)
		 {
		           WakeUp_Flag[Controller0_ID]=one;
		           CAN0_MCR_R=Controller0_STOP_MODE;/*Value responsible for putting controller in stopped mode*/
				   CAN0_IER_R=zero;/*disabling interrupts not required for this state*/
		           CAN0_MSR_R = WKUI_Clear;/*clearing error INT flag*/
				   CAN0_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2);
					 /*SWS_Can_00270] ⌈On hardware wake-up (triggered by a wake-up event from CAN bus),
						  the CAN controller shall transition into the state STOPPED.*/
				   Controller0_StopTransition_Poll();
				    /*[SWS_Can_00271] On hardware wake up (triggered by a wake-up event from CAN bus),
				      the Can module shall call the function EcuM_CheckWakeup either in interrupt context
					  or in the context of Can_MainFunction_Wakeup.*/

				    /*EcuM_CheckWakeup(*CanContainer.CanConfigSet.CanController[Controller0_ID].CanWakeupSourceRef)*/
		 }
#endif
#if (CanBusoffProcessing0==INTERRUPT)
	    STATUS0 = CAN0_ESR_R;
	  if ((STATUS0 & CAN_ESR_BOFF) != 0U)
	      {
	        /*Bus-Off */
		   CAN0_MCR_R=Controller0_STOP_MODE;/*Value responsible for putting controller in stopped mode*/
		   CAN0_IER_R=zero;/*disabling interrupts not required for this state*/
		   CAN0_MSR_R =ERRI_Clear;/*clearing error INT flag*/
		   CAN0_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2); /*[SWS_Can_00273] After bus-off detection, the Can module shall cancel still
		                                                              pending messages.*/
		   /*[SWS_Can_00272]After bus-off detection, the CAN controller shall transition to the state
		   STOPPED and the Can module shall ensure that the CAN controller doesn’t participate on the
		   network anymore.*/
		   Controller0_StopTransition_Poll();
		   /*[SWS_Can_00020]  triggered by hardware if the CAN controller reaches bus-off state
		     The CanIf module is notified with the function CanIf_ControllerBusOff after bus off
		     detection*/
		   /*CanIf_ControllerBusOff(Controller0_ID)*/
 }
#endif
}
void Can0_TX_InterruptHandler()
{
   uint32_t TX0_STATUS=CAN0_TSR_R;

#if (Controller0_MailBox0==ON)
	  if ((TX0_STATUS & CAN_TSR_TXOK0) != 0U)
			{
	            CAN0_TSR_R=MailBox0_INT_CLEAR_FLAG ;/*clearing the interrupt flag*/
	            Controller0_MailBoxes_semaphore[MailBox0]= confirmed;
		        /*HTH_To_MailBoxes_Map[ Controller0_MailBox0_HTH].Busy_Mailboxes_Num--;*/
		        /*[SWS_Can_00276] The function Can_Write shall store the swPduHandle that is given inside the parameter
				  PduInfo until the Can module calls the CanIf_TxConfirmation for this request where the swPduHandle is
				  given as parameter.*/
		       /*CanIf_TxConfirmation(Controller0_MailBox_Pending_PduId[0];*/
			}
#endif
#if (Controller0_MailBox1==ON)
	  if ((TX0_STATUS & CAN_TSR_TXOK1) != 0U)
	 	    {
		        CAN0_TSR_R=MailBox1_INT_CLEAR_FLAG;/*clearing the interrupt flag*/
		        Controller0_MailBoxes_semaphore[MailBox1]= confirmed;
//		        HTH_To_MailBoxes_Map[Controller0_MailBox1_HTH].Busy_Mailboxes_Num--;
		        /*CanIf_TxConfirmation(Controller0_MailBox_Pending_PduId[1];*/
	 	    }
#endif
#if (Controller0_MailBox2==ON)
	  if ((TX0_STATUS & CAN_TSR_TXOK2) != 0U)
	 	 	{
		        CAN0_TSR_R=MailBox2_INT_CLEAR_FLAG;/*clearing the interrupt flag*/
		        Controller0_MailBoxes_semaphore[MailBox2]= confirmed;
//		        HTH_To_MailBoxes_Map[Controller0_MailBox2_HTH].Busy_Mailboxes_Num--;
		    /*CanIf_TxConfirmation(Controller0_MailBox_Pending_PduId[2];*/
	 	 	}
#endif
}
void Can0_RX0_InterruptHandler(void)
{
	/*[SWS_Can_00395] Can module shall raise the runtime error CAN_E_DATALOST in case of
		      “overwrite” or “overrun” event detection.()*/

uint8_t RX00_STATUS=CAN0_RF0R_R;

if ((RX00_STATUS & CAN_RF0R_FOVR0) != 0U)
{
   CAN0_RF0R_R=CAN_RF0R_FOVR0;
  /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID,   ,CAN_E_DATALOST )*/
  return;
}

#if (Controller0_FIFO_0_Processing==INTERRUPT)
	 PduInfoType received_PduInfo00;
	 Can_HwType Rx_Mailbox00;
	 uint8_t rx0_MsgData0[8U];

if ((RX00_STATUS & CAN_RF0R_FMP0) != 0U)
{

	    Rx_Mailbox00.hoh=Controller0_FIFO_0_HRH;
	    /*[SWS_Can_00423] In case of an Extended CAN frame, the Can module shall convert the ID to a
		  standardised format since the Upper layer (CANIF) does not know whether the received CAN frame
		  is a Standard CAN frame or Extended CAN frame. In case of an Extended CAN frame, MSB of a received
		  CAN frame ID needs to be made as ‘1’ to mark the received CAN frame as Extended.*/
	    Rx_Mailbox00.id=CAN_RI0R_IDE &CAN0_FIFO_0_RIR_R;
	    if (Rx_Mailbox00.id == STANDARD)
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_0_HRH].CanIdType==EXTENDED)
					{
	    		       CAN0_RF0R_R|=CAN_RF0R_RFOM0;
					   return;
					}
	    	      else
	    	      {
	    	         Rx_Mailbox00.id = (CAN_RI0R_STID &CAN0_FIFO_0_RIR_R) >> CAN_TI0R_STID_Pos;
	    	      }
	    }
	    else
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_0_HRH].CanIdType==STANDARD)
					{
	    		       CAN0_RF0R_R|=CAN_RF0R_RFOM0;
					   return;
					}
	    	   else
	    	   {
	    	      Rx_Mailbox00.id = (((CAN_RI0R_EXID | CAN_RI0R_STID) &CAN0_FIFO_0_RIR_R) >> CAN_RI0R_EXID_Pos)|EXT_MASK;
	    	   }
	    }
	    Rx_Mailbox00.controllerlId=Controller0_ID;;

	    /* Get the data */
	    received_PduInfo00.SduLength=(CAN_RDT0R_DLC &CAN0_FIFO_0_RDTR_R)>>CAN_RDT0R_DLC_Pos;
	    received_PduInfo00.SduDataptr=rx0_MsgData0;

	    /*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
	      which is received first is array element 0 the CAN data byte which is received last is array element 7*/
	    received_PduInfo00.SduDataptr[0] = (uint8_t)((CAN_RDL0R_DATA0 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
	    received_PduInfo00.SduDataptr[1] = (uint8_t)((CAN_RDL0R_DATA1 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
	    received_PduInfo00.SduDataptr[2] = (uint8_t)((CAN_RDL0R_DATA2 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
	    received_PduInfo00.SduDataptr[3] = (uint8_t)((CAN_RDL0R_DATA3 &CAN0_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
	    received_PduInfo00.SduDataptr[4] = (uint8_t)((CAN_RDH0R_DATA4 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
	    received_PduInfo00.SduDataptr[5]=  (uint8_t)((CAN_RDH0R_DATA5 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
	    received_PduInfo00.SduDataptr[6] = (uint8_t)((CAN_RDH0R_DATA6 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
	    received_PduInfo00.SduDataptr[7] = (uint8_t)((CAN_RDH0R_DATA7 &CAN0_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
	    CAN0_RF0R_R|=CAN_RF0R_RFOM0;/*releasing the FIFO as if clearing the interrupt*/

	    /*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
	  	  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
	  	  parameter PduInfoPtr.*/
	    /*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
		  the function Can_MainFunction_Read in case of polling mode shall call the callback function
		  CanIf_RxIndication.*/
	    /* CanIf_RxIndication(&Rx_Mailbox00,&received_PduInfo00);*/
}
#endif
	    /*[SWS_Can_00395] Can module shall raise the runtime error CAN_E_DATALOST in case of
	      “overwrite” or “overrun” event detection.()*/


}
void Can0_RX1_InterruptHandler(void)
{
	 uint8_t RX01_STATUS=CAN0_RF1R_R;
	  if (( RX01_STATUS& CAN_RF1R_FOVR1) != 0U)
	  {
		  CAN0_RF1R_R=CAN_RF1R_FOVR1;
		  /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID,   ,CAN_E_DATALOST )*/
		  return;
	  }
#if (Controller0_FIFO_1_Processing==INTERRUPT)
 PduInfoType received_PduInfo01;
 Can_HwType Rx_Mailbox01;
 uint8_t rx0_MsgData1[8U];
if (RX01_STATUS & CAN_RF1R_FMP1)
{
    Rx_Mailbox01.hoh=Controller0_FIFO_1_HRH;

	    /*[SWS_Can_00423] In case of an Extended CAN frame, the Can module shall convert the ID to a
		  standardised format since the Upper layer (CANIF) does not know whether the received CAN frame
		  is a Standard CAN frame or Extended CAN frame. In case of an Extended CAN frame, MSB of a received
		  CAN frame ID needs to be made as ‘1’ to mark the received CAN frame as Extended.*/
         Rx_Mailbox01.id=CAN_RI0R_IDE &CAN0_FIFO_1_RIR_R;
			if (Rx_Mailbox01.id == STANDARD)
			{
				if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_1_HRH].CanIdType==EXTENDED)
				{
					 CAN0_RF1R_R|=CAN_RF1R_RFOM1;
                   return;
				}
				else
				{
					 Rx_Mailbox01.id = (CAN_RI1R_STID &CAN0_FIFO_1_RIR_R ) >> CAN_TI0R_STID_Pos;
				}
			}
			else
			{
				if(CanContainer.CanConfigSet.CanHardwareObject[Controller0_FIFO_1_HRH].CanIdType==STANDARD)
					{
					   CAN0_RF1R_R|=CAN_RF1R_RFOM1;
					   return;
					}
				else{
				     Rx_Mailbox01.id = (((CAN_RI1R_EXID | CAN_RI1R_STID) &CAN0_FIFO_1_RIR_R ) >> CAN_RI0R_EXID_Pos)|EXT_MASK;
				    }
			}

	    Rx_Mailbox01.controllerlId=Controller0_ID;;

	    /* Get the data */
	    received_PduInfo01.SduLength=(CAN_RDT1R_DLC &CAN0_FIFO_1_RDTR_R)>>CAN_RDT0R_DLC_Pos;
	    received_PduInfo01.SduDataptr=rx0_MsgData1;

	    /*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
	      which is received first is array element 0 the CAN data byte which is received last is array element 7*/
	    received_PduInfo01.SduDataptr[0] = (uint8_t)((CAN_RDL0R_DATA1 &CAN0_FIFO_1_RDTR_R) >> CAN_RDL0R_DATA0_Pos);
	    received_PduInfo01.SduDataptr[1] = (uint8_t)((CAN_RDL0R_DATA1 &CAN0_FIFO_1_RDTR_R) >> CAN_RDL0R_DATA1_Pos);
	    received_PduInfo01.SduDataptr[2] = (uint8_t)((CAN_RDL0R_DATA2 &CAN0_FIFO_1_RDTR_R) >> CAN_RDL0R_DATA2_Pos);
	    received_PduInfo01.SduDataptr[3] = (uint8_t)((CAN_RDL0R_DATA3 &CAN0_FIFO_1_RDTR_R) >> CAN_RDL0R_DATA3_Pos);
	    received_PduInfo01.SduDataptr[4] = (uint8_t)((CAN_RDH0R_DATA4 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
	    received_PduInfo01.SduDataptr[5]=  (uint8_t)((CAN_RDH0R_DATA5 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
	    received_PduInfo01.SduDataptr[6] = (uint8_t)((CAN_RDH0R_DATA6 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
	    received_PduInfo01.SduDataptr[7] = (uint8_t)((CAN_RDH0R_DATA7 &CAN0_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
	    CAN0_RF1R_R|=CAN_RF1R_RFOM1;/*releasing the FIFO as if clearing the interrupt*/
	    /*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
	  	  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
	  	  parameter PduInfoPtr.*/
	    /*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
		  the function Can_MainFunction_Read in case of polling mode shall call the callback function
		  CanIf_RxIndication.*/
	    /* CanIf_RxIndication(&Rx_Mailbox01,&received_PduInfo01);*/
    return;
}
#endif

}

void Can1_BUSOFF_and_WAKEUP_InterruptHandler(void)
{
	uint8_t  STATUS1;
#if (CanWakeupProcessing1==INTERRUPT)
	STATUS1=CAN1_MSR_R;
	 if ((STATUS1 & CAN_MSR_WKUI) != 0U)
		 {
		           WakeUp_Flag[Controller1_ID]=one;
		           CAN1_MCR_R=Controller1_STOP_MODE;/*Value responsible for putting controller in stopped mode*/
				   CAN1_IER_R=zero;/*disabling interrupts not required for this state*/
		           CAN1_MSR_R = WKUI_Clear;/*clearing error INT flag*/
				   CAN1_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2);
					 /*SWS_Can_00270] ⌈On hardware wake-up (triggered by a wake-up event from CAN bus),
						  the CAN controller shall transition into the state STOPPED.*/
				   Controller1_StopTransition_Poll();
				    /*[SWS_Can_00271] On hardware wake up (triggered by a wake-up event from CAN bus),
				      the Can module shall call the function EcuM_CheckWakeup either in interrupt context
					  or in the context of Can_MainFunction_Wakeup.*/

				    /*EcuM_CheckWakeup(*CanContainer.CanConfigSet.CanController[Controller1_ID].CanWakeupSourceRef)*/
		 }
#endif
#if (CanBusoffProcessing1==INTERRUPT)
	    STATUS1 = CAN1_ESR_R;
	  if ((STATUS1 & CAN_ESR_BOFF) != 0U)
	      {
	        /*Bus-Off */
		   CAN1_MCR_R=Controller0_STOP_MODE;/*Value responsible for putting controller in stopped mode*/
		   CAN1_IER_R=zero;/*disabling interrupts not required for this state*/
		   CAN1_MSR_R =ERRI_Clear;/*clearing error INT flag*/
		   CAN1_TSR_R|=(CAN_TSR_ABRQ0|CAN_TSR_ABRQ1|CAN_TSR_ABRQ2); /*[SWS_Can_00273] After bus-off detection, the Can module shall cancel still
		                                                              pending messages.*/
		   /*[SWS_Can_00272]After bus-off detection, the CAN controller shall transition to the state
		   STOPPED and the Can module shall ensure that the CAN controller doesn’t participate on the
		   network anymore.*/
		   Controller1_StopTransition_Poll();
		   /*[SWS_Can_00020]  triggered by hardware if the CAN controller reaches bus-off state
		     The CanIf module is notified with the function CanIf_ControllerBusOff after bus off
		     detection*/
		   /*CanIf_ControllerBusOff(Controller1_ID)*/
 }
#endif
}
void Can1_TX_InterruptHandler()
{
   uint32_t TX1_STATUS=CAN1_TSR_R;

#if (Controller1_MailBox0==ON)
	  if ((TX1_STATUS & CAN_TSR_TXOK0) != 0U)
			{
	            CAN1_TSR_R=MailBox0_INT_CLEAR_FLAG ;/*clearing the interrupt flag*/
	            Controller1_MailBoxes_semaphore[MailBox0]= confirmed;
		        /*HTH_To_MailBoxes_Map[ Controller0_MailBox0_HTH].Busy_Mailboxes_Num--;*/
		        /*[SWS_Can_00276] The function Can_Write shall store the swPduHandle that is given inside the parameter
				  PduInfo until the Can module calls the CanIf_TxConfirmation for this request where the swPduHandle is
				  given as parameter.*/
		       /*CanIf_TxConfirmation(Controller1_MailBox_Pending_PduId[0];*/
			}
#endif
#if (Controller1_MailBox1==ON)
	  if ((TX1_STATUS & CAN_TSR_TXOK1) != 0U)
	 	    {
		        CAN1_TSR_R=MailBox1_INT_CLEAR_FLAG;/*clearing the interrupt flag*/
		        Controller1_MailBoxes_semaphore[MailBox1]= confirmed;
//		        HTH_To_MailBoxes_Map[Controller0_MailBox1_HTH].Busy_Mailboxes_Num--;
		        /*CanIf_TxConfirmation(Controller1_MailBox_Pending_PduId[1];*/
	 	    }
#endif
#if (Controller1_MailBox2==ON)
	  if ((TX1_STATUS & CAN_TSR_TXOK2) != 0U)
	 	 	{
		        CAN1_TSR_R=MailBox2_INT_CLEAR_FLAG;/*clearing the interrupt flag*/
		        Controller1_MailBoxes_semaphore[MailBox2]= confirmed;
//		        HTH_To_MailBoxes_Map[Controller1_MailBox2_HTH].Busy_Mailboxes_Num--;
		    /*CanIf_TxConfirmation(Controller1_MailBox_Pending_PduId[2];*/
	 	 	}
#endif
}
void Can1_RX0_InterruptHandler(void)
{
	/*[SWS_Can_00395] Can module shall raise the runtime error CAN_E_DATALOST in case of
		      “overwrite” or “overrun” event detection.()*/

uint8_t RX10_STATUS=CAN1_RF0R_R;

if ((RX10_STATUS & CAN_RF0R_FOVR0) != 0U)
{
   CAN1_RF0R_R=CAN_RF0R_FOVR0;
  /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID,   ,CAN_E_DATALOST )*/
  return;
}

#if (Controller1_FIFO_0_Processing==INTERRUPT)
	 PduInfoType received_PduInfo10;
	 Can_HwType Rx_Mailbox10;
	 uint8_t rx1_MsgData0[8U];

if ((RX10_STATUS & CAN_RF0R_FMP0) != 0U)
{

	    Rx_Mailbox10.hoh=Controller1_FIFO_0_HRH;
	    /*[SWS_Can_00423] In case of an Extended CAN frame, the Can module shall convert the ID to a
		  standardised format since the Upper layer (CANIF) does not know whether the received CAN frame
		  is a Standard CAN frame or Extended CAN frame. In case of an Extended CAN frame, MSB of a received
		  CAN frame ID needs to be made as ‘1’ to mark the received CAN frame as Extended.*/
	    Rx_Mailbox10.id=CAN_RI0R_IDE &CAN1_FIFO_0_RIR_R;
	    if (Rx_Mailbox10.id == STANDARD)
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_FIFO_0_HRH].CanIdType==EXTENDED)
					{
	    		       CAN1_RF0R_R|=CAN_RF0R_RFOM0;
					   return;
					}
	    	      else
	    	      {
	    	         Rx_Mailbox10.id = (CAN_RI0R_STID &CAN1_FIFO_0_RIR_R) >> CAN_TI0R_STID_Pos;
	    	      }
	    }
	    else
	    {
	    	if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_FIFO_0_HRH].CanIdType==STANDARD)
					{
	    		       CAN1_RF0R_R|=CAN_RF0R_RFOM0;
					   return;
					}
	    	   else
	    	   {
	    	      Rx_Mailbox10.id = (((CAN_RI0R_EXID | CAN_RI0R_STID) &CAN1_FIFO_0_RIR_R) >> CAN_RI0R_EXID_Pos)|EXT_MASK;
	    	   }
	    }
	    Rx_Mailbox10.controllerlId=Controller1_ID;;

	    /* Get the data */
	    received_PduInfo10.SduLength=(CAN_RDT0R_DLC &CAN0_FIFO_0_RDTR_R)>>CAN_RDT0R_DLC_Pos;
	    received_PduInfo10.SduDataptr=rx1_MsgData0;

	    /*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
	      which is received first is array element 0 the CAN data byte which is received last is array element 7*/
	    received_PduInfo10.SduDataptr[0] = (uint8_t)((CAN_RDL0R_DATA0 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA0_Pos);
	    received_PduInfo10.SduDataptr[1] = (uint8_t)((CAN_RDL0R_DATA1 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA1_Pos);
	    received_PduInfo10.SduDataptr[2] = (uint8_t)((CAN_RDL0R_DATA2 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA2_Pos);
	    received_PduInfo10.SduDataptr[3] = (uint8_t)((CAN_RDL0R_DATA3 &CAN1_FIFO_0_RDLR_R) >> CAN_RDL0R_DATA3_Pos);
	    received_PduInfo10.SduDataptr[4] = (uint8_t)((CAN_RDH0R_DATA4 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
	    received_PduInfo10.SduDataptr[5]=  (uint8_t)((CAN_RDH0R_DATA5 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
	    received_PduInfo10.SduDataptr[6] = (uint8_t)((CAN_RDH0R_DATA6 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
	    received_PduInfo10.SduDataptr[7] = (uint8_t)((CAN_RDH0R_DATA7 &CAN1_FIFO_0_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
	    CAN1_RF0R_R|=CAN_RF0R_RFOM0;/*releasing the FIFO as if clearing the interrupt*/

	    /*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
	  	  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
	  	  parameter PduInfoPtr.*/
	    /*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
		  the function Can_MainFunction_Read in case of polling mode shall call the callback function
		  CanIf_RxIndication.*/
	    /* CanIf_RxIndication(&Rx_Mailbox10,&received_PduInfo10);*/
}
#endif
	    /*[SWS_Can_00395] Can module shall raise the runtime error CAN_E_DATALOST in case of
	      “overwrite” or “overrun” event detection.()*/


}
void Can1_RX1_InterruptHandler(void)
{
	 uint8_t RX11_STATUS=CAN0_RF1R_R;
	  if (( RX11_STATUS& CAN_RF1R_FOVR1) != 0U)
	  {
		  CAN1_RF1R_R=CAN_RF1R_FOVR1;
		  /*Det_ReportError(CAN_MODULE_ID, CAN_INSTANCE_ID,   ,CAN_E_DATALOST )*/
		  return;
	  }
#if (Controller1_FIFO_1_Processing==INTERRUPT)
 PduInfoType received_PduInfo11;
 Can_HwType Rx_Mailbox11;
 uint8_t rx1_MsgData1[8U];
if (RX11_STATUS & CAN_RF1R_FMP1)
{
    Rx_Mailbox11.hoh=Controller1_FIFO_1_HRH;

	    /*[SWS_Can_00423] In case of an Extended CAN frame, the Can module shall convert the ID to a
		  standardised format since the Upper layer (CANIF) does not know whether the received CAN frame
		  is a Standard CAN frame or Extended CAN frame. In case of an Extended CAN frame, MSB of a received
		  CAN frame ID needs to be made as ‘1’ to mark the received CAN frame as Extended.*/
         Rx_Mailbox11.id=CAN_RI0R_IDE &CAN1_FIFO_1_RIR_R;
			if (Rx_Mailbox11.id == STANDARD)
			{
				if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_FIFO_1_HRH].CanIdType==EXTENDED)
				{
					 CAN1_RF1R_R|=CAN_RF1R_RFOM1;
                   return;
				}
				else
				{
					 Rx_Mailbox11.id = (CAN_RI1R_STID &CAN1_FIFO_1_RIR_R ) >> CAN_TI0R_STID_Pos;
				}
			}
			else
			{
				if(CanContainer.CanConfigSet.CanHardwareObject[Controller1_FIFO_1_HRH].CanIdType==STANDARD)
					{
					   CAN1_RF1R_R|=CAN_RF1R_RFOM1;
					   return;
					}
				else{
				     Rx_Mailbox11.id = (((CAN_RI1R_EXID | CAN_RI1R_STID) &CAN0_FIFO_1_RIR_R ) >> CAN_RI0R_EXID_Pos)|EXT_MASK;
				    }
			}

	    Rx_Mailbox11.controllerlId=Controller1_ID;;

	    /* Get the data */
	    received_PduInfo11.SduLength=(CAN_RDT1R_DLC &CAN1_FIFO_1_RDTR_R)>>CAN_RDT0R_DLC_Pos;
	    received_PduInfo11.SduDataptr=rx1_MsgData1;

	    /*[SWS_Can_00060] Data mapping by CAN to memory is defined in a way that the CAN data byte
	      which is received first is array element 0 the CAN data byte which is received last is array element 7*/
	    received_PduInfo11.SduDataptr[0] = (uint8_t)((CAN_RDL0R_DATA1 &CAN1_FIFO_1_RDTR_R) >> CAN_RDL0R_DATA0_Pos);
	    received_PduInfo11.SduDataptr[1] = (uint8_t)((CAN_RDL0R_DATA1 &CAN1_FIFO_1_RDTR_R) >> CAN_RDL0R_DATA1_Pos);
	    received_PduInfo11.SduDataptr[2] = (uint8_t)((CAN_RDL0R_DATA2 &CAN1_FIFO_1_RDTR_R) >> CAN_RDL0R_DATA2_Pos);
	    received_PduInfo11.SduDataptr[3] = (uint8_t)((CAN_RDL0R_DATA3 &CAN1_FIFO_1_RDTR_R) >> CAN_RDL0R_DATA3_Pos);
	    received_PduInfo11.SduDataptr[4] = (uint8_t)((CAN_RDH0R_DATA4 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA4_Pos);
	    received_PduInfo11.SduDataptr[5]=  (uint8_t)((CAN_RDH0R_DATA5 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA5_Pos);
	    received_PduInfo11.SduDataptr[6] = (uint8_t)((CAN_RDH0R_DATA6 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA6_Pos);
	    received_PduInfo11.SduDataptr[7] = (uint8_t)((CAN_RDH0R_DATA7 &CAN1_FIFO_1_RDHR_R) >> CAN_RDH0R_DATA7_Pos);
	    CAN1_RF1R_R|=CAN_RF1R_RFOM1;/*releasing the FIFO as if clearing the interrupt*/
	    /*[SWS_Can_00279] On L-PDU reception, the Can module shall call the RX indication callback function CanIf_RxIndication
	  	  with ID, HOH, abstract CanIf ControllerId in parameter Mailbox, and the Data Length and pointer to the L-SDU buffer in
	  	  parameter PduInfoPtr.*/
	    /*[SWS_Can_00396] The RX-interrupt service routine of the corresponding HW resource or
		  the function Can_MainFunction_Read in case of polling mode shall call the callback function
		  CanIf_RxIndication.*/
	    /* CanIf_RxIndication(&Rx_Mailbox11,&received_PduInfo11);*/
    return;
}
#endif

}





































