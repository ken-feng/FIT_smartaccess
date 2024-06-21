//***************************************************** 
// The KW38 serial head file define 
// Define the main program Ram section  
//***************************************************** 
#include "RNG_Interface.h"
#include "TimersManager.h"
#include "FunctionLib.h"
#include "Messaging.h"
#include "MemManager.h"
#include "GPIO_Adapter.h"
#include "gpio_pins.h"
#include "fsl_gpio.h"
#include "LED.h"
#include "board.h"
#include "ApplMain.h"
#include "NVM_Interface.h"
#include "Reset.h"

#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
#include "Ranger4UciCmd.h"
#include "Ranger4_demo_task.h"
#include "fit_uwb_range_app.h"
#endif

#if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
#include "segger_rtt_utils.h"
#endif
//****************************************************
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#ifdef __HW_TEST_PIN_H
    #define __TEST_PTA_HIGH(x)			GPIO_PinWrite (GPIOA, x, 1)
    #define __TEST_PTA_LOW(x)			GPIO_PinWrite (GPIOA, x, 0)
    #define __TEST_TogglePTA(x)			GPIO_PortToggle (GPIOA, 1<<x)
    #define __TEST_PTB_HIGH(x)			GPIO_PinWrite (GPIOB, x, 1)
    #define __TEST_PTB_LOW(x)			GPIO_PinWrite (GPIOB, x, 0)
    #define __TEST_TogglePTB(x)			GPIO_PortToggle (GPIOB, 1<<x)
    #define __TEST_PTC_HIGH(x)			GPIO_PinWrite (GPIOC, x, 1)
    #define __TEST_PTC_LOW(x)			GPIO_PinWrite (GPIOC, x, 0)
    #define __TEST_TogglePTC(x)			GPIO_PortToggle (GPIOC, 1<<x)
#else
    #define __TEST_PTA_HIGH(x)
    #define __TEST_PTA_LOW(x)
    #define __TEST_TogglePTA(x)
    #define __TEST_PTB_HIGH(x)
    #define __TEST_PTB_LOW(x)
    #define __TEST_TogglePTB(x)
    #define __TEST_PTC_HIGH(x)
    #define __TEST_PTC_LOW(x)
    #define __TEST_TogglePTC(x)
#endif

#define __inRangeDistance 			200

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef enum TimeSyncState_tag
{
    mTSS_NoSynchronization,
    mTSS_NeedsSynchronization,
    mTSS_InSynchronizing,
    mTSS_Synchronized
}TimeSyncState_t;

typedef struct appUwbInfo_tag
{
    uint32_t            RangingSessionId;
    uint64_t            TsUwbLocalVehicleTime;
    uint64_t            TsLocalVehicleEventCount;
    int64_t             TsUwbTimeOffset;
    uint32_t            TsBleEvtDelay;
    uint32_t            TsConnectionIntervalInMicroSecond;
    uint8_t             TsUwbDeviceTimeUncertainty;
    uint16_t            TsRetryDelay;
    TimeSyncState_t     TsTimeSyncState;
} appUwbInfo_t;

typedef struct appUwbTsPeerDevice_tag
{
    uint8_t head;
    uint8_t tail;
    uint8_t maxListSize;
    deviceId_t DeviceIdList[UWB_MAX_QUERY_TIMESTAMP_LIST_SIZE];
}appUwbTsPeerDevice_t;

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
UwbCapabilityManagement_t UwbCapability;                // Capability of UWB should achieve from NCJ29D5 via UCI command.

SessionManagement_t UwbSessions;                        // UWB Ranging Session

uint64_t TsUwbCurrentTimeStamp = 0;                     // for time sync using
uint64_t uwbInitTime0 = 0;

uint16_t AnchorsDistance[4] = {0};                      // for collect all anchor Ranging result, index is from 0-3 as ANCHOR_INDEX
bool_t   IsAnchorRangingFinished[4] = {false};          // flag indicate if test result corresponding ANCHOR_INDEX is collected

appUwbInfo_t maPeerInformation[gAppMaxConnections_c];  // Table with peer devices information

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
//******************************************************************************
// UWB (NCJ29D5)
//******************************************************************************
static appUwbTsPeerDevice_t     TsPeerDeviceList;
//static tmrTimerID_t             mTimeSyncTimerId;
//==============================================================================
/*** Ranger4 core configuration ***/
//==============================================================================
static uint8_t UwbLowPowerMode[1] = {0x00}; //disable low power mode.
static Ranger4Uci_ParameterStructure_t UwbCoreCfg[1] =
{
    {
        .ID = R4_PARAM_LOW_POWER_MODE,
        .length = 0x01,
        .paramValue = UwbLowPowerMode
    }
};

static uint8_t UwbCoreCfgSize = NumberOfElements(UwbCoreCfg);

static Ranger4Uci_DeviceStatusType_t mUwbDeviceStatus = R4_UCI_DEVICE_STATUS_UNDEFINE;

static uint16_t mRanger4Distance = 0;
static uint16_t mRangingDistanceBuff[4] = {0};
static uint16_t mRangingDistanceAvg = 0;
static uint8_t  mRangingCnt = 0;

static bool_t  mIsRanger4Init = false;
static bool_t  mIsRanger4Sleep = false;
static bool_t  mIsRangingSessionInit = false;

static uint16_t mRangingReportDistance[3] = {0};
static bool_t mIsCanReport = false;

//static appUwbInfo_t    appUwbPeerInformation[gAppMaxConnections_c];
//******************************************************************************
//
//******************************************************************************
extern void CAN_set_ranging_report(int uwb_rep,int in_rge,uint16_t dist1, uint16_t dist2, uint16_t dist3);

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_Init
 * Description   : Calls debug console initialization functions
 *
 *
 * END ****************************************************************************************************************/

uint8_t UWB_Api_Start_Ranging(void* pUsrkDst, uint16_t sessio_id)
{
	#if defined __FIT_UWB_RFIC_SYNC_H
	if(mIsRangingSessionInit == false)
	{
        UwbSessions.UwbSessionID = sessio_id;                  // because uwbSession=0x01 is test mode
        Ranger4UciCmd_MacSessionCfgInit(UwbSessions.UwbSessionID, UwbSessions.SessionType);
        mIsRangingSessionInit = true;
        return	0;
	}
	else{
		return	1;
	}
	#endif
}

uint8_t UWB_Api_Stop_Ranging(uint16_t sessio_id)
{
	if(mIsRangingSessionInit == true)
	{
		UwbSessions.UwbSessionID = sessio_id;                  // because uwbSession=0x01 is test mode
        Ranger4UciCmd_MacSessionCfgDeInit(UwbSessions.UwbSessionID);
        UwbSessions.SessionStatus = R4_SESSION_STATE_DEINIT;
//        mIsRangingStart = false;
        mIsRangingSessionInit = false;
        return	0;
	}
	else{
		return	1;
	}
}

void UWB_Api_Reset(void)
{

}

void Set_Ranging_Report(int RangingDistance , int report_bool)
{
	int in_Range_bool = 0;
	mRangingReportDistance[0] = mRangingReportDistance[1];
	mRangingReportDistance[1] = mRangingReportDistance[2];
	mRangingReportDistance[2] = RangingDistance;
	if ((mRangingReportDistance[0]<__inRangeDistance) && (mRangingReportDistance[1]<__inRangeDistance) && (mRangingReportDistance[2]<__inRangeDistance))
	{
		in_Range_bool = 1;
	}
	else
	{
		in_Range_bool = 0;
	}
	CAN_set_ranging_report(report_bool , in_Range_bool , mRangingReportDistance[0],mRangingReportDistance[1],mRangingReportDistance[2]);

}
/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_Init
 * Description   : Calls debug console initialization functions
 * 
 *
 * END ****************************************************************************************************************/
void Init_UWB_Range(void)
{
	//--------------------------------------------------------------------------
	// [ Allcate Timer ] - for UWB
	//--------------------------------------------------------------------------
	//    mTimeSyncTimerId = TMR_AllocateTimer();

	//--------------------------------------------------------------------------
	// [ Initial data ] - for UWB
	//--------------------------------------------------------------------------
	TsPeerDeviceList.head = 0;
	TsPeerDeviceList.tail = 0;
	TsPeerDeviceList.maxListSize = UWB_MAX_QUERY_TIMESTAMP_LIST_SIZE;
	FLib_MemSet(TsPeerDeviceList.DeviceIdList, 0xFF, TsPeerDeviceList.maxListSize);

	//--------------------------------------------------------------------------
	// [ Initial function ]
	//--------------------------------------------------------------------------
	Ranger4App_RegisterCallback(Ranger4App_RspCallback, Ranger4App_NtfCallback);
	Ranger4App_task_Init();
}

/* FUNCTION ************************************************************************************************************
 * \brief        Query Time Stamp from UWB
 *
 * \param[in]    deviceId   0xF0 & valid DeviceId: start ranging after get timestamp
                            valid DeviceId: get timestamp for peer device time sync
 * END ****************************************************************************************************************/
void QueryUwbTimeStamp(deviceId_t deviceId)
{
    uint8_t freeListSize = 0;

    /* record peer device id that needs time sync*/
    if(TsPeerDeviceList.head >= TsPeerDeviceList.tail)
    {
        freeListSize = TsPeerDeviceList.maxListSize - (TsPeerDeviceList.head - TsPeerDeviceList.tail);
    }
    else
    {
        freeListSize = TsPeerDeviceList.tail - TsPeerDeviceList.head;
    }

    if(freeListSize != 0 )
    {
        TsPeerDeviceList.DeviceIdList[TsPeerDeviceList.head] = deviceId;
        TsPeerDeviceList.head++;
        if(TsPeerDeviceList.head >= TsPeerDeviceList.maxListSize)
        {
            TsPeerDeviceList.head = 0;
        }

        /* Send command to get current timestamp from UWB */
        Ranger4UciCmd_ProprietaryQueryTimeStamp();
    }
    else
    {
        /*list is full*/
        panic(0,(uint32_t)&QueryUwbTimeStamp, 0, 0);
    }

}
/* FUNCTION ************************************************************************************************************
 * \brief        App_SetRangingSessionCfg
 *
 * \param[in]

 * END ****************************************************************************************************************/
void App_SetRangingSessionCfg(deviceId_t deviceId, SessionManagement_t * pSessionPara)
{
    uint8_t CfgSize = 0;

//    static uint8_t uwbRxTimeout[3] = {0,0x20, 0x4E};// 20 ms {0, 0x20, 0x4E}; // 40 ms {0, 0x40, 0x9C}; //10ms {0,0x10, 0x27}  //15ms 0,0x98, 0x3A}
//    static uint8_t uwbRxMarginTimeout[3] = {0,0x20, 0x4E}; //20ms
    static uint32_t uwbInitTime = 0;
    static deviceId_t uwbInfoPtr;

    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;
    Ranger4Uci_SessionCfgStructure_t * pRangingSessionCfg;

    pRangingSessionCfg = MEM_BufferAlloc(sizeof(Ranger4Uci_SessionCfgStructure_t) * 30);
    FLib_MemSet((uint8_t *)pRangingSessionCfg, 0, sizeof(Ranger4Uci_SessionCfgStructure_t) * 30);

    //add configurations here
    pRangingSessionCfg[CfgSize].ID = R4_SESSION_UWB_CONFIG_ID;
    pRangingSessionCfg[CfgSize].length = 0x02U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SelectedUwbConfigId;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_PULSESHAPE_COMBO;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SelectedPulseShapeCombo;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_RANGING_INTERVAL;
    pRangingSessionCfg[CfgSize].length = 0x04U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SessionRangingBlockT;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_CHANNEL_ID;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SelectedChannel;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_RANGING_SLOT_LENGTH;
    pRangingSessionCfg[CfgSize].length = 0x02U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SessionChapsperSlot;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_NUMBER_OF_ANCHORS;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->Number_Responders_Nodes;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_SLOTS_PER_RR;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->Number_Slots_per_Round;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_HOPPING_MODE;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SessionHopingMode;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_PREAMBLE_ID;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->Selected_SYNC_Code_Index;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_STS_INDEX0;
    pRangingSessionCfg[CfgSize].length = 0x04U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->STS_Index0;
    CfgSize++;
    //add fixed configurations
    FLib_MemCpy(&pRangingSessionCfg[CfgSize], pSessionPara->pSessionfixedCfg, sizeof(Ranger4Uci_SessionCfgStructure_t) * pSessionPara->SessionSetCfgSize);
    CfgSize += pSessionPara->SessionSetCfgSize;

    //==========================================================================
    // Session UWB initial time
    //==========================================================================
	#if ( ANCHOR_INDEX == 0u )
    /* add UWB init time parameter */
    uwbInfoPtr = 0;
    uwbInitTime0 = (pSessionPara->UWB_Time0 - maPeerInformation[uwbInfoPtr].TsUwbTimeOffset);     // convert UWB_Time0 to local uwb time
//    uwbInitTime0 = uwbInitTime0 + ((uint64_t)1 << (maPeerInformation[uwbInfoPtr].TsUwbDeviceTimeUncertainty >> 3));
	#endif
    //--------------------------------------------------------------------------
    // UWB sync connect after BLE connect
    //--------------------------------------------------------------------------
    if(uwbInitTime0 >= TsUwbCurrentTimeStamp)
    {
        pRangingSessionCfg[CfgSize].ID = R4_SESSION_UWB_INITIATION_TIME;
        pRangingSessionCfg[CfgSize].length = 4;
        uwbInitTime = ((uwbInitTime0 - TsUwbCurrentTimeStamp)/1000);     // convert to millisecond
		#if ( ANCHOR_INDEX == 0u )
        /* Jia: On the phone side, APP can not get correct UWB timestamp corresponding PHY Ind, UWB timestamp only can
           only be retrieved from callback with a Phone system delay, we do not know the exact delay, we set a 225 ms
           as compensation for delay */
        uwbInitTime += 225;     // subtrack 225 millisecond local uwb TimeUncertainty (intital time 0~10000ms; default=0xFFFFFFFF)
		#endif
        pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&uwbInitTime;
        CfgSize++;

//    // Need R4_SESSION_RCM_RX_TIMEOUT
//    pRangingSessionCfg[CfgSize].ID = R4_SESSION_RCM_RX_MARGIN_TIME;
//    pRangingSessionCfg[CfgSize].length = 3;
//    pRangingSessionCfg[CfgSize].paramValue = uwbRxMarginTimeout;
//    CfgSize++;
//
//    // Need R4_SESSION_RCM_RX_TIMEOUT
//    pRangingSessionCfg[CfgSize].ID = R4_SESSION_RCM_RX_TIMEOUT;
//    pRangingSessionCfg[CfgSize].length = 3;
//    pRangingSessionCfg[CfgSize].paramValue = uwbRxTimeout;
//    CfgSize++;
    }
    //--------------------------------------------------------------------------
    // UWB sync connect when no BLE connect
    //--------------------------------------------------------------------------
    else
    {
        /* Local UWB Ranging session start late, start Ranging session immediatly */
        uwbInitTime = 0;
    }

    //==========================================================================
    // Send to NCJ29D5 to AppSessionConfig
    //==========================================================================
    en_Status = Ranger4UciCmd_MacSessionConfigure(pSessionPara->UwbSessionID, CfgSize, pRangingSessionCfg);

    MSG_Free(pRangingSessionCfg);

    //==========================================================================
    // END
    //==========================================================================
    if( en_Status == errorCode_InvalidRange)
    {
//        //too many configurations, split configurations
//        Ranger4UciCmd_MacSessionConfigure(pSessionPara->UwbSessionID, CfgSize/2, pRangingSessionCfg);
//
//        Ranger4UciCmd_MacSessionConfigure(pSessionPara->UwbSessionID, (CfgSize/2 + (CfgSize%2)), &pRangingSessionCfg[CfgSize/2]);
//
    }
    else if(en_Status == errorCode_Undefined)
    {
        //Out of memory
        panic(0,0,0,0);
    }
#if(( ANCHOR_INDEX == 0u ) && ( NUM_ANCHORS > 1 ))
    Flexcan_RangingSessionCfg(gAllSlaveNodes, pSessionPara,uwbInitTime0);
#endif
}


/* FUNCTION ************************************************************************************************************
 * \brief        Process Time Stamp get from UWB
 *
 * \param[in]    pPayload   point of UWB UCI response frame

 * END ****************************************************************************************************************/
void ProcessUwbTimeStamp(uint8_t * pPayload, phscaR4CadsTypesIntf_UciStatusCode_t StateCode)
{
    uint8_t PeerId = 0;
    union
    {
        uint8_t Buf[8];
        uint64_t TsValue;
    }TsUwbTime;

    FLib_MemCpy(TsUwbTime.Buf, &pPayload[0], 8);

    /* Update current timestamp */
    TsUwbCurrentTimeStamp = TsUwbTime.TsValue;

    /* Get nearst Id of device that needs time sync  */
    PeerId  = TsPeerDeviceList.DeviceIdList[TsPeerDeviceList.tail];

    /* Update device list to next device ID */
    TsPeerDeviceList.tail++;
    if(TsPeerDeviceList.tail >= TsPeerDeviceList.maxListSize)
    {
        TsPeerDeviceList.tail = 0;
    }

    if(StateCode != uciStatusCode_Ok)
    {
        QueryUwbTimeStamp(PeerId);
        return;
    }

    if(PeerId < gAppMaxConnections_c)
    {
        /* Subtract ble event delay */
        maPeerInformation[PeerId].TsUwbLocalVehicleTime = TsUwbTime.TsValue - (uint64_t)maPeerInformation[PeerId].TsBleEvtDelay;
#if(( ANCHOR_INDEX == 0u ) && ( NUM_ANCHORS > 1 ))
        /* Send timestamp to slave node*/
        Flexcan_CmdTimeSync(gAllSlaveNodes, TsUwbCurrentTimeStamp);
#endif
    }
    else
    {
        //----------------------------------------------------------------------
        // BLE connect
        //----------------------------------------------------------------------
        if(PeerId != gInvalidDeviceId_c)
        {
            PeerId = PeerId & ~START_RANGING_FLAG;
            if(PeerId < gAppMaxConnections_c)
            {
                App_SetRangingSessionCfg(PeerId, &UwbSessions);
            }
        }
        //----------------------------------------------------------------------
        // No Ble connect
        //----------------------------------------------------------------------
        else
        {
            /* just read UWB timestamp */
            //Modify (Ken):NXP-V0001 NO.12 -20240509
            #ifdef  __FIT_UWB_NoBLERanging_TEST_H
            UwbSessions.UwbSessionID = 0x01;
            Ranger4UciCmd_MacSessionCfgInit(UwbSessions.UwbSessionID, UwbSessions.SessionType);
            #endif
        }
    }

    #if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
    log_debug("\r\n Ranger4App_RspCallback ---> Uwb Current TimeStamp is:");
    log_debug_hexBigend((uint8_t *)&TsUwbCurrentTimeStamp,8);
    log_debug(" \r\n");
    #endif
}

/* FUNCTION ************************************************************************************************************
 * \brief        Extraction capability required by CCC capability exchange process
 *
 * \param[in]    payload   point of UWB UCI response frame

 * END ****************************************************************************************************************/
void GetCapabilityData(uint8_t *payload)
{
    uint8_t i, j, TlvCnt, TlvOffset;

    TlvCnt = payload[1];
    TlvOffset = 2;
    for(i = 0; i < TlvCnt; i++)
    {
        switch (payload[TlvOffset])
        {
            case R4_DEVICE_CAP_SLOT_BITMASK:
            {
                UwbCapability.Slot_bitMask = payload[TlvOffset+2];
            }
            break;

            case R4_DEVICE_CAP_SYNC_CODE_INDEX_BITMASK:
            {
                FLib_MemCpy(UwbCapability.SYNC_Code_Index_Bitmaks, &payload[TlvOffset+2], 4);
            }
            break;

            case R4_DEVICE_CAP_HOPPING_CONFIG_BITMASK:
            {
                UwbCapability.Hopping_Config_Bitmask = payload[TlvOffset+2];
            }
            break;

            case R4_DEVICE_CAP_CHANNEL_BITMASK: //Channel Bitmask
            {
                UwbCapability.Channel_BitMask = payload[TlvOffset+2];
            }
            break;

            case R4_DEVICE_CAP_SUPPORTED_PROTOCOL_VERSION: //Supported Protocol Version
            {
                UwbCapability.ProtoVer_Len = payload[TlvOffset + 1];
                for(j = 0; j < UwbCapability.ProtoVer_Len; j += 2)
                {
                    FLib_MemCpyReverseOrder(&UwbCapability.Supported_Protocol_Version[j],
                                            &payload[TlvOffset + 2 + j],
                                            2);
                }
            }
            break;

            case R4_DEVICE_CAP_SUPPORTED_UWB_CONFIG_ID: //Supported Configure Id
            {
                UwbCapability.CfgId_Len = payload[TlvOffset + 1];
                for(j = 0; j < UwbCapability.CfgId_Len; j += 2)
                {
                    FLib_MemCpy(&UwbCapability.Supported_Cfg_Id[j],
                                &payload[TlvOffset + 2 + j],
                                2);
                }
            }
            break;

            case R4_DEVICE_CAP_SUPPORTED_PULSESHAPE_COMBO: //Supported Pluse shape Combination
            {
                UwbCapability.PluseshapeCombo_Len = payload[TlvOffset + 1];
                FLib_MemCpy(UwbCapability.Supported_Pluseshape_Combo,
                            &payload[TlvOffset + 2],
                            UwbCapability.PluseshapeCombo_Len);
            }
            break;

            default:
                /* Other capability parameters are not used in this demo, ignore  */
            break;
        }

        TlvOffset += payload[TlvOffset + 1];//next TLV start point
        TlvOffset += 2;
    }
}

/* FUNCTION ************************************************************************************************************
 * \brief        UCI response callback function, this function could be rework by
 *               customer application code
 *
 * \param[in]    frame   UCI response frame

 * END ****************************************************************************************************************/
void Ranger4App_RspCallback(phscaR4CadsTypesIntf_UciFrame_t * frame)
{
    phscaR4CadsTypesIntf_UciStatusCode_t StatusCode;
    uint8_t sGroupCode, sOpCode;

    //==========================================================================
    // Get Payload Data
    //==========================================================================
    StatusCode = (phscaR4CadsTypesIntf_UciStatusCode_t)frame->uciPacket.payload[0];
    sGroupCode = frame->uciPacket.header.groupId;
    sOpCode    = frame->uciPacket.header.opcodeId;

    //==========================================================================
    // State Error
    //==========================================================================
//    if(StatusCode != uciStatusCode_Ok){
//        return;
//    }

    //==========================================================================
    // Group Identifier (GID)
    //==========================================================================
    switch(sGroupCode)
    {
        case PHSCA_R4CADSTYPES_UCI_GID_CORE:
        {
            //------------------------------------------------------------------
            // Opcode Identifier (OID)
            //------------------------------------------------------------------
            switch (sOpCode)
            {
                case R4_UCI_CORE_OID_RESET_DEVICE:
                  {
                    if(StatusCode == uciStatusCode_Ok)
                    {
                        for(uint8_t i = 0; i < gAppMaxConnections_c; i++)
                        {
                            maPeerInformation[i].TsTimeSyncState = mTSS_NoSynchronization;
                        }
                    }
                  } break;

                case R4_UCI_CORE_OID_CORE_GET_DEVICE_INFO:
                  {
                  } break;
                case R4_UCI_CORE_OID_CORE_GET_CAPS_INFO:
                  {
                    if(StatusCode == uciStatusCode_Ok)
                    {
                        if( mIsRanger4Init == false )
                        {
                            //set core configuration, to prevent UWB(NCJ29D5D) enter Low power mode
                            //just disable UWB enter low power mode
                            Ranger4UciCmd_MacCoreSetConfiguration(UwbCoreCfgSize, UwbCoreCfg);
                        }
                        /* extract capability that required during CCC capability exchange */
                        GetCapabilityData(frame->uciPacket.payload);
                    }
                  } break;
                case R4_UCI_CORE_OID_CORE_GET_CONFIG:
                  {
                  } break;
                case R4_UCI_CORE_OID_CORE_SET_CONFIG:
                  {
                    if(StatusCode == uciStatusCode_Ok)
                    {
                        /* Set core configuration complete */
                        if( mIsRanger4Init == false )
                        {
                            /* Start UWB timer by calling query UWB timestamp command */
                            QueryUwbTimeStamp(gInvalidDeviceId_c);
                        }

                      //Jia: Just for test UWB without BLE  (Don't setting here because miss receive QueryUwbTimeStamp)
                      #ifdef  __FIT_UWB_NoBLERanging_TEST_H
                      UwbSessions.UwbSessionID = 0x01;
                      Ranger4UciCmd_MacSessionCfgInit(UwbSessions.UwbSessionID, UwbSessions.SessionType);
                      #endif
                    }
                    else
                    {}
                  } break;

                default:
                    break;
            }
        } break;

        case PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG:
        {
            //------------------------------------------------------------------
            // Opcode Identifier (OID)
            //------------------------------------------------------------------
            switch(sOpCode)
            {
                case R4_UCI_SESSION_CFG_OID_INIT://session init complete
                  {
                    if(StatusCode == uciStatusCode_Ok)
                    {
//                            //Jia: Just for test UWB without BLE
//                            Ranger4UciCmd_MacSessionConfigure(UwbSessions.UwbSessionID,
//                                                              UwbSessions.SessionSetCfgSize,
//                                                              UwbSessions.pSessionCfg);
						#if defined __FIT_UWB_RangingSpeed_TEST_H || defined __FIT_UWB_NoBLERanging_TEST_H || defined __FIT_UWB_RFIC_SYNC_H
						App_SetRangingSessionCfg(UwbSessions.UwbSessionID, &UwbSessions);
						#endif
                    }
                    else
                    {}
                  } break;
                case R4_UCI_SESSION_CFG_OID_SET_APP_CFG: //session config complete
                  {
                    if(StatusCode == uciStatusCode_Ok)
                    {
                        Ranger4UciCmd_MacSessionStartRanging(UwbSessions.UwbSessionID);
                        #if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
                        log_debug("\r\n Ranger4App_RspCallback ---> ranging start, session ID is:\r\n");
                        log_debug_hexBigend((uint8_t*)&UwbSessions.UwbSessionID,4);
                        log_debug(" \r\n");
                        #endif
                    }
                    else
                    {

                    }
                  } break;
                default:
                    break;
            }
        } break;

        case PHSCA_R4CADSTYPES_UCI_GID_RANGING_SESSION_CONTROL:
        {
            //------------------------------------------------------------------
            // Opcode Identifier (OID)
            //------------------------------------------------------------------
            switch(sOpCode)
            {
              case R4_UCI_RANGE_CTR_OID_START://session init complete
            	  break;
              case R4_UCI_RANGE_CTR_OID_STOP: //session config complete
            	  break;
              default:
                  break;
            }
        } break;

        case PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN:
        {
            //------------------------------------------------------------------
            // Opcode Identifier (OID)
            //------------------------------------------------------------------
            switch(sOpCode)
            {
                case R4_UCI_PROPRIETARY_OID_QUERY_UWB_TIMESTAMP:
                {
                    if(StatusCode == uciStatusCode_Ok)
                    {
                        if( mIsRanger4Init == false )
                        {
                            mIsRanger4Init = true;
                        }
                    }

                    ProcessUwbTimeStamp(&frame->uciPacket.payload[1], StatusCode);
                } break;

                //Modify (Ken):NXP-V0001 NO.5 -20240319
                case R4_UCI_PROPRIETARY_OID_DEVICE_SUSPEND:
                {
                    if(StatusCode == uciStatusCode_Ok)
                    {
                       if( mIsRanger4Sleep == false )
                       {
                          mIsRanger4Sleep = true;
                       }
                    }
                } break;

                default:
                  break;
            }
        } break;

        default:
            break;
    }
}

/* FUNCTION ************************************************************************************************************
 * \brief        UCI notification callback function, this function could be rework by
 *               customer application code
 *
 * \param[in]    frame   UCI response frame

 * END ****************************************************************************************************************/
void Ranger4App_NtfCallback(phscaR4CadsTypesIntf_UciFrame_t * frame)
{
#define __InRangeDistance       200             // unit: cm
#define __OutRangeDistance      500             // unit: cm
#define __ToleranceDistance      10             // unit: cm

    #if (NUM_ANCHORS >= 1)
    uint32_t sessionId ;
    #endif
    Ranger4Uci_SessionStatusType_t	SessionSTA_Tmp;

    //==========================================================================
    // Group Identifier (GID)
    //==========================================================================
    switch (frame->uciPacket.header.groupId)
    {
        case PHSCA_R4CADSTYPES_UCI_GID_CORE:
        {
            //------------------------------------------------------------------
            // Opcode Identifier (OID)
            //------------------------------------------------------------------
            if(frame->uciPacket.header.opcodeId == R4_UCI_CORE_OID_CORE_DEVICE_STATUS_NTF)
            {
                //This state is the first state of UWBS after power ON/Reset or on receipt of DEVICE_RESET_CMD from host.
                mUwbDeviceStatus = (Ranger4Uci_DeviceStatusType_t)frame->uciPacket.payload[0];
                if(mUwbDeviceStatus >= R4_UCI_DEVICE_STATUS_NOT_SUPPORT)
                {
                    /* When the device type is not supported, only CORE_GET_DEVICE_INFO_CMD and CORE_RESET_DEVICE_CMD are
                       supported, it need to check if UWB device support UCI in hardware level*/
                    #if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
                    log_debug("\r\n Ranger4App_NtfCallback ---> Device respone not support UCI, please check UWB device firmware. \r\n");
                    #endif
                    panic(0, (uint32_t)Ranger4App_NtfCallback, mUwbDeviceStatus, 0);
                }
                else
                {
                    if(mUwbDeviceStatus == R4_UCI_DEVICE_STATUS_READY)
                    {
                        if(mIsRanger4Init == false)
                        {
                            /* UWB status is ready after ranger4 initialized, then read capability from UWB */
                            Ranger4UciCmd_MacCoreGetCapsInfo();

                            /* Session variable init*/
                            Ranger4App_InitDefaultSessionCfg(&UwbSessions);
                            UwbSessions.SessionStatus = R4_SESSION_STATE_DEINIT;
                        }
                        else
                        {
                            /* UWB status is ready, on other case */
                        }
                    }
                }
            }
            //Modify (Ken):NXP-V0001 NO.3 -20240124
            else if(frame->uciPacket.header.opcodeId == R4_UCI_CORE_OID_CORE_GENERIC_ERROR_NTF)
            {
                /* This Notification is used in error situations when the error cannot
                   be notified using an error status in a Response Message*/
            }
        } break;

        case PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG:
        {
            //------------------------------------------------------------------
            // Opcode Identifier (OID)
            //------------------------------------------------------------------
            if(frame->uciPacket.header.opcodeId == R4_UCI_SESSION_CFG_OID_STATUS_NTF)
            {
            	SessionSTA_Tmp = UwbSessions.SessionStatus;

                UwbSessions.UwbSessionID  = (uint32_t)frame->uciPacket.payload[0];
                UwbSessions.UwbSessionID |= ((uint32_t)frame->uciPacket.payload[1]) << 8;
                UwbSessions.UwbSessionID |= ((uint32_t)frame->uciPacket.payload[2]) << 16;
                UwbSessions.UwbSessionID |= ((uint32_t)frame->uciPacket.payload[3]) << 24;
                UwbSessions.SessionStatus = frame->uciPacket.payload[4];
                UwbSessions.SessionStatusChangeReason = frame->uciPacket.payload[5];

                //--------------------------------------------------------------
                // Start Range to Idle mode : deInit session
                //--------------------------------------------------------------
                if(SessionSTA_Tmp == R4_SESSION_STATE_ACTIVE && UwbSessions.SessionStatus==R4_SESSION_STATE_IDLE)
                {
                	UWB_Api_Stop_Ranging(UwbSessions.UwbSessionID);
                }
            }
        } break;

        case PHSCA_R4CADSTYPES_UCI_GID_RANGING_SESSION_CONTROL:
        {
            //------------------------------------------------------------------
            // Opcode Identifier (OID)
            //------------------------------------------------------------------
             if(frame->uciPacket.header.opcodeId == R4_UCI_RANGE_CTR_OID_CCC_DATA_NTF)
             {
                 mRanger4Distance = (uint16_t)frame->uciPacket.payload[11] | ((uint16_t)frame->uciPacket.payload[12] << 8);

                 //-------------------------------------------------------------
                 // Anchor 0
                 //-------------------------------------------------------------
                 #if (ANCHOR_INDEX == 0)
                 #if (NUM_ANCHORS > 1)
                 AnchorsDistance[ANCHOR_INDEX] = mRanger4Distance;
                 IsAnchorRangingFinished[ANCHOR_INDEX] = true;

                 /* Local ranging result is collect in first time, because other slave anchor ranging result will be
                   transmitted to this master anchor via CAN bus */
                 for(uint8_t i = 1; i < NUM_ANCHORS; i++)
                 {
                     IsAnchorRangingFinished[i] = false;
                 }
                 #elif defined SEND_RANGING_RESULT_TO_DEVICE
                 sessionId = *((uint32_t *)&frame->uciPacket.payload[0]);
                 BleApp_SendUWBRangingResult(sessionId, mRanger4Distance);
                 #endif
                 //-------------------------------------------------------------
                 // Other Anchor
                 //-------------------------------------------------------------
                 #else
                 Flexcan_SendRangingResultToMainNode(mRanger4Distance);
                 #endif
                 //-------------------------------------------------------------
                 // Distance Algorithm
                 //-------------------------------------------------------------
            	 __TEST_TogglePTB(2);
                 if(mRanger4Distance)
                 {
                	 mIsCanReport = true;	// start report for canbus
                     #if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
                     log_debug("Ranger4App_NtfCallback ---> Distance is: %d cm \r\n", mRanger4Distance);
                     #endif
                     mRangingDistanceBuff[mRangingCnt] = mRanger4Distance;
                     mRangingCnt ++;
                     if(mRangingCnt > 3)
                     {
                         mRangingCnt = 0;
                         mRangingDistanceAvg = mRangingDistanceBuff[0] + mRangingDistanceBuff[1] + mRangingDistanceBuff[2] + mRangingDistanceBuff[3];
                         mRangingDistanceAvg = (mRangingDistanceAvg >> 2);
                     }

                     if(mRangingDistanceAvg < __InRangeDistance)
                     {
                           //do something
                     }
//                     else if((mRangingDistanceAvg >= 200) &&(mRangingDistanceAvg < 230))
//                     {
//                         //do nothing
//                     }
                     else if((mRangingDistanceAvg >= __InRangeDistance+__ToleranceDistance) &&(mRangingDistanceAvg < __OutRangeDistance))
                     {
                          //do something
                     }
                     else
                     {
                          //do something
                     }
                 }
                 else
                 {
                	 __TEST_TogglePTB(3);
                	 mIsCanReport = false;	// cancel report for canbus
                     #if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
                     log_debug("\r\n Ranger4App_NtfCallback ---> UnSync \r\n");
                     #endif
                 }
             }
             //------------------------------------------------------------------
             //
             //------------------------------------------------------------------
             Set_Ranging_Report(mRanger4Distance , mIsCanReport);
        } break;

        case PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN:
        {
            //------------------------------------------------------------------
            // Opcode Identifier (OID)
            //------------------------------------------------------------------
            if(frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_LOG_NTF)
            {
            }
            else if(frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_TEST_STOP_NTF)
            {
            }
            else if(frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_TEST_LOOPBACK_NTF)
            {
            }
            else if(frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_SET_TRIM_VALUE_NTF)
            {
            }
        } break;

        default:
             /*JIA: For other group notification, not sure how to handle it, just ignore it */
             break;

    }
}


/* FUNCTION ************************************************************************************************************
 *
 * Function Name :
 * Description   :
 * 
 *
 * END ****************************************************************************************************************/


#endif




