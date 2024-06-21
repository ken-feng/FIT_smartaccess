//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
/*! *********************************************************************************
* Include
********************************************************************************** */
#include "fsl_common.h"
#include "fsl_os_abstraction.h"
#include "Panic.h"
//#include "SerialManager.h"
#include "MemManager.h"
#include "Messaging.h"
#include "FunctionLib.h"
#include "ble_general.h"
#include "LED.h"

#include "phscaR4DriverHw.h"
//#include "phscaR4_RCICmd.h"
#include "phscaR4UciSpi.h"
#include "Ranger4UciCmd.h"
#include "Ranger4_demo_task.h" 

//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
#include "segger_rtt_utils.h"
#endif
/*! *********************************************************************************
* Private macros
********************************************************************************** */
#define R4SWUP_UART_CMD_START_SWUP        0xF1
#define R4SWUP_UART_CMD_GET_NEW_BLOCK     0xF2
#define R4SWUP_UART_CMD_TRANSFER_COMPLETE 0xF3
#define R4SWUP_UART_CMD_STOP_TRANSFER     0xF4
#define R4SWUP_UART_CMD_ERROR_NOTIFY      0xF5

#define R4SWUP_UART_CMDTYPE_REQUEST       0x00
#define R4SWUP_UART_CMDTYPE_COMFIRM       0x01
#define R4SWUP_UART_CMDTYPE_INDECATION    R4SWUP_UART_CMDTYPE_REQUEST




#if 0 //Jia
#define R4SWUP_APP_STATE_NO_ERROR             0x00000000u
#define R4SWUP_APP_STATE_RESET_FAILED         0x10000001u
#define R4SWUP_APP_STATE_PARAM_ERROR          0x10000002u
#define R4SWUP_APP_STATE_PROCESS_STARTED      0x10000003u
#define R4SWUP_APP_STATE_SWUP_ERROR           0x10000004u
#define R4SWUP_APP_STATE_ACTIVATESWUP_FAILED  0x10000005u
#endif

/*! *********************************************************************************
* Private function declarations
********************************************************************************** */
static void Ranger4App_task
(
    osaTaskParam_t param
);
static void Ranger4App_RunningHandler(void);
//static void Ranger4App_DeviceSWUPHandler(void);
static void Ranger4App_Interrupt (void *pParam);
static void Ranger4App_RcvDateFromR4Handle(phscaR4CadsTypesIntf_UciFrame_t * frame);

#if 0 //Jia
static void R4Swup_ErrorNotify(deviceId_t deviceId, uint32_t error_state);
static void R4Swup_StartConfirm(deviceId_t deviceId);
static void R4Swup_GetNewPacket(deviceId_t deviceId);
static void R4Swup_TransferComplete(deviceId_t deviceId);
static void R4SWUP_UwbDeviceStateMachineHandler(deviceId_t deviceId, R4Swup_SerialCmdPacket_t * pPacket);
static void R4Swup_StopSwupProcess(deviceId_t deviceId);
static void R4SWUP_ProcessRxPkt(R4Swup_SerialCmdPacket_t * pPacket);
static bool R4Swup_SendPktToSerial(uint32_t SerMgrIf, R4Swup_SerialCmdPacket_t *pPacket);
static bool R4Swup_SendCmdToSerial(uint32_t SerMgrIf, uint8_t cmd, uint8_t * data);
static void R4Swup_CollectComponectInfo(R4Swup_UpdateImgStruct_t * ImageInfo, R4Swup_SerialCmdPacket_t * pPacket);
#endif
/*! *********************************************************************************
* Private memory declarations
********************************************************************************** */
#if 0 //Jia
static anchor_t mR4SwupRcvPktInputMsgQueue;
static deviceId_t R4SwupDeviceId = 0xFF;
static bool R4Swup_ProcessOnGoing = false;
static R4Swup_SerialComm_t mR4SwupUartComm; 
static R4Swup_UpdateImgStruct_t mR4SwupUpdataImg;
static R4Swup_SpiProtocolType_t R4Swup_SpiProtocolType = R4SWUP_SPI_PROTOCOL_RCI;
//static anchor_t                     mRanger4SwupRcvPktInputMsgQueue;
//static Ranger4AppRunning_Status_t   mRanger4App_RunningStatus = Ranger4AppRunning_Init;
#endif


static bool                          mRanger4App_SwupRequire = false;
static osaEventFlags_t               mRanger4App_event = 0U;
static Ranger4App_Status_t           mRanger4App_Status = Ranger4App_Init;
static Ranger4HandleUciRcvCallback_t handleUciNtfCallback= NULL;
static Ranger4HandleUciRcvCallback_t handleUciRspCallback= NULL;

//Ranger4Uci_SessionStatus_t mUwbSessionStatus;

/*! *********************************************************************************
* Public memory declarations
********************************************************************************** */
OSA_TASK_DEFINE(Ranger4App_task, gRanger4TaskPriority_c, 1, gRanger4TaskStackSize_c, FALSE );
osaEventId_t   mRanger4ThreadEventId;
osaTaskId_t    mRanger4ThreadId; 
Ranger4UciCmd_TransferStateManagement_t mRanger4TransSta;
anchor_t                     mRanger4SendPktMsgQueue;
anchor_t                     mRanger4RcvPktMsgQueue;
phscaR4CadsTypesIntf_UciFrame_t  m_Ranger4UciRcvBuffer;

extern uint8_t gAppSerMgrIf;
extern Ranger4Uci_SessionCfgStructure_t UwbSessionCfg[__InitSessionCfg_Size];
/*! *********************************************************************************
* Public functions
********************************************************************************** */
void Ranger4App_InitDefaultSessionCfg(SessionManagement_t * pSessionCfg)
{
    pSessionCfg->UwbSessionID = 0x01;
    pSessionCfg->pSessionfixedCfg = UwbSessionCfg;
    pSessionCfg->SessionSetCfgSize = NumberOfElements(UwbSessionCfg);
    pSessionCfg->SessionType = R4_SESSION_CCC_RANGING_SESSION;
    pSessionCfg->RAN_Multiplier = 1;
    pSessionCfg->Number_Chaps_per_Slot = 3;
    pSessionCfg->Selected_Hopping_Config_Bitmask = 0x80;
    
    pSessionCfg->SelectedDkProtolVersion[0] = 0x01;
    pSessionCfg->SelectedDkProtolVersion[1] = 0x00;
    
    pSessionCfg->SelectedUwbConfigId  = 0x0000;
    
    pSessionCfg->SelectedPulseShapeCombo = 0x11;
    
    pSessionCfg->SessionRangingBlockT = pSessionCfg->RAN_Multiplier * CCC_MIN_BLOCK;
    
    pSessionCfg->SelectedChannel = 0x09;

    pSessionCfg->SessionChapsperSlot = pSessionCfg->Number_Chaps_per_Slot * CCC_TCHAP;

    //Modify (Ken):NXP-V0001 NO.5 -20240319
    #ifdef  __UWB_ROLE_RESPONDER_H
    pSessionCfg->Number_Responders_Nodes = NUM_ANCHORS;         //Responder
    #else
    pSessionCfg->Number_Responders_Nodes = 0;                   //Initiator
    #endif

    pSessionCfg->Number_Slots_per_Round = 12; 
    
    /* because we don't use hopping mode, so just set to no hopping mode */     
    pSessionCfg->SessionHopingMode = 0x00;    
    
    pSessionCfg->Selected_SYNC_Code_Index = 0x0A;

    pSessionCfg->STS_Index0 = 0x00000000;
    
    pSessionCfg->UWB_Time0 = 0;
}


//******************************************************************************
// Setting Callback function for Notification and Response from NCJ29D5
//******************************************************************************
void Ranger4App_RegisterCallback(Ranger4HandleUciRcvCallback_t  RspCallback,
                                 Ranger4HandleUciRcvCallback_t  NtfCallback)
{
    handleUciRspCallback = RspCallback;
    handleUciNtfCallback = NtfCallback;
}
//******************************************************************************


//******************************************************************************
// Range Task Initialize
//******************************************************************************
/*! -------------------------------------------------------------------------
 * \brief initialize the Ranger4 task
 *---------------------------------------------------------------------------*/
void Ranger4App_task_Init(void)
{
    static uint8_t R4Task_initialized = FALSE;

    if( !R4Task_initialized )
    {
        //Modify (Ken):NXP-V0001 NO.3 -20240319
        #if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
        log_debug("\r\n UWB task init.\r\n");        
        #endif
        //======================================================================
        /* Initialization of hardware*/
        //======================================================================
        (void)phscaR4UciSpi_InterfaceInit();
        phscaR4Driver_SetINTCallBack(Ranger4App_Interrupt, NULL);
        
        //======================================================================
        //First time runing this task
        //======================================================================
        mRanger4App_Status = Ranger4App_Init;
        //======================================================================
        // Only use for receive cmd over UART
        //======================================================================
//        mR4SwupUartComm.bytesReceived = 0; //Jia
//        mR4SwupUartComm.bytesExpect = 0;
        //======================================================================
        /* Prepare callback input queue.*/
        //======================================================================
//        MSG_InitQueue(&mRanger4SwupRcvPktInputMsgQueue);       //Jia
        
        MSG_InitQueue(&mRanger4SendPktMsgQueue);
        MSG_InitQueue(&mRanger4RcvPktMsgQueue);
        
        //======================================================================
        /* Event*/
        //======================================================================
        mRanger4ThreadEventId = OSA_EventCreate(TRUE);
        
        if( NULL == mRanger4ThreadEventId )
        {
            panic( 0, (uint32_t)Ranger4App_task_Init, 0, 0 );
        }
        else
        {
            //------------------------------------------------------------------
            /* Creat task*/
            //------------------------------------------------------------------
            mRanger4ThreadId = OSA_TaskCreate(OSA_TASK(Ranger4App_task), NULL);
            
            if( NULL == mRanger4ThreadId )
            {
                panic( 0, (uint32_t)Ranger4App_task_Init, 0, 0 );
            }
        }
        
        //======================================================================
        /* Finish */
        //======================================================================
        R4Task_initialized = TRUE;
        
//        Ranger4App_RegisterCallback(Ranger4App_RspCallback, Ranger4App_NtfCallback);
        //at first time, notify task in reset device status
//        (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c); //Jia
    }
}
//******************************************************************************


//******************************************************************************
// Range Task Process
//******************************************************************************
static void Ranger4App_task
(
    osaTaskParam_t param
)
{ 
    while(1)
    {
        switch (mRanger4App_Status)
        {
            case Ranger4App_Init:
            {
                mRanger4TransSta.state = TransferState_Idle;
                /* Switch to next state */
                mRanger4App_Status = Ranger4App_HardwareResetDevice;
                break;
            }
        
            case Ranger4App_HardwareResetDevice:
            {
                /* Reset device by pin*/
                phscaR4UciSpi_HardwareResetRanger4();
                /* Switch to running state*/
                if(mRanger4App_SwupRequire)
                {
                    mRanger4App_Status = Ranger4App_DeviceSWUP;
                }
                else
                {
                    mRanger4App_Status = Ranger4App_Running;
                }
                break;
            }
        
            case Ranger4App_Running:
            {
                Ranger4App_RunningHandler();
                break;
            }
//            case Ranger4App_DeviceSWUP:
//            {
//                Ranger4App_DeviceSWUPHandler();
//                break;
//            }        
            case Ranger4App_UciCommunicationError:
            {
                break;
            }
        
            case Ranger4App_CriticalError:
            {
                break;
            }
            case Ranger4App_UndefinedError:
            {
                break;
            }
            default:
            {
                /* Do nothing */
                break;
            }  
        }
    }
}
//******************************************************************************


//******************************************************************************
// Ranging Evet Proccess
//******************************************************************************
static void Ranger4App_RunningHandler(void)
{  
    phscaR4CadsTypesIntf_ErrorCode_t error = errorCode_Success;
    phscaR4CadsTypesIntf_UciFrame_t *pMsg;
    
    (void)OSA_EventWait(mRanger4ThreadEventId, osaEventFlagsAll_c, FALSE, osaWaitForever_c, &mRanger4App_event);

    //==========================================================================
    // wait receive response after send data in Send Sync
    //==========================================================================
    if(mRanger4App_event & gR4EvtUwb2HostMsgReceivedEvent_c)
    {
        pMsg = MSG_DeQueue(&mRanger4RcvPktMsgQueue);
        /* process received data */
        Ranger4App_RcvDateFromR4Handle(pMsg);
        
        (void)MSG_Free(pMsg);
    }
    
    //==========================================================================
    // when interrupt from ncj29d5
    //==========================================================================
    if(mRanger4App_event & gR4EvtUwb2HostMsgAvailable_c)
    {
        /* Receive data over UCI */
        error = phscaR4UciSpi_ReceiveData(&m_Ranger4UciRcvBuffer);
        if(error == errorCode_Success)
        {
            /* process receive data */
            Ranger4App_RcvDateFromR4Handle(&m_Ranger4UciRcvBuffer);
        }
        else if((error == errorCode_UciNoEvent))//Jia need compare with CADS
        {
            /* Do nothing, notification may already read in other branch */
        }
        else
        {}
    }
    
    //==========================================================================
    // Transfer packet to ncj29d5
    //==========================================================================
    if(mRanger4App_event & gR4EvtTransferCmdToR4Event_c)
    {
        if(MSG_Pending(&mRanger4SendPktMsgQueue) && (mRanger4TransSta.state == TransferState_Idle))
        {
            pMsg = MSG_DeQueue(&mRanger4SendPktMsgQueue);
            error = phscaR4UciSpi_TransferData((phscaR4CadsTypesIntf_UciFrame_t *)pMsg, &m_Ranger4UciRcvBuffer); 

            //------------------------------------------------------------------
            // Command Send complete
            //------------------------------------------------------------------
            if(error == errorCode_UciSendSuccess )
            { 
                mRanger4TransSta.state = TransferState_Busy;
                /* Send success, save sent packet data. */
                memcpy(mRanger4TransSta.PreviousUciFrame.bytes, pMsg->bytes, pMsg->uciPacket.header.payloadLength + 4);
                /* Messages must be freed*/
                (void)MSG_Free(pMsg);
            }
            //------------------------------------------------------------------
            // Command message not send
            //------------------------------------------------------------------
            else
            {
                if(error == errorCode_UciRecvSuccess)
                {
                    //Process received data
                    Ranger4App_RcvDateFromR4Handle(&m_Ranger4UciRcvBuffer);
                    //Command message not send, not need to free Command msg, we need to  
                    //send it in next loop, and it in queue head, wait for next send
                    (void)MSG_QueueHead(&mRanger4SendPktMsgQueue,pMsg);
                    //Re-call event to send pkg
                    (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtTransferCmdToR4Event_c);
                }
                else if(error == errorCode_UciR4NotReadyError)
                {
                    //Command message not send, not need to free Command msg, we need to  
                    //send it in next loop, and it in queue head, wait for next send
                    (void)MSG_QueueHead(&mRanger4SendPktMsgQueue,pMsg);
                    //Re-call event to send pkg
                    (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtTransferCmdToR4Event_c);
                }
                else
                {
                    //errorCode_UciGenericError
                    mRanger4App_Status = Ranger4App_UndefinedError;
                    /*Send success, Messages must be freed. */
                    (void)MSG_Free(pMsg);
                }
            }
        }
    }
    
    //==========================================================================
    // check Send packet queue
    //==========================================================================
    if(MSG_Pending(&mRanger4SendPktMsgQueue))
    {
        (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtTransferCmdToR4Event_c);
    }
}
//******************************************************************************


//******************************************************************************
// 
//******************************************************************************
static void Ranger4App_RcvDateFromR4Handle(phscaR4CadsTypesIntf_UciFrame_t * frame)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt;
    
    //==========================================================================
    // CCC Notification
    //==========================================================================
    if(frame->uciPacket.header.msgType == PHSCA_R4CADSTYPES_UCI_MT_CMD_NTF)
    {
        if(handleUciNtfCallback != NULL)
        {
            handleUciNtfCallback(frame);
        }
    }
    //==========================================================================
    // CCC Response
    //==========================================================================
    else if (frame->uciPacket.header.msgType == PHSCA_R4CADSTYPES_UCI_MT_CMD_RSP)
    {
        /* Both GID and OID received is same as previous send CMD, means response corresponding command */
        if((mRanger4TransSta.PreviousUciFrame.uciPacket.header.groupId == frame->uciPacket.header.groupId) 
           && (mRanger4TransSta.PreviousUciFrame.uciPacket.header.opcodeId == frame->uciPacket.header.opcodeId))
        {
            if(frame->uciPacket.payload[0] == uciStatusCode_Command_Retry)
            {
                //UWB request retransmission UCI frame, add previous frame in queue head
                pUciCmdPkt = MEM_BufferAlloc(PHSCAR4_UCI_HEADER_LENGTH + mRanger4TransSta.PreviousUciFrame.uciPacket.header.payloadLength);
                memcpy(pUciCmdPkt, mRanger4TransSta.PreviousUciFrame.bytes, (PHSCAR4_UCI_HEADER_LENGTH + mRanger4TransSta.PreviousUciFrame.uciPacket.header.payloadLength));
                (void)MSG_QueueHead(&mRanger4SendPktMsgQueue,pUciCmdPkt);
                //Re-call event to send pkg
                (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtTransferCmdToR4Event_c);
            }
            else
            {
                /*Received response, then next command can be send */
                mRanger4TransSta.state = TransferState_Idle;
                if(handleUciRspCallback != NULL)
                {
                    handleUciRspCallback(frame);
                }
            }
        }
        else
        {
            /* For R4Pkt_SendSync */
            if((frame->uciPacket.header.groupId == PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN) 
               && (frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_QUERY_UWB_TIMESTAMP))
            {
                if(handleUciRspCallback != NULL)
                {
                    handleUciRspCallback(frame);
                }
            }
        }
    }
    //==========================================================================
    //
    //==========================================================================
    else
    {
        /*  Other msg type should not be receive on device, if it received, ignore that*/
    }
}
//******************************************************************************


//******************************************************************************
// GPIO INT Interrupt from NCJ29D5
//******************************************************************************
static void Ranger4App_Interrupt (void *pParam)
{
    (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtUwb2HostMsgAvailable_c);
}
//******************************************************************************


//******************************************************************************
// reserved/unused
//******************************************************************************
//static void Ranger4App_DeviceSWUPHandler(void)
//{
//    
//}


#if 0 //Jia

static void Ranger4_task
(
    osaTaskParam_t param
)
{ 
    R4Swup_PacketRcvStatus_t checkPktSta;
    osaEventFlags_t event = 0U;
    while(1)
    {
        (void)OSA_EventWait(mRanger4ThreadEventId, osaEventFlagsAll_c, FALSE, osaWaitForever_c, &event);
        
        if(event & gR4EvtNewPacketFromUart_c)
        {
            /* Check for existing messages in queue */
            if (MSG_Pending(&mR4SwupRcvPktInputMsgQueue))
            {
                /* Pointer for storing the callback messages. */
                R4Swup_SerialCmdPacket_t *pMsgIn = MSG_DeQueue(&mR4SwupRcvPktInputMsgQueue);
                if (pMsgIn != NULL)
                {
                    //deal with Rx packet
                    R4SWUP_ProcessRxPkt(pMsgIn);
                        
                    /* Messages must always be freed. */
                    (void)MSG_Free(pMsgIn);
                }
            }
        }
        if((event & gR4EvtMsgPacketFromSPI_c) || (event & gR4EvtNewPacketMsgFromBLE_c))
        {
            /* Check for existing messages in queue */
            if (MSG_Pending(&mR4SwupRcvPktInputMsgQueue))
            {
                /* Pointer for storing the callback messages. */
                R4Swup_SerialCmdPacket_t *pMsgIn = MSG_DeQueue(&mR4SwupRcvPktInputMsgQueue);
                if (pMsgIn != NULL)
                {
                    uint16_t len;
                    len = (uint16_t)pMsgIn->raw_data[1] | ((uint16_t)pMsgIn->raw_data[2] << 8);
                    checkPktSta = R4SWUP_checkPacket(&mR4SwupUartComm, & pMsgIn->raw_data[0], (len + 3));
                    if(checkPktSta == Packet_Rcv_Ok)
                    {    
                        mR4SwupUartComm.bytesReceived = 0;
                        mR4SwupUartComm.bytesExpect = 0;
                        //deal with Rx packet
                        R4SWUP_ProcessRxPkt(pMsgIn);
                    }
                    else
                    {
                        mR4SwupUartComm.bytesReceived = 0;
                        mR4SwupUartComm.bytesExpect = 0;
                    }
                        
                    /* Messages must always be freed. */
                    (void)MSG_Free(pMsgIn);
                }
            }
        }     
        if(event & gR4EvtSelfNotify_c)
        {
            R4SWUP_UwbDeviceStateMachineHandler(R4SwupDeviceId, NULL);
        }
        
        event = MSG_Pending(&mR4SwupRcvPktInputMsgQueue) ? gR4EvtNewPacketFromUart_c : 0U;
        if (event != 0U)
        {
            (void)OSA_EventSet(mRanger4ThreadEventId, gR4EvtNewPacketFromUart_c);
        }
    }
}


/*! -------------------------------------------------------------------------
 * \brief start SWUP process command handler
 * \param[in]    deviceId   connected BLE device Id, if device not connected, 
 *                          then device id should 0xFF, only use for BLE to SPI.
 * \param[in]    pPacket    received command pkt from serial(UART/SPI) or BLE.
 *---------------------------------------------------------------------------*/
void R4SWUP_HandleStartSwupProcessCmd(deviceId_t deviceId, R4Swup_SerialCmdPacket_t * pPacket)
{
    if(R4Swup_ProcessOnGoing)
    {
        /* device has started SWUP process, could not process again */
        R4Swup_ErrorNotify(deviceId, R4SWUP_APP_STATE_PROCESS_STARTED);
    }
    else
    {
        uint16_t NumOfPkt = 0;
        //get total packets count 
        NumOfPkt = (uint16_t)pPacket->pkt.payload[0] | ((uint16_t)pPacket->pkt.payload[1] << 8); 
        //default size of pkt 
        mR4SwupUpdataImg.PacketdataSize = SWUP_PKT_SIZE;
        //get Image total size, calculate total packet that should get from uart and send over SPI
        mR4SwupUpdataImg.ImageSize = (uint32_t)pPacket->pkt.payload[2] | ((uint32_t)pPacket->pkt.payload[3] << 8) |
                                     ((uint32_t)pPacket->pkt.payload[4] << 16) | ((uint32_t)pPacket->pkt.payload[5] << 24);
                    
        mR4SwupUpdataImg.totalpackets = (mR4SwupUpdataImg.ImageSize + mR4SwupUpdataImg.PacketdataSize - 1) / mR4SwupUpdataImg.PacketdataSize;
        
        if( mR4SwupUpdataImg.totalpackets != NumOfPkt )
        {
            R4Swup_ErrorNotify(deviceId, R4SWUP_APP_STATE_PARAM_ERROR);
        }
        else
        {
            R4SwupDeviceId = deviceId;
            /* Only when in idle will start swup process, */
            R4SWUP_UwbDeviceStateMachineHandler(deviceId, pPacket);
            (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c);
        }
    }
}

/*! -------------------------------------------------------------------------
 * \brief Image Block response command handler
 * \param[in]    deviceId   connected BLE device Id, if device not connected, 
 *                          then device id should 0xFF, only use for BLE to SPI.
 * \param[in]    pPacket    received command pkt from serial(UART/SPI) or BLE.
 *---------------------------------------------------------------------------*/
void R4SWUP_HandleImageBlockResponseCmd(deviceId_t deviceId, R4Swup_SerialCmdPacket_t * pPacket)
{
    R4SWUP_UwbDeviceStateMachineHandler(deviceId, pPacket);
}

/*! -------------------------------------------------------------------------
 * \brief Stop process  command handler
 * \param[in]    deviceId   connected BLE device Id, if device not connected, 
 *                         then device id should 0xFF, only use for BLE to SPI.
 *---------------------------------------------------------------------------*/
void R4SWUP_HandleStopProcessCmd(deviceId_t deviceId)
{
    switch(mR4DemoSwupAppStatus)
    {
        case R4_SWUP_ResetDevice: /* UWB device not activate SWUP*/     
        case R4_SWUP_Idle:        
        case R4_SWUP_StartProcess:
        {
        }
        break;
        
        /* UWB device activate SWUP Complete*/ 
        case R4_SWUP_ActivationSwupComplete:
        {
            /*if receive stop update cmd here, the SWUP in activete status, deactivate SWUP is needed*/
            if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
            {
                if(phscaR4_SWUPDeactivate(R4Swup_SpiProtocolType) == PHSCAR4_RCI_CMD_STATUS_SUCCESS)
                {   //delay some time for device reboot
                    OSA_TimeDelay(100);
                }
            }
            else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
            { 
                if( phscaR4_SWUPUciDeactivate(R4Swup_SpiProtocolType) == uciStatusCode_Ok)
                {   
                    OSA_TimeDelay(100);                         
                }
            }
        }
        break;
        
        case R4_SWUP_TransferManifest:
        {
            if(mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_ACTIVE)
            {
                /*if receive stop update cmd here, the SWUP in activete status, deactivate SWUP is needed*/
                if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
                {
                     if(phscaR4_SWUPDeactivate(R4Swup_SpiProtocolType) == PHSCAR4_RCI_CMD_STATUS_SUCCESS)
                         OSA_TimeDelay(100);
                }
                else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
                { 
                     if( phscaR4_SWUPUciDeactivate(R4Swup_SpiProtocolType) == uciStatusCode_Ok)
                     {   //delay some time for device reboot
                         OSA_TimeDelay(100);                         
                     }
                }
            }
            else if(mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_TRANSFER)
            {
                goto FinishUpdate;
                //mR4DemoSwupAppStatus = R4_SWUP_FinishUpdate
            }
        }
        case R4_SWUP_StartUpdate:
        {}
        break;
        case R4_SWUP_TransferComponent:
        case R4_SWUP_VerifyComponent:
        case R4_SWUP_FinishUpdate:
        {
FinishUpdate:            
            if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
            {
                phscaR4_SWUPFinistUpdate();
            }
            else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
            { 
                phscaR4_SWUPUciFinistUpdate(); 
            }
            OSA_TimeDelay(100);
        }
        break;
        
        default:
        break;

    }
    R4Swup_StopSwupProcess(deviceId);
}


static void R4SWUP_UwbDeviceStateMachineHandler(deviceId_t deviceId, R4Swup_SerialCmdPacket_t * pPacket)
{
    static uint8_t errCnt = 0;
    uint32_t Status;
    phscaR4_HwDeviceInfo_t deviceinfo;  
    bool stopproc = false;
    
    switch(mR4DemoSwupAppStatus)
    {   
        case R4_SWUP_ResetDevice:
        {    
            R4Swup_ProcessOnGoing = false;
            if(phscaR4_UCIInit() == false)
            {
                if((errCnt++) > 5)
                {
                    errCnt = 0;
                   /* send error notification to serial/BLE */   
                   R4Swup_ErrorNotify(deviceId, R4SWUP_APP_STATE_RESET_FAILED);
                }
                else
                {
                    //delay 100ms and try again.
                    OSA_TimeDelay(100);
                }
                (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c);
            }
            else
            {
                errCnt = 0;
                mR4DemoSwupAppStatus = R4_SWUP_Idle;
            }
            break;
        }
        
        case R4_SWUP_Idle:
        {
            /* send confirm response to serial/BLE */   
            R4Swup_StartConfirm(deviceId);
            R4Swup_ProcessOnGoing = true;
            mR4DemoSwupAppStatus = R4_SWUP_StartProcess;
            mR4SwupUpdataImg.SwupStatus = PHSCAR4_SWUP_STATUS_INVALID;
        }
        break;
        
        case R4_SWUP_StartProcess:
        {
            /* Try send UCI SWUP Activate cmd to ranger4 first. In case of success of SWUP Activate cmd,
               NCJ29D5 device resets after 100 milli-seconds and activates SWUP, no reset notification send after reset.*/
            Status = phscaR4_SWUPUciActivate(UciAppCode_MacAppSupport, R4Swup_SpiProtocolType);
            if( Status == uciStatusCode_Ok)
            {
                OSA_TimeDelay(200);
                mR4DemoSwupAppStatus = R4_SWUP_ActivationSwupComplete;
                (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c);
            }
            else
            {
                /*  other error dule to communication error £¬ reset slave device */
                /* send error notification to serial/BLE */   
                R4Swup_ErrorNotify(deviceId, Status);
                R4Swup_StopSwupProcess(deviceId);
            }
        }
        break;
        
        case R4_SWUP_ActivationSwupComplete:
        {
            /* Get device info to check status of SWUP*/ 
            if( mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_INIT ||  mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_INVALID)
            {
                if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
                {
                    Status = phscaR4_SWUPGetDeviceInfo(&deviceinfo.Rci_Info, &mR4SwupUpdataImg.SwupStatus);                       
                }
                else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
                { 
                    Status = phscaR4_SWUPUciGetDeviceInfo(&deviceinfo.Uci_Info, &mR4SwupUpdataImg.SwupStatus);
                }
            }

            if((mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_INVALID) ||
               (mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_ERROR))
            {
                //Cmd response is not Success,check SPI communication
                R4Swup_ErrorNotify(deviceId, R4SWUP_APP_STATE_SWUP_ERROR);
                stopproc = true;
            }
            else if((mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_ACTIVE) || 
                    (mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_TRANSFER))
            {
                //whatever the SWUP is in ACTIVE or TRANSFER status, the host always need to get Manifest.
                mR4DemoSwupAppStatus = R4_SWUP_TransferManifest;
                stopproc = false;
                //get first pkt from serial
                mR4SwupUpdataImg.ReqPktNum = 0;         
                /* get new block */   
                R4Swup_GetNewPacket(deviceId);
            }
            else //(mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_INIT)
            {   /* wait device init complete */
                stopproc = false;
            }

            if(stopproc)
            {
                /* The SWUP in activete status, deactivate SWUP is needed*/
                if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
                {
                     if(phscaR4_SWUPDeactivate(R4Swup_SpiProtocolType) == PHSCAR4_RCI_CMD_STATUS_SUCCESS)
                         OSA_TimeDelay(100);
                }
                else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
                { 
                     if( phscaR4_SWUPUciDeactivate(R4Swup_SpiProtocolType) == uciStatusCode_Ok)
                     {   //delay some time for device reboot
                         OSA_TimeDelay(100);                         
                     }
                }
                R4Swup_StopSwupProcess(deviceId);
            }   
        }
        break;
        
        case R4_SWUP_TransferManifest:
        {
            mR4SwupUpdataImg.RcvPktNum = (uint16_t)pPacket->pkt.payload[0] | ((uint16_t)pPacket->pkt.payload[1] << 8);

            if(mR4SwupUpdataImg.ReqPktNum == mR4SwupUpdataImg.RcvPktNum)
            {
                if(mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_ACTIVE)
                {                              
                    if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
                    {
                        Status = phscaR4_SWUPTransferManifest((uint8_t)mR4SwupUpdataImg.ReqPktNum,
                                                              &pPacket->pkt.payload[2]);
                    }
                    else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
                    { 
                        Status = phscaR4_SWUPUciTransferManifest((uint8_t)mR4SwupUpdataImg.ReqPktNum,
                                                                 &pPacket->pkt.payload[2]); 
                    }
                }
         
                if(Status != PHSCAR4_RCI_CMD_STATUS_SUCCESS)
                {
                    R4Swup_ErrorNotify(deviceId, Status);
                    /* The SWUP in activete status, deactivate SWUP is needed*/
                    if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
                    {
                         if(phscaR4_SWUPDeactivate(R4Swup_SpiProtocolType) == PHSCAR4_RCI_CMD_STATUS_SUCCESS)
                             OSA_TimeDelay(100);
                    }
                    else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
                    { 
                         if( phscaR4_SWUPUciDeactivate(R4Swup_SpiProtocolType) == uciStatusCode_Ok)
                         {   /* Delay some time for device reboot */
                             OSA_TimeDelay(100);                         
                         }
                    }
                    R4Swup_StopSwupProcess(deviceId);
                }
                else
                {
                    //collect Component info from Manifest
                    R4Swup_CollectComponectInfo(&mR4SwupUpdataImg, pPacket);
                    //get next pkt
                    mR4SwupUpdataImg.ReqPktNum ++;
                    //first 4 packet of image is Manifest, ReqPktNum is 4 means all Manifest achieved
                    if(mR4SwupUpdataImg.ReqPktNum >= 4)
                    {
                        mR4DemoSwupAppStatus = R4_SWUP_StartUpdate;
                        mR4SwupUpdataImg.ReqPktNum = 0x0004;
                        (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c);
                    }
                    else
                    {   
                        R4Swup_GetNewPacket(deviceId); 
                    }
                }
            }
            else
            {
                /* Error occured, try get pkt again. */
                R4Swup_GetNewPacket(deviceId);
            }
        }
        break;
        
        case R4_SWUP_StartUpdate:
        {
            mR4DemoSwupAppStatus = R4_SWUP_TransferComponent;
            
            if(mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_ACTIVE)
            {   /* device in active state, send start update cmd*/
                if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
                {
                    Status = phscaR4_SWUPStartUpdate();
                }
                else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
                { 
                    Status = phscaR4_SWUPUciStartUpdate(); 
                }
                
                if(Status != PHSCAR4_RCI_CMD_STATUS_SUCCESS)
                {
                    R4Swup_ErrorNotify(deviceId, Status);
                    /* The SWUP in activete status, deactivate SWUP is needed*/
                    if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
                    {
                         if(phscaR4_SWUPDeactivate(R4Swup_SpiProtocolType) == PHSCAR4_RCI_CMD_STATUS_SUCCESS)
                             OSA_TimeDelay(100);
                    }
                    else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
                    { 
                         if( phscaR4_SWUPUciDeactivate(R4Swup_SpiProtocolType) == uciStatusCode_Ok)
                         {   //delay some time for device reboot
                             OSA_TimeDelay(100);                         
                         }
                    }
                    R4Swup_StopSwupProcess(deviceId);
                }
                else
                {
                    mR4SwupUpdataImg.ComponentId = 0;
                    mR4SwupUpdataImg.CurrentComponentPktIndex = 0;
                    mR4SwupUpdataImg.ComponentPkts = (mR4SwupUpdataImg.ComponentSize[mR4SwupUpdataImg.ComponentId] + PHSCAR4_UCISWUP_DATA_SIZE -1)/PHSCAR4_UCISWUP_DATA_SIZE;
                    //get first component pkt .
                    mR4SwupUpdataImg.ReqPktNum = 0x0004;
                    R4Swup_GetNewPacket(deviceId);
                }
            }
            else if(mR4SwupUpdataImg.SwupStatus == PHSCAR4_SWUP_STATUS_TRANSFER)
            {   /* device in transfer state, not need to send start update cmd*/
                mR4SwupUpdataImg.ComponentId = 0;
                mR4SwupUpdataImg.CurrentComponentPktIndex = 0;
                mR4SwupUpdataImg.ComponentPkts = (mR4SwupUpdataImg.ComponentSize[mR4SwupUpdataImg.ComponentId] + PHSCAR4_UCISWUP_DATA_SIZE -1)/PHSCAR4_UCISWUP_DATA_SIZE;
                //get first component pkt .
                mR4SwupUpdataImg.ReqPktNum = 0x0004;
                R4Swup_GetNewPacket(deviceId);
            } 
        }
        break;
        
        case R4_SWUP_TransferComponent:
        {
            mR4SwupUpdataImg.RcvPktNum = (uint16_t)pPacket->pkt.payload[0] | ((uint16_t)pPacket->pkt.payload[1] << 8);
            if(mR4SwupUpdataImg.ReqPktNum == mR4SwupUpdataImg.RcvPktNum)
            {   //send component data over SPI
                 if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
                 {
                     Status = phscaR4_SWUPTransferComponent(mR4SwupUpdataImg.ComponentId, 
                                                            mR4SwupUpdataImg.CurrentComponentPktIndex,
                                                            &pPacket->pkt.payload[2]);
                 }
                 else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
                 { 
                     Status = phscaR4_SWUPUciTransferComponent(mR4SwupUpdataImg.ComponentId, 
                                                               mR4SwupUpdataImg.CurrentComponentPktIndex,
                                                               &pPacket->pkt.payload[2]); 
                 }
                    
                 if(Status != PHSCAR4_RCI_CMD_STATUS_SUCCESS)
                 {
                     /* Error occurred, UWB device in transfer status, only finish update cmd can end the SWUP*/
                     mR4DemoSwupAppStatus = R4_SWUP_FinishUpdate;
                     R4Swup_ErrorNotify(deviceId, Status);
                     (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c);
                     break;
                 }   
                    
                 if(mR4SwupUpdataImg.CurrentComponentPktIndex < (mR4SwupUpdataImg.ComponentPkts -1))
                 { /* Get next component */
                     mR4SwupUpdataImg.CurrentComponentPktIndex ++;
                     mR4SwupUpdataImg.ReqPktNum++;
                     R4Swup_GetNewPacket(deviceId); 
                 }
                else
                {   /* One component data transfer complete, prepare transfer next component */
                    mR4SwupUpdataImg.ComponentId++;
                    mR4SwupUpdataImg.CurrentComponentPktIndex = 0;
                    mR4SwupUpdataImg.ComponentPkts = (mR4SwupUpdataImg.ComponentSize[mR4SwupUpdataImg.ComponentId] + PHSCAR4_UCISWUP_DATA_SIZE -1)/PHSCAR4_UCISWUP_DATA_SIZE;
                    
                    if(mR4SwupUpdataImg.ComponentId >= mR4SwupUpdataImg.ComponentCount)
                    {/* All component data transfer complete, verify it */
                        mR4DemoSwupAppStatus = R4_SWUP_VerifyComponent;
                        (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c);
                    }
                    else
                    {
                        mR4SwupUpdataImg.ReqPktNum++;
                        R4Swup_GetNewPacket(deviceId);
                    }
                }
            }
            else
            {   /* Error occured, try get pkt again */
                R4Swup_GetNewPacket(deviceId);
            }
        }
        break;
        
        case R4_SWUP_VerifyComponent:
        {
            if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
            {
                Status = phscaR4_SWUPVerifyAll();
            }
            else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
            { 
                Status = phscaR4_SWUPUciVerifyAll(); 
            }
            
            if(Status == PHSCAR4_RCI_CMD_STATUS_SUCCESS)
            {
                mR4DemoSwupAppStatus = R4_SWUP_FinishUpdate;  
                (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c); 
            }
            else
            {
                /* Error handle here, component verify error, re-transfer component*/
                mR4DemoSwupAppStatus = R4_SWUP_TransferComponent;
                mR4SwupUpdataImg.ComponentId = 0;
                mR4SwupUpdataImg.CurrentComponentPktIndex = 0;
                mR4SwupUpdataImg.ComponentPkts = (mR4SwupUpdataImg.ComponentSize[mR4SwupUpdataImg.ComponentId] + PHSCAR4_UCISWUP_DATA_SIZE -1)/PHSCAR4_UCISWUP_DATA_SIZE;
                /* get first component pkt */
                mR4SwupUpdataImg.ReqPktNum = 0x0004;
                R4Swup_GetNewPacket(deviceId);
            }
        }
        break;
        
        case R4_SWUP_FinishUpdate:
        {
            if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_RCI)
            {
                Status = phscaR4_SWUPFinistUpdate();
            }
            else if(R4Swup_SpiProtocolType == R4SWUP_SPI_PROTOCOL_UCI)
            { 
                Status = phscaR4_SWUPUciFinistUpdate();       
            }
            
            OSA_TimeDelay(100);
            R4Swup_TransferComplete(deviceId);
            mR4DemoSwupAppStatus = R4_SWUP_ResetDevice;
            (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c);
        }
        break;

    default:
        break;
    }
    
}


/*! -------------------------------------------------------------------------
 * \brief process RX pkt from serial
 * \param[in]    pPacket    received command pkt from serial(UART/SPI) or BLE.
 *---------------------------------------------------------------------------*/
static void R4SWUP_ProcessRxPkt(R4Swup_SerialCmdPacket_t * pPacket)
{
    switch(pPacket->pkt.Cmd)
    {
        case R4SWUP_UART_CMD_START_SWUP:
        {
            if(pPacket->pkt.CmdType == R4SWUP_UART_CMDTYPE_REQUEST)
            {
                R4SWUP_HandleStartSwupProcessCmd(0xFF, pPacket);
            }
            else
            {   /* This should never be sent by an SWUP client - Ignore */}
        }
        break;
        
        case R4SWUP_UART_CMD_GET_NEW_BLOCK:
        {
            if(pPacket->pkt.CmdType == R4SWUP_UART_CMDTYPE_REQUEST)
            { /* This should never be sent by an SWUP client - Ignore */ }
            else
            {
                R4SWUP_HandleImageBlockResponseCmd(0xFF, pPacket);
            }
        }
        break;
        
        case R4SWUP_UART_CMD_TRANSFER_COMPLETE:
        {
            /* This should never be sent by an SWUP client - Ignore */
        }
        break;
        
        case R4SWUP_UART_CMD_STOP_TRANSFER:
        {
            if(pPacket->pkt.CmdType == R4SWUP_UART_CMDTYPE_REQUEST)
            {
                R4SWUP_HandleStopProcessCmd(0xFF);
            }
            else
            {
               /* This should never be sent by an SWUP client - Ignore */ 
            }
        }
        break;
        
        default:
        break;
    }
    
}
    
                
static void R4Swup_CollectComponectInfo(R4Swup_UpdateImgStruct_t * ImageInfo, R4Swup_SerialCmdPacket_t * pPacket)
{
    uint8_t cnt = 0;;

    switch(ImageInfo->RcvPktNum)
    {
        case 0:
            cnt = 112;
            ImageInfo->ComponentCount = 0;
            ImageInfo->ComponentId = 0;
        break;
      
        case 1:
            cnt = 34;
            break;
                        
        case 2:
            cnt = 6;
            break;
                        
        case 3:
            cnt = 28;
            break;
                        
        default:
            return;
            break;
    }
    
    while(cnt<128)
    {
        if((pPacket->pkt.payload[cnt + 2] != 0xFF) && (pPacket->pkt.payload[cnt+2] < 0x03))
        {  
        ImageInfo->ComponentAddress[ImageInfo->ComponentCount] = phscaSTD_u32_ConvertU8toU32(pPacket->pkt.payload[cnt + 4],
                                                                                             pPacket->pkt.payload[cnt + 5],
                                                                                             pPacket->pkt.payload[cnt + 6],
                                                                                             pPacket->pkt.payload[cnt + 7]);
        
        ImageInfo->ComponentSize[ImageInfo->ComponentCount] = phscaSTD_u32_ConvertU8toU32(pPacket->pkt.payload[cnt + 8],
                                                                                          pPacket->pkt.payload[cnt + 9],
                                                                                          pPacket->pkt.payload[cnt + 10],
                                                                                          pPacket->pkt.payload[cnt + 11]);
        ImageInfo->ComponentCount++;
        }
        cnt += 50;
    }
}

static bool R4Swup_SendPktToSerial(uint32_t SerMgrIf, R4Swup_SerialCmdPacket_t *pPacket)
{
    uint16_t CrcValue = 0;
    uint16_t pktLen = 0;
    
    pPacket->pkt.CmdHead = SWUP_UART_CMD_KW2PC_MARKER;
    
    pktLen = (uint16_t)pPacket->pkt.CmdLen[0] | ((uint16_t)pPacket->pkt.CmdLen[1] << 8);
    //Calculate CRC
    CrcValue = phscaR4_calculateCRC16Software(&pPacket->raw_data[0], (pktLen + 1));
    
    //fill CRC to tx buffer
    pPacket->raw_data[pktLen + SWUP_UART_HEADER_SIZE - 2] = (uint8_t)(CrcValue & 0x00FF); 
    pPacket->raw_data[pktLen + SWUP_UART_HEADER_SIZE - 1] = (uint8_t)((CrcValue & 0xFF00)>>8); 

#if gDEMO_SWUP_NCJ29D5D_UART_TO_SPI    
    //(void)Serial_SyncWrite(SerMgrIf, pPacket, packetLen);
     if(gSerial_Success_c != Serial_AsyncWrite( SerMgrIf, &pPacket->raw_data[0], pktLen + SWUP_UART_HEADER_SIZE, (pSerialCallBack_t)MEM_BufferFree, pPacket))
     {
         (void)MEM_BufferFree(pPacket);
         return false;
     }
#elif gDEMO_SWUP_NCJ29D5D_SPI_TO_SPI
     if(kStatus_Success != slaveSpi_PrepareSend(&pPacket->raw_data[0], pktLen + SWUP_UART_HEADER_SIZE))
     {
         (void)MEM_BufferFree(pPacket);
         return false;
     }
     else
     {
         (void)MEM_BufferFree(pPacket);
     }
#endif
     return true;
}

static bool R4Swup_SendCmdToSerial(uint32_t SerMgrIf, uint8_t cmd, uint8_t * data)
{
    R4Swup_SerialCmdPacket_t * pCmdPkt = NULL;
    bool status = true;
    
    switch(cmd)
    {
        case R4SWUP_UART_CMD_START_SWUP:
        {
            //response start SWUP 
            pCmdPkt = MEM_BufferAlloc(SWUP_UART_HEADER_SIZE + 4); 
            if(pCmdPkt != NULL)
            {
                pCmdPkt->pkt.CmdLen[0] = 4;
                pCmdPkt->pkt.CmdLen[1] = 0;    
                pCmdPkt->pkt.CmdType = R4SWUP_UART_CMDTYPE_COMFIRM;
                pCmdPkt->pkt.Cmd = R4SWUP_UART_CMD_START_SWUP;
                //send response , buffer will free in this function
                status = R4Swup_SendPktToSerial(gAppSerMgrIf, pCmdPkt);                    
            }
            else
            {
                //handle error here
                status = false;
            }
            break;
        }
        
        case R4SWUP_UART_CMD_GET_NEW_BLOCK:
        {
            //request start SWUP 
            pCmdPkt = MEM_BufferAlloc(SWUP_UART_HEADER_SIZE + 6);
            if(pCmdPkt != NULL)
            {
                pCmdPkt->pkt.CmdLen[0] = 6;
                pCmdPkt->pkt.CmdLen[1] = 0;    
                pCmdPkt->pkt.CmdType = R4SWUP_UART_CMDTYPE_REQUEST;
                pCmdPkt->pkt.Cmd = R4SWUP_UART_CMD_GET_NEW_BLOCK;
                pCmdPkt->pkt.payload[0] = data[0]; 
                pCmdPkt->pkt.payload[1] = data[1]; 
                //send request , buffer will free in this function
                status = R4Swup_SendPktToSerial(gAppSerMgrIf, pCmdPkt);                    
            }
            else
            {
                //handle error here
                status = false;
            }
            break;
        }
        
        case R4SWUP_UART_CMD_TRANSFER_COMPLETE:
        {
            //comfirm transfer SWUP image complete 
            pCmdPkt = MEM_BufferAlloc(SWUP_UART_HEADER_SIZE + 5);
            if(pCmdPkt != NULL)
            {
                pCmdPkt->pkt.CmdLen[0] = 5;
                pCmdPkt->pkt.CmdLen[1] = 0;    
                pCmdPkt->pkt.CmdType = R4SWUP_UART_CMDTYPE_COMFIRM;
                pCmdPkt->pkt.Cmd = R4SWUP_UART_CMD_TRANSFER_COMPLETE;
                pCmdPkt->pkt.payload[0] = data[0];
                //send comfirm , buffer will free in this function
                status = R4Swup_SendPktToSerial(gAppSerMgrIf, pCmdPkt);                    
            }
            else
            {
                //handle error here
                status = false;
            }
            break;
        }
        
        case R4SWUP_UART_CMD_STOP_TRANSFER:
        {
            //response stop SWUP transfer
            pCmdPkt = MEM_BufferAlloc(SWUP_UART_HEADER_SIZE + 4);
            if(pCmdPkt != NULL)
            {
                pCmdPkt->pkt.CmdLen[0] = 4;
                pCmdPkt->pkt.CmdLen[1] = 0;    
                pCmdPkt->pkt.CmdType = R4SWUP_UART_CMDTYPE_COMFIRM;
                pCmdPkt->pkt.Cmd = R4SWUP_UART_CMD_STOP_TRANSFER;
                //send response , buffer will free in this function
                status = R4Swup_SendPktToSerial(gAppSerMgrIf, pCmdPkt);                    
            }
            else
            {
                //handle error here
                status = false;
            }
            break;
        }
        
        default:
            status = false;
            break;
    }
    
    return status;
}

static void R4Swup_ErrorNotify(deviceId_t deviceId, uint32_t error_state)
{
    uint32_t sta = error_state;
#if gDEMO_SWUP_NCJ29D5D_UART_TO_SPI || gDEMO_SWUP_NCJ29D5D_SPI_TO_SPI
    R4Swup_SendCmdToSerial(gAppSerMgrIf, R4SWUP_UART_CMD_ERROR_NOTIFY, (uint8_t *)&sta);    
#elif  gDEMO_SWUP_NCJ29D5D_BLE_TO_SPI
    if(deviceId != gInvalidDeviceId_c)
    {
        deviceId
    }
#endif
}

static void R4Swup_StartConfirm(deviceId_t deviceId)
{
#if gDEMO_SWUP_NCJ29D5D_UART_TO_SPI || gDEMO_SWUP_NCJ29D5D_SPI_TO_SPI
    R4Swup_SendCmdToSerial(gAppSerMgrIf, R4SWUP_UART_CMD_START_SWUP, NULL);    
#elif  gDEMO_SWUP_NCJ29D5D_BLE_TO_SPI
    if(deviceId != gInvalidDeviceId_c)
    {
        deviceId
    }
#endif  
}

                
static void R4Swup_GetNewPacket(deviceId_t deviceId)
{
#if gDEMO_SWUP_NCJ29D5D_UART_TO_SPI || gDEMO_SWUP_NCJ29D5D_SPI_TO_SPI 
    R4Swup_SendCmdToSerial(gAppSerMgrIf, R4SWUP_UART_CMD_GET_NEW_BLOCK, (uint8_t *)&mR4SwupUpdataImg.ReqPktNum);   
#elif  gDEMO_SWUP_NCJ29D5D_BLE_TO_SPI
    if(deviceId != gInvalidDeviceId_c)
    {
        deviceId
    }
#endif  
}
                
static void R4Swup_TransferComplete(deviceId_t deviceId)
{
#if gDEMO_SWUP_NCJ29D5D_UART_TO_SPI || gDEMO_SWUP_NCJ29D5D_SPI_TO_SPI 
    R4Swup_SendCmdToSerial(gAppSerMgrIf, R4SWUP_UART_CMD_TRANSFER_COMPLETE, NULL); 
#elif  gDEMO_SWUP_NCJ29D5D_BLE_TO_SPI
    if(deviceId != gInvalidDeviceId_c)
    {
        deviceId
    }
#endif  
}
                
//brief: switch process to reset state , send cmd to serial for notify PC
static void R4Swup_StopSwupProcess(deviceId_t deviceId) 
{
    mR4DemoSwupAppStatus = R4_SWUP_ResetDevice;
    R4Swup_ProcessOnGoing = false;

#if gDEMO_SWUP_NCJ29D5D_UART_TO_SPI || gDEMO_SWUP_NCJ29D5D_SPI_TO_SPI 
        R4Swup_SendCmdToSerial(gAppSerMgrIf, R4SWUP_UART_CMD_STOP_TRANSFER, NULL );   
#elif  gDEMO_SWUP_NCJ29D5D_BLE_TO_SPI
        if(deviceId != gInvalidDeviceId_c)
        {
            deviceId
        }
#endif
    (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtSelfNotify_c);
}


void R4SWUP_RxTimoutCallback(void *param)
{
    uint16_t readBytes;
    uint8_t c_byte;
    R4Swup_PacketRcvStatus_t status;
    R4Swup_SerialCmdPacket_t *pPkt = NULL;
    //Get one byte first from rx buffer
    if( gSerial_Success_c == Serial_GetByteFromRxBuffer( gAppSerMgrIf, &c_byte, &readBytes ) )
    {
        while(readBytes)
        {
            /* It is a new packet*/
            if(mR4SwupUartComm.bytesReceived == 0)
            {
                if(c_byte == SWUP_UART_CMD_PC2KW_MARKER)
                {
                    mR4SwupUartComm.bytesReceived++;
                    mR4SwupUartComm.Pkt.pkt.CmdHead = c_byte;
                    mR4SwupUartComm.bytesExpect = 0;
                }
            }
            else
            {
                status = R4SWUP_checkPacket(&mR4SwupUartComm, &c_byte, 1);
                if(status == Packet_Rcv_Ok)
                {    
                    pPkt = MEM_BufferAlloc((mR4SwupUartComm.bytesExpect + 3));
                    if(pPkt != NULL)
                    {
                        FLib_MemCpy(pPkt->raw_data, &mR4SwupUartComm.Pkt, (mR4SwupUartComm.bytesExpect + 3));
                        /* Put received valid packet in the mR4SwupRcvPktInputMsgQueue queue */                                
                        (void)MSG_Queue(&mR4SwupRcvPktInputMsgQueue, pPkt);
                        (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtNewPacketFromUart_c);
                    }
                    mR4SwupUartComm.bytesReceived = 0;
                    mR4SwupUartComm.bytesExpect = 0;
                    
                }
                else if(status == Packet_CrcError)
                {
                    mR4SwupUartComm.bytesReceived = 0;
                    mR4SwupUartComm.bytesExpect = 0;
                }
                else if(status == Packet_Unspecified_Error)
                {   
                    mR4SwupUartComm.bytesReceived = 0;
                    mR4SwupUartComm.bytesExpect = 0;
                }
                else //if(status == Packet_Rcv_Not_Complete)
                {}
            }
            //Get next byte from rx buffer    
            if ( gSerial_Success_c != Serial_GetByteFromRxBuffer( gAppSerMgrIf, &c_byte, &readBytes ) )
            {
                break;
            }
        }
    }
}

R4Swup_PacketRcvStatus_t R4SWUP_checkPacket(R4Swup_SerialComm_t *pData, uint8_t * data, uint8_t len)
{
    R4Swup_PacketRcvStatus_t status = Packet_Rcv_Not_Complete;
    uint16_t CrcValue_cal;
    uint16_t CrcValue_rcv; 
    if(pData == NULL)
    {
        return Packet_Unspecified_Error;
    }
    //put data to buffer
    FLib_MemCpy(&pData->Pkt.raw_data[pData->bytesReceived], data, len);
    pData->bytesReceived += len;
//    pData->uartPkt.raw_data[pData->bytesReceived++] = data;
    
    if(pData->bytesReceived < 3)
    {
        status = Packet_Rcv_Not_Complete; 
    }
    else 
    {
        if(pData->bytesExpect == 0)
        {
            pData->bytesExpect = (uint16_t)pData->Pkt.raw_data[1] | ((uint16_t)pData->Pkt.raw_data[2] << 8); 
        }
        
        if(pData->bytesReceived < pData->bytesExpect + 3 )
        {
            status = Packet_Rcv_Not_Complete;
        }
        else
        {
            /* If the length looks right, make sure that the checksum is correct. */
            //get crc value that received
            CrcValue_rcv = (uint16_t)pData->Pkt.raw_data[pData->bytesReceived-2] | 
                           ((uint16_t)pData->Pkt.raw_data[pData->bytesReceived-1] << 8);
            //calculate crc value
            CrcValue_cal = phscaR4_calculateCRC16Software(pData->Pkt.raw_data, pData->bytesReceived-2);
            if(CrcValue_rcv == CrcValue_cal)
            {
                status = Packet_Rcv_Ok;
            }
            else
            {
                status = Packet_CrcError;
            }
        }
    }

    return status;
}
   
bool R4SWUP_SetProtocolType(R4Swup_SpiProtocolType_t type)
{
    //The ProtocolType only can be set when process not on going
    if(!R4Swup_ProcessOnGoing)
    {
        R4Swup_SpiProtocolType = type;
        return true;
    }
    return false;
}

uint32_t R4SWUP_PostRcvPktToSwapTask( uint8_t * pRxData, uint32_t pktLen )
{
    R4Swup_SerialCmdPacket_t *pPkt = NULL;

    pPkt = MEM_BufferAlloc(pktLen);
    if(pPkt == NULL)
    {
        return 1;
    }

    FLib_MemCpy(pPkt->raw_data, pRxData, pktLen);
    /* Put received valid packet in the mR4SwupRcvPktInputMsgQueue queue */                                
    (void)MSG_Queue(&mR4SwupRcvPktInputMsgQueue, pPkt);
    (void)OSA_EventSet(mRanger4ThreadEventId, (uint32_t)gR4EvtMsgPacketFromSPI_c);

    return 0;
}
#endif

#endif  // for UWB_Support

